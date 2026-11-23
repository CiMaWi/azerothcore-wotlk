/*
 * This file is part of the AzerothCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "LightweightPerf.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <locale>
#include <mutex>
#include <sstream>

#include "UpdateTime.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <intrin.h>
#include <windows.h>
#else
#include <ctime>
#endif

namespace LightweightPerf
{
namespace
{
constexpr std::size_t MaxMaps = 128;
struct MapRows
{
    std::uint32_t id = 0;
    std::array<Row, MapRowCount> rows{};
};

struct Interval
{
    std::array<Row, WorldPhaseCount> world{};
    std::array<MapRows, MaxMaps + 1> maps{};
    std::size_t mapCount = 0;
};

std::atomic<bool> enabled{false};
std::atomic<std::uint64_t> epoch{0};
std::mutex intervalMutex;
Interval interval;
std::uint64_t since = 0;
thread_local MapScope* activeMap = nullptr;
thread_local WorldScope* activeWorld = nullptr;
thread_local UnitScope* activeUnit = nullptr;
thread_local VisibilityScope* activeVisibility = nullptr;
thread_local std::uint64_t mapQueuedAt = 0;

std::uint64_t Now()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool ThreadCpu(std::uint64_t& nanoseconds)
{
#ifdef _WIN32
    FILETIME creation, exit, kernel, user;
    if (!GetThreadTimes(GetCurrentThread(), &creation, &exit, &kernel, &user))
        return false;

    std::uint64_t const kernelTicks = (std::uint64_t(kernel.dwHighDateTime) << 32) | kernel.dwLowDateTime;
    std::uint64_t const userTicks = (std::uint64_t(user.dwHighDateTime) << 32) | user.dwLowDateTime;
    nanoseconds = (kernelTicks + userTicks) * 100;
    return true;
#elif defined(CLOCK_THREAD_CPUTIME_ID)
    timespec time{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time))
        return false;

    nanoseconds = std::uint64_t(time.tv_sec) * 1000000000 + std::uint64_t(time.tv_nsec);
    return true;
#else
    (void)nanoseconds;
    return false;
#endif
}

// The world row's clock: Windows counts the thread's TSC cycles (GetThreadTimes is quantised to the scheduler tick),
// scaled to nanoseconds by the TSC rate over the same scope; elsewhere the thread clock in nanoseconds.
bool PreciseCpu(std::uint64_t& value)
{
#ifdef _WIN32
    ULONG64 cycles = 0;
    if (!QueryThreadCycleTime(GetCurrentThread(), &cycles))
        return false;

    value = cycles;
    return true;
#else
    return ThreadCpu(value);
#endif
}

std::uint64_t Tsc()
{
#ifdef _WIN32
    return __rdtsc();
#else
    return 0;
#endif
}

constexpr char const* WorldNames[] = {"update", "sessions",  "maps",         "scripts",
                                      "other",  "maps_wait", "maps_delayed", "cpu"};
constexpr char const* MapNames[] = {"sessions",
                                    "player_update",
                                    "events",
                                    "respawns",
                                    "mark_cells",
                                    "non_player_objects",
                                    "send_object_updates",
                                    "scripts",
                                    "moves",
                                    "delayed_visibility",
                                    "rest",
                                    "task",
                                    "queue",
                                    "cpu",
                                    "ai",
                                    "pregate",
                                    "objects_sampled",
                                    "objects_creature_seen",
                                    "objects_creature_cell",
                                    "objects_creature_active",
                                    "objects_creature_combat",
                                    "objects_creature_summon",
                                    "objects_creature_waypoint",
                                    "objects_creature_evade",
                                    "objects_creature_formation",
                                    "objects_creature_leaving",
                                    "objects_gameobject_seen",
                                    "objects_gameobject_cell",
                                    "objects_gameobject_active",
                                    "objects_gameobject_transport",
                                    "objects_gameobject_leaving",
                                    "objects_dynamicobject",
                                    "unit_player_fight_events",
                                    "unit_player_fight_notify",
                                    "unit_player_fight_spells",
                                    "unit_player_fight_combat",
                                    "unit_player_fight_movement",
                                    "unit_player_fight_ai",
                                    "unit_player_fight_regen",
                                    "unit_player_fight_hooks",
                                    "unit_player_fight_rest",
                                    "unit_player_idle_events",
                                    "unit_player_idle_notify",
                                    "unit_player_idle_spells",
                                    "unit_player_idle_combat",
                                    "unit_player_idle_movement",
                                    "unit_player_idle_ai",
                                    "unit_player_idle_regen",
                                    "unit_player_idle_hooks",
                                    "unit_player_idle_rest",
                                    "unit_creature_fight_events",
                                    "unit_creature_fight_notify",
                                    "unit_creature_fight_spells",
                                    "unit_creature_fight_combat",
                                    "unit_creature_fight_movement",
                                    "unit_creature_fight_ai",
                                    "unit_creature_fight_regen",
                                    "unit_creature_fight_hooks",
                                    "unit_creature_fight_rest",
                                    "unit_creature_idle_events",
                                    "unit_creature_idle_notify",
                                    "unit_creature_idle_spells",
                                    "unit_creature_idle_combat",
                                    "unit_creature_idle_movement",
                                    "unit_creature_idle_ai",
                                    "unit_creature_idle_regen",
                                    "unit_creature_idle_hooks",
                                    "unit_creature_idle_rest",
                                    "visibility_player",
                                    "visibility_creature",
                                    "visibility_checked_player",
                                    "visibility_checked_creature",
                                    "count_players",
                                    "count_player_updates",
                                    "count_cpu_valid",
                                    "count_cpu_invalid",
                                    "count_overflow_tasks",
                                    "count_work_mask_updates",
                                    "count_live_ai_checks"};
static_assert(std::size(WorldNames) == WorldPhaseCount);
static_assert(std::size(MapNames) == MapRowCount);

void WriteRow(std::ostringstream& out, bool& first, std::string const& name, Row const& row, bool counter,
              bool always = false)
{
    if (!row.calls && !always)
        return;

    if (!first)
        out << ',';
    first = false;
    double const divisor = counter ? 1.0 : 1000.0;
    out << "{\"name\":\"" << name << "\",\"calls\":" << row.calls << ",\"total_us\":" << row.total / divisor
        << ",\"max_us\":" << row.maximum / divisor << '}';
}
}  // namespace

void Row::Add(std::uint64_t value)
{
    ++calls;
    total += value;
    maximum = std::max(maximum, value);
}

void Row::Merge(Row const& other)
{
    calls += other.calls;
    total += other.total;
    maximum = std::max(maximum, other.maximum);
}

bool Enabled() { return enabled.load(std::memory_order_relaxed); }

void Configure(bool on)
{
    std::lock_guard<std::mutex> guard(intervalMutex);
    if (on == Enabled())
        return;

    enabled.store(on, std::memory_order_relaxed);
    epoch.fetch_add(1, std::memory_order_relaxed);
    interval = {};
    since = Now();
}

std::uint64_t QueuedAt() { return Enabled() ? Now() : 0; }

void SetMapQueuedAt(std::uint64_t queuedAt) { mapQueuedAt = queuedAt; }

MapScope::MapScope(std::uint32_t mapId) : _on(Enabled()), _mapId(mapId)
{
    std::uint64_t const queuedAt = mapQueuedAt;
    mapQueuedAt = 0;
    if (!_on)
        return;

    _rows = {};
    _epoch = epoch.load(std::memory_order_relaxed);
    _start = Now();
    _cpuValid = ThreadCpu(_cpuStart);
    _rows[Queue].Add(queuedAt && queuedAt <= _start ? _start - queuedAt : 0);
    activeMap = this;
}

MapScope::~MapScope()
{
    if (!_on)
        return;

    std::uint64_t cpuEnd = 0;
    bool const valid = ThreadCpu(cpuEnd) && _cpuValid && cpuEnd >= _cpuStart;
    _rows[Task].Add(Now() - _start);
    if (valid)
        _rows[Cpu].Add(cpuEnd - _cpuStart);
    _rows[CpuValid].Add(valid ? 1 : 0);
    _rows[CpuInvalid].Add(valid ? 0 : 1);
    activeMap = nullptr;

    std::lock_guard<std::mutex> guard(intervalMutex);
    if (_epoch != epoch.load(std::memory_order_relaxed) || !Enabled())
        return;

    std::size_t index = 0;
    while (index < interval.mapCount && interval.maps[index].id != _mapId)
        ++index;
    if (index == interval.mapCount && index < MaxMaps)
    {
        interval.maps[index].id = _mapId;
        ++interval.mapCount;
    }
    if (index == MaxMaps)
        _rows[OverflowTasks].Add(1);

    auto& target = interval.maps[index].rows;
    for (std::size_t i = 0; i < MapRowCount; ++i)
        target[i].Merge(_rows[i]);
}

void RecordMapSections(std::uint64_t const* sections, std::uint32_t players, std::uint32_t playerUpdates)
{
    if (!activeMap)
        return;

    for (std::size_t i = 0; i < MapSectionCount; ++i)
        activeMap->Add(i, sections[i]);
    activeMap->Add(Players, players);
    activeMap->Add(PlayerUpdates, playerUpdates);
}

bool MapTimed() { return activeMap != nullptr; }

void RecordObjectKinds(std::array<Row, ObjectKindCount> const& kinds)
{
    if (!activeMap)
        return;

    std::uint64_t total = 0;
    for (std::size_t i = 0; i < ObjectKindCount; ++i)
    {
        activeMap->Merge(ObjectKinds + i, kinds[i]);
        total += kinds[i].total;
    }
    activeMap->Add(ObjectsSampled, total);
}

void UnitLap(UnitPhase phase)
{
    if (activeUnit)
        activeUnit->Lap(phase);
}

void UnitScope::Begin()
{
    if (!activeMap || activeUnit)
        return;

    _on = true;
    _task = activeMap;
    _last = Now();
    activeUnit = this;
}

void UnitScope::Lap(UnitPhase phase)
{
    std::uint64_t const now = Now();
    _ns[phase] += now - _last;
    _last = now;
}

void UnitScope::End()
{
    Lap(PhaseRest);
    std::size_t const first = UnitPhases + _group * UnitPhaseCount;
    for (std::size_t i = 0; i < UnitPhaseCount; ++i)
        _task->Add(first + i, _ns[i]);
    activeUnit = nullptr;
}

void CountChecked(std::uint32_t objects)
{
    if (activeVisibility)
        activeVisibility->Count(objects);
}

void VisibilityScope::Begin()
{
    if (!activeMap || activeVisibility)
        return;

    _on = true;
    _task = activeMap;
    _start = Now();
    activeVisibility = this;
}

void VisibilityScope::End()
{
    _task->Add(_player ? VisibilityPlayer : VisibilityCreature, Now() - _start);
    _task->Merge(_player ? VisibilityCheckedPlayer : VisibilityCheckedCreature, Row{_checked, 0, 0});
    activeVisibility = nullptr;
}

// Bot AI runs inside a player's update, from a script hook: the hook time before it stays hooks, its own is ai
AiScope::AiScope() : _task(activeMap), _start(_task ? Now() : 0) { UnitLap(PhaseHooks); }

AiScope::~AiScope()
{
    if (_task)
        _task->Add(Ai, Now() - _start);
    UnitLap(PhaseAi);
}

void AiScope::MarkPregate()
{
    if (_task)
        _task->Add(Pregate, Now() - _start);
}

void AiScope::Count(MapRow row, bool hit)
{
    if (_task)
        _task->Add(row, hit ? 1 : 0);
}

WorldScope::WorldScope() : _on(Enabled())
{
    if (!_on)
        return;

    _rows = {};
    _epoch = epoch.load(std::memory_order_relaxed);
    _cpuValid = PreciseCpu(_cpuStart);
    _tscStart = Tsc();
    _start = Now();
    activeWorld = this;
}

WorldScope::~WorldScope()
{
    if (!_on)
        return;

    std::uint64_t const elapsed = Now() - _start;
    std::uint64_t const tscEnd = Tsc();
    std::uint64_t cpuEnd = 0;
    if (PreciseCpu(cpuEnd) && _cpuValid && cpuEnd >= _cpuStart)
    {
#ifdef _WIN32
        if (tscEnd > _tscStart)
            _rows[WorldCpu].Add(std::uint64_t(double(cpuEnd - _cpuStart) * elapsed / double(tscEnd - _tscStart)));
#else
        (void)tscEnd;
        _rows[WorldCpu].Add(cpuEnd - _cpuStart);
#endif
    }
    _rows[Update].Add(elapsed);
    std::uint64_t const phases = _rows[Sessions].total + _rows[Maps].total + _rows[Scripts].total;
    _rows[Other].Add(elapsed >= phases ? elapsed - phases : 0);
    activeWorld = nullptr;

    std::lock_guard<std::mutex> guard(intervalMutex);
    if (_epoch != epoch.load(std::memory_order_relaxed) || !Enabled())
        return;

    for (std::size_t i = 0; i < WorldPhaseCount; ++i)
        interval.world[i].Merge(_rows[i]);
}

WorldPhaseScope::WorldPhaseScope(WorldPhase phase) : _world(activeWorld), _phase(phase), _start(_world ? Now() : 0) {}

WorldPhaseScope::~WorldPhaseScope()
{
    if (_world)
        _world->Add(_phase, Now() - _start);
}

std::string Snapshot(bool reset, bool metricsEnabled)
{
    Interval result;
    std::uint64_t elapsed = 0;
    bool on;
    {
        std::lock_guard<std::mutex> guard(intervalMutex);
        std::uint64_t const now = Now();
        on = Enabled();
        if (reset)
            epoch.fetch_add(1, std::memory_order_relaxed);
        else
        {
            result = interval;
            elapsed = since && now >= since ? now - since : 0;
        }
        interval = {};
        since = now;
    }

    std::ostringstream out;
    out.imbue(std::locale::classic());
    out.precision(17);
    out << "{\"version\":1,\"enabled\":" << (on ? "true" : "false")
        << ",\"metrics_enabled\":" << (metricsEnabled ? "true" : "false") << ",\"elapsed_us\":" << elapsed / 1000.0
        << ",\"rows\":[";
    bool first = true;
    for (std::size_t i = 0; i < WorldPhaseCount; ++i)
        WriteRow(out, first, std::string("world/") + WorldNames[i], result.world[i], false);

    for (std::size_t m = 0; m <= MaxMaps; ++m)
    {
        if (m >= result.mapCount && m != MaxMaps)
            continue;
        std::string const id = m == MaxMaps ? "overflow" : std::to_string(result.maps[m].id);
        // Every map update is a full one. The row names keep "full", so stored intervals and the tools that read
        // them still match. The pair counters stand beside every ai row, also while nothing feeds them
        auto const& rows = result.maps[m].rows;
        for (std::size_t i = 0; i < MapRowCount; ++i)
            WriteRow(out, first, "map/" + id + "/full/" + MapNames[i], rows[i], i >= Players,
                     i >= WorkMaskUpdates && rows[Ai].calls);
    }
    out << ']';

    // The world update-time table (last AVG_DIFF_COUNT main-loop intervals, sleep included); a snapshot leaves it alone
    std::uint32_t const samples = sWorldUpdateTime.GetDatasetSize();
    out << ",\"tick\":{\"samples\":" << samples;
    if (samples)
        out << ",\"p50_ms\":" << sWorldUpdateTime.GetPercentile(50)
            << ",\"p80_ms\":" << sWorldUpdateTime.GetPercentile(80)
            << ",\"p95_ms\":" << sWorldUpdateTime.GetPercentile(95)
            << ",\"p99_ms\":" << sWorldUpdateTime.GetPercentile(99)
            << ",\"max_ms\":" << sWorldUpdateTime.GetPercentile(100);
    out << "}}";
    return out.str();
}
}  // namespace LightweightPerf
