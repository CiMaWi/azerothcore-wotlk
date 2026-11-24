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

#ifndef AZEROTHCORE_LIGHTWEIGHT_PERF_H
#define AZEROTHCORE_LIGHTWEIGHT_PERF_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace LightweightPerf
{
struct Row
{
    std::uint64_t calls = 0;
    std::uint64_t total = 0;
    std::uint64_t maximum = 0;

    void Add(std::uint64_t value);
    void Merge(Row const& other);
};

enum WorldPhase : std::size_t
{
    Update,
    Sessions,
    Maps,
    Scripts,
    Other,
    MapsWait,
    MapsDelayed,
    WorldCpu,
    WorldPhaseCount
};

constexpr std::size_t MapSectionCount = 11;
// ReachCounter::SiteCount, checked there
constexpr std::size_t ReachSiteCount = 11;
// ReachCounter::LinkFamilyMax (core and registered families), checked there
constexpr std::size_t LinkFamilyCount = 32;

// Why a non-player object is on a map's update list: the first reason that holds, in this order. Seen: a player
// has it in view. Cell: a player's marked cells cover it, but no player sees it. The reasons after those keep an
// object updating with no player near. Leaving: no reason holds any more, the recheck drops it.
enum ObjectKind : std::size_t
{
    CreatureSeen,
    CreatureCell,
    CreatureActive,
    CreatureCombat,
    CreatureSummon,
    CreatureWaypoint,
    CreatureEvade,
    CreatureFormation,
    CreatureLeaving,
    GameObjectSeen,
    GameObjectCell,
    GameObjectActive,
    GameObjectTransport,
    GameObjectLeaving,
    DynamicObjects,
    ObjectKindCount
};

// Where a unit's update spends its time, in the order Unit::Update and its callers run it. A sample, taken with the
// object kinds: a phase row counts the units and their time in that phase
enum UnitPhase : std::size_t
{
    PhaseEvents,
    PhaseNotify,
    PhaseSpells,
    PhaseCombat,
    PhaseMovement,
    PhaseAi,
    PhaseRegen,
    PhaseHooks,
    PhaseRest,
    UnitPhaseCount
};

// In combat or not, read before the update
enum UnitGroup : std::size_t
{
    PlayerFight,
    PlayerIdle,
    CreatureFight,
    CreatureIdle,
    UnitGroupCount
};

enum MapRow : std::size_t
{
    Task = MapSectionCount,
    Queue,
    Cpu,
    Ai,
    Pregate,
    // A sample, not a total: taken on the map update that rechecks the update list (every 30 s). ObjectsSampled
    // counts those updates and their object time; the kind rows count the objects and their update time
    ObjectsSampled,
    ObjectKinds,
    // Per UnitGroup the UnitPhase rows, then the delayed relocations of the same pass: per mover type its time, and
    // (calls only) the objects its notifiers visited
    UnitPhases = ObjectKinds + ObjectKindCount,
    VisibilityPlayer = UnitPhases + UnitGroupCount * UnitPhaseCount,
    VisibilityCreature,
    VisibilityCheckedPlayer,
    VisibilityCheckedCreature,
    Players,
    PlayerUpdates,
    CpuValid,
    CpuInvalid,
    OverflowTasks,
    // Pair counters (calls = all, total = hits): updates with a non-zero work mask, AI checks with live
    WorkMaskUpdates,
    LiveAiChecks,
    // Pair counters per ReachCounter::Site (calls = lookups made while a subject updates, total = far ones)
    ReachRows,
    // Census per ReachCounter link family (calls = links seen, total = far ones, maximum = farthest far one in yd)
    LinkRows = ReachRows + ReachSiteCount,
    MapRowCount = LinkRows + LinkFamilyCount
};

bool Enabled();
void Configure(bool enabled);
std::uint64_t QueuedAt();
void SetMapQueuedAt(std::uint64_t queuedAt);
void RecordMapSections(std::uint64_t const* sections, std::uint32_t players, std::uint32_t playerUpdates);
// True while a timed map task runs on this thread
bool MapTimed();
// One sampled pass over a map's update list: per ObjectKind the objects and their update time (ns)
void RecordObjectKinds(std::array<Row, ObjectKindCount> const& kinds);
// Files the time since the previous lap of the current UnitScope under the phase; no scope: returns
void UnitLap(UnitPhase phase);
// Adds visited objects to the current VisibilityScope; none: returns
void CountChecked(std::uint32_t objects);
// Merges count rows into the current map task's buffer; outside a task, into the interval of the map
void MergeMapRows(std::uint32_t mapId, std::size_t first, Row const* rows, std::size_t count);
// World thread only: it reads the world update-time table, whose percentiles sort it without a lock.
std::string Snapshot(bool reset, bool metricsEnabled);

// One task owns its buffer. Only completion merges into the bounded shared interval.
class MapScope
{
public:
    explicit MapScope(std::uint32_t mapId);
    ~MapScope();
    MapScope(MapScope const&) = delete;
    MapScope& operator=(MapScope const&) = delete;

    void Add(std::size_t row, std::uint64_t value) { _rows[row].Add(value); }
    void Merge(std::size_t row, Row const& other) { _rows[row].Merge(other); }

private:
    bool _on;
    bool _cpuValid = false;
    std::uint32_t _mapId;
    std::uint64_t _epoch = 0;
    std::uint64_t _start = 0;
    std::uint64_t _cpuStart = 0;
    std::array<Row, MapRowCount> _rows;
};

// One sampled unit update on a timed map task. Scopes do not nest: one made while another is current is off
class UnitScope
{
public:
    UnitScope(bool on, UnitGroup group) : _group(group)
    {
        if (on)
            Begin();
    }

    ~UnitScope()
    {
        if (_on)
            End();
    }

    UnitScope(UnitScope const&) = delete;
    UnitScope& operator=(UnitScope const&) = delete;

    void Lap(UnitPhase phase);

private:
    void Begin();
    void End();

    bool _on = false;
    UnitGroup _group;
    MapScope* _task = nullptr;
    std::uint64_t _last = 0;
    std::array<std::uint64_t, UnitPhaseCount> _ns{};
};

// One sampled delayed relocation: its time and the objects its notifiers visit, by mover type
class VisibilityScope
{
public:
    VisibilityScope(bool on, bool player) : _player(player)
    {
        if (on)
            Begin();
    }

    ~VisibilityScope()
    {
        if (_on)
            End();
    }

    VisibilityScope(VisibilityScope const&) = delete;
    VisibilityScope& operator=(VisibilityScope const&) = delete;

    void Count(std::uint32_t objects) { _checked += objects; }

private:
    void Begin();
    void End();

    bool _on = false;
    bool _player;
    MapScope* _task = nullptr;
    std::uint64_t _start = 0;
    std::uint64_t _checked = 0;
};

class AiScope
{
public:
    AiScope();
    ~AiScope();
    AiScope(AiScope const&) = delete;
    AiScope& operator=(AiScope const&) = delete;

    void MarkPregate();
    void Count(MapRow row, bool hit);

private:
    MapScope* _task;
    std::uint64_t _start;
};

class WorldScope
{
public:
    WorldScope();
    ~WorldScope();
    WorldScope(WorldScope const&) = delete;
    WorldScope& operator=(WorldScope const&) = delete;
    void Add(WorldPhase phase, std::uint64_t value) { _rows[phase].Add(value); }

private:
    bool _on;
    bool _cpuValid = false;
    std::uint64_t _epoch = 0;
    std::uint64_t _start = 0;
    std::uint64_t _cpuStart = 0;
    std::uint64_t _tscStart = 0;
    std::array<Row, WorldPhaseCount> _rows;
};

class WorldPhaseScope
{
public:
    explicit WorldPhaseScope(WorldPhase phase);
    ~WorldPhaseScope();
    WorldPhaseScope(WorldPhaseScope const&) = delete;
    WorldPhaseScope& operator=(WorldPhaseScope const&) = delete;

private:
    WorldScope* _world;
    WorldPhase _phase;
    std::uint64_t _start;
};
}  // namespace LightweightPerf

#endif
