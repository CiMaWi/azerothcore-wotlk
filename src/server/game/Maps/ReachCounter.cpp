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

#include "ReachCounter.h"

#include "AbstractFollower.h"
#include "CombatManager.h"
#include "Creature.h"
#include "CreatureGroups.h"
#include "DynamicObject.h"
#include "Group.h"
#include "GroupReference.h"
#include "LightweightPerf.h"
#include "Map.h"
#include "MotionMaster.h"
#include "MovementGenerator.h"
#include "Object.h"
#include "ObjectAccessor.h"
#include "Player.h"
#include "SpellAuras.h"
#include "StringFormat.h"
#include "TemporarySummon.h"
#include "ThreatManager.h"
#include "Unit.h"
#include "Vehicle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <utility>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <dbghelp.h>
#pragma comment(lib, "dbghelp.lib")
#endif

namespace ReachCounter
{
std::atomic<bool> active{false};

namespace
{
// The leaf lookup and the four frames above it: wrappers among them are dropped when the report resolves them
constexpr std::size_t FrameCount = 5;
constexpr std::size_t TaskHits = 64;
constexpr std::size_t MaxHits = 4096;

constexpr char const* SiteNames[] = {"creature",
                                     "gameobject",
                                     "pet",
                                     "corpse",
                                     "dynamic_object",
                                     "player_on_map",
                                     "find_player",
                                     "find_connected_player",
                                     "find_player_by_name",
                                     "spawned_creature",
                                     "spawned_gameobject"};
static_assert(std::size(SiteNames) == SiteCount);
static_assert(SiteCount == LightweightPerf::ReachSiteCount);
static_assert(LinkFamilyMax == LightweightPerf::LinkFamilyCount);

constexpr char const* CoreLinkNames[] = {"link_victim",
                                         "link_attacker",
                                         "link_combat_pve",
                                         "link_combat_pvp",
                                         "link_threat",
                                         "link_owner",
                                         "link_charmer",
                                         "link_charmed",
                                         "link_controlled",
                                         "link_pet",
                                         "link_totem",
                                         "link_guardian",
                                         "link_vehicle",
                                         "link_single_aura",
                                         "link_aura_caster",
                                         "link_dynobj",
                                         "link_gameobj",
                                         "link_follow",
                                         "link_formation",
                                         "link_combo",
                                         "link_summoner",
                                         "link_group_member",
                                         "link_selection",
                                         "link_viewpoint"};
static_assert(std::size(CoreLinkNames) == LinkFamilyCoreCount);
static_assert(LinkFamilyCoreCount <= LinkFamilyMax);

constexpr std::size_t MaxLinkWalkers = 8;

using Frames = std::array<std::uintptr_t, FrameCount>;
using Key = std::pair<std::size_t, Frames>;

// One caller's far hits, with the farthest one as the example
struct Hit
{
    std::size_t site = 0;
    Frames frames{};
    std::uint64_t count = 0;
    float maximum = 0.0f;
    std::uint32_t mapId = 0;
    std::uint8_t subjectType = 0;
    std::uint8_t foundType = 0;
    std::uint32_t subjectEntry = 0;
    std::uint32_t foundEntry = 0;

    void Merge(Hit const& other)
    {
        std::uint64_t const sum = count + other.count;
        if (other.maximum > maximum)
            *this = other;
        count = sum;
    }
};

struct Buffer
{
    Map const* map = nullptr;
    std::uint32_t mapId = 0;
    std::array<std::uint64_t, SiteCount> calls{};
    std::array<std::uint64_t, SiteCount> distant{};
    std::array<Hit, TaskHits> hits{};
    std::size_t hitCount = 0;
    std::uint64_t lost = 0;
    // Zeroed when TaskTally first hands it out in this task, so a task without a census costs nothing
    LinkTally links;
    bool linksUsed = false;
};

thread_local Buffer taskBuffer;
thread_local Buffer* currentTask = nullptr;
thread_local WorldObject const* currentSubject = nullptr;

// Module families and walkers: written at module init before the first map update, published by the counts; read
// by the census and the snapshot
std::mutex registryMutex;
std::array<std::string, LinkFamilyMax - LinkFamilyCoreCount> moduleLinkNames;
std::atomic<std::size_t> linkFamilies{LinkFamilyCoreCount};
std::array<LinkWalker, MaxLinkWalkers> linkWalkers{};
std::atomic<std::size_t> linkWalkerCount{0};

std::mutex tableMutex;
std::map<Key, Hit> table;
std::array<std::uint64_t, SiteCount> totalCalls{};
std::array<std::uint64_t, SiteCount> totalFar{};
// Far hits counted but not filed under a caller: a task's or the shared table was full
std::uint64_t lostHits = 0;

char const* TypeName(std::uint8_t type)
{
    switch (type)
    {
        case TYPEID_UNIT:
            return "creature";
        case TYPEID_PLAYER:
            return "player";
        case TYPEID_GAMEOBJECT:
            return "gameobject";
        case TYPEID_DYNAMICOBJECT:
            return "dynamic_object";
        case TYPEID_CORPSE:
            return "corpse";
        default:
            return "object";
    }
}

struct Frame
{
    std::string text;
    bool wrapper = false;
};

// The lookups themselves and their wrappers: the report names the code above them
bool IsWrapper(std::string const& function)
{
    static constexpr char const* Prefixes[] = {"ReachCounter::", "ObjectAccessor::Get", "ObjectAccessor::Find"};
    static constexpr char const* Leaves[] = {"Map::GetCreature", "Map::GetGameObject",     "Map::GetPet",
                                             "Map::GetCorpse",   "Map::GetDynamicObject", "Map::GetTransport"};
    for (char const* prefix : Prefixes)
        if (function.rfind(prefix, 0) == 0)
            return true;
    for (char const* leaf : Leaves)
        if (function == leaf)
            return true;
    return false;
}

class Resolver
{
public:
    Resolver()
    {
#ifdef _WIN32
        _process = GetCurrentProcess();
        _options = SymGetOptions();
        SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES);
        _symbols = SymInitialize(_process, nullptr, TRUE) != FALSE;
#endif
    }

    // The crash report initializes the symbol handler itself: leave it free
    ~Resolver()
    {
#ifdef _WIN32
        if (_symbols)
            SymCleanup(_process);
        SymSetOptions(_options);
#endif
    }

    Resolver(Resolver const&) = delete;
    Resolver& operator=(Resolver const&) = delete;

    Frame const& Resolve(std::uintptr_t address)
    {
        auto const known = _frames.find(address);
        if (known != _frames.end())
            return known->second;

        Frame frame;
        std::string function;
        std::string where;
#ifdef _WIN32
        if (_symbols)
        {
            alignas(SYMBOL_INFO) char storage[sizeof(SYMBOL_INFO) + MAX_SYM_NAME] = {};
            SYMBOL_INFO* symbol = reinterpret_cast<SYMBOL_INFO*>(storage);
            symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
            symbol->MaxNameLen = MAX_SYM_NAME;
            DWORD64 displacement = 0;
            if (SymFromAddr(_process, address, &displacement, symbol))
                function = symbol->Name;

            // A return address points behind the call
            IMAGEHLP_LINE64 line = {};
            line.SizeOfStruct = sizeof(line);
            DWORD lineDisplacement = 0;
            if (SymGetLineFromAddr64(_process, address - 1, &lineDisplacement, &line) && line.FileName)
            {
                std::string file = line.FileName;
                std::size_t const slash = file.find_last_of("\\/");
                if (slash != std::string::npos)
                    file = file.substr(slash + 1);
                where = Acore::StringFormat("{}:{}", file, line.LineNumber);
            }
        }
#endif
        frame.wrapper = !function.empty() && IsWrapper(function);
        if (function.empty())
            frame.text = Acore::StringFormat("0x{:x}", address);
        else if (where.empty())
            frame.text = function;
        else
            frame.text = Acore::StringFormat("{} ({})", function, where);
        return _frames.emplace(address, std::move(frame)).first->second;
    }

private:
    std::unordered_map<std::uintptr_t, Frame> _frames;
#ifdef _WIN32
    HANDLE _process = nullptr;
    DWORD _options = 0;
    bool _symbols = false;
#endif
};

struct Group
{
    std::size_t site = 0;
    std::string caller;
    std::string next;
    Hit best;
    std::uint64_t count = 0;
};
}  // namespace

void Configure(bool on) { active.store(on, std::memory_order_relaxed); }

char const* SiteName(std::size_t site) { return site < SiteCount ? SiteNames[site] : "unknown"; }

bool CountsMap(Map const* map)
{
    if (!map || map->Instanceable())
        return false;

    std::uint32_t const id = map->GetId();
    return id == 0 || id == 1 || id == 530 || id == 571;
}

void Record(Site site, WorldObject const* found)
{
    Buffer* const task = currentTask;
    WorldObject const* const subject = currentSubject;
    if (!task || !subject)
        return;

    ++task->calls[site];
    // A player from the global store may be on another map: only one on this task's map can be far
    if (!found || found->FindMap() != task->map)
        return;

    float const dx = found->GetPositionX() - subject->GetPositionX();
    float const dy = found->GetPositionY() - subject->GetPositionY();
    float const squared = dx * dx + dy * dy;
    if (squared <= Cutoff * Cutoff)
        return;

    ++task->distant[site];
    Hit hit;
    hit.site = site;
#ifdef _WIN32
    void* raw[FrameCount] = {};
    USHORT const captured = CaptureStackBackTrace(1, static_cast<DWORD>(FrameCount), raw, nullptr);
    for (USHORT i = 0; i < captured; ++i)
        hit.frames[i] = reinterpret_cast<std::uintptr_t>(raw[i]);
#endif
    hit.count = 1;
    hit.maximum = std::sqrt(squared);
    hit.mapId = task->mapId;
    hit.subjectType = static_cast<std::uint8_t>(subject->GetTypeId());
    hit.subjectEntry = subject->GetEntry();
    hit.foundType = static_cast<std::uint8_t>(found->GetTypeId());
    hit.foundEntry = found->GetEntry();

    for (std::size_t i = 0; i < task->hitCount; ++i)
    {
        Hit& filed = task->hits[i];
        if (filed.site == hit.site && filed.frames == hit.frames)
        {
            filed.Merge(hit);
            return;
        }
    }
    if (task->hitCount < TaskHits)
        task->hits[task->hitCount++] = hit;
    else
        ++task->lost;
}

bool TaskScope::Begin(Map const* map)
{
    if (currentTask || !CountsMap(map))
        return false;

    Buffer& buffer = taskBuffer;
    buffer.map = map;
    buffer.mapId = map->GetId();
    buffer.calls.fill(0);
    buffer.distant.fill(0);
    buffer.hitCount = 0;
    buffer.lost = 0;
    buffer.linksUsed = false;
    currentSubject = nullptr;
    currentTask = &buffer;
    return true;
}

void TaskScope::End()
{
    Buffer const& buffer = taskBuffer;
    currentTask = nullptr;
    currentSubject = nullptr;

    if (buffer.linksUsed)
    {
        std::array<LightweightPerf::Row, LinkFamilyMax> links{};
        bool anyLink = false;
        for (std::size_t family = 0; family < LinkFamilyMax; ++family)
        {
            if (!buffer.links.seen[family])
                continue;
            links[family].calls = buffer.links.seen[family];
            links[family].total = buffer.links.distant[family];
            links[family].maximum = static_cast<std::uint64_t>(buffer.links.farthest[family]);
            anyLink = true;
        }
        if (anyLink)
            LightweightPerf::MergeMapRows(buffer.mapId, LightweightPerf::LinkRows, links.data(), links.size());
    }

    std::array<LightweightPerf::Row, SiteCount> rows{};
    bool any = false;
    for (std::size_t site = 0; site < SiteCount; ++site)
    {
        if (!buffer.calls[site])
            continue;
        rows[site].calls = buffer.calls[site];
        rows[site].total = buffer.distant[site];
        rows[site].maximum = buffer.distant[site] ? 1 : 0;
        any = true;
    }
    if (!any)
        return;

    LightweightPerf::MergeMapRows(buffer.mapId, LightweightPerf::ReachRows, rows.data(), rows.size());

    std::lock_guard<std::mutex> guard(tableMutex);
    for (std::size_t site = 0; site < SiteCount; ++site)
    {
        totalCalls[site] += buffer.calls[site];
        totalFar[site] += buffer.distant[site];
    }
    lostHits += buffer.lost;
    for (std::size_t i = 0; i < buffer.hitCount; ++i)
    {
        Hit const& hit = buffer.hits[i];
        Key const key(hit.site, hit.frames);
        auto const filed = table.find(key);
        if (filed != table.end())
            filed->second.Merge(hit);
        else if (table.size() < MaxHits)
            table.emplace(key, hit);
        else
            lostHits += hit.count;
    }
}

bool Scope::Enter(WorldObject const* subject, WorldObject const*& previous)
{
    if (!currentTask)
        return false;

    previous = currentSubject;
    currentSubject = subject;
    return true;
}

void Scope::Leave(WorldObject const* previous) { currentSubject = previous; }

std::uint64_t TaskFarHits()
{
    Buffer const* const task = currentTask;
    std::uint64_t sum = 0;
    if (task)
        for (std::uint64_t distant : task->distant)
            sum += distant;
    return sum;
}

std::atomic<bool> censusOn{false};

void SetCensus(bool on) { censusOn.store(on, std::memory_order_relaxed); }

LinkTally* TaskTally()
{
    Buffer* const task = currentTask;
    if (!task)
        return nullptr;

    if (!task->linksUsed)
    {
        task->links = LinkTally();
        task->linksUsed = true;
    }
    return &task->links;
}

std::size_t RegisterLinkFamily(char const* name)
{
    std::lock_guard<std::mutex> guard(registryMutex);
    std::size_t const family = linkFamilies.load(std::memory_order_relaxed);
    if (!name || family >= LinkFamilyMax)
        return LinkFamilyMax;

    moduleLinkNames[family - LinkFamilyCoreCount] = name;
    linkFamilies.store(family + 1, std::memory_order_release);
    return family;
}

char const* LinkFamilyName(std::size_t family)
{
    if (family < LinkFamilyCoreCount)
        return CoreLinkNames[family];
    if (family < linkFamilies.load(std::memory_order_acquire))
        return moduleLinkNames[family - LinkFamilyCoreCount].c_str();
    return nullptr;
}

void RegisterLinkWalker(LinkWalker walker)
{
    std::lock_guard<std::mutex> guard(registryMutex);
    std::size_t const count = linkWalkerCount.load(std::memory_order_relaxed);
    if (!walker || count >= MaxLinkWalkers)
        return;

    linkWalkers[count] = walker;
    linkWalkerCount.store(count + 1, std::memory_order_release);
}

void CountLink(LinkTally& tally, std::size_t family, WorldObject const& subject, WorldObject const* other)
{
    if (!other || other == &subject || family >= LinkFamilyMax)
        return;

    // A link to another map is no reach; only an end on the subject's own map is read further
    Map const* const map = subject.FindMap();
    if (!map || other->FindMap() != map)
        return;

    ++tally.seen[family];
    float const dx = other->GetPositionX() - subject.GetPositionX();
    float const dy = other->GetPositionY() - subject.GetPositionY();
    float const squared = dx * dx + dy * dy;
    if (squared <= Cutoff * Cutoff)
        return;

    ++tally.distant[family];
    tally.farthest[family] = std::max(tally.farthest[family], std::sqrt(squared));
}

void Census(Unit& unit, LinkAction action, LinkTally& tally)
{
    if (!unit.IsInWorld())
        return;

    Map* const map = unit.FindMap();
    if (!map)
        return;

    // The census's own lookups are not the subject's: a null subject leaves them uncounted
    Scope quiet(nullptr);
    auto const link = [&](LinkFamily family, WorldObject const* other) { CountLink(tally, family, unit, other); };
    auto const linkUnit = [&](LinkFamily family, ObjectGuid const& guid)
    {
        if (guid)
            link(family, ObjectAccessor::GetUnit(unit, guid));
    };
    auto const linkObject = [&](LinkFamily family, ObjectGuid const& guid)
    {
        if (guid)
            link(family, ObjectAccessor::GetWorldObject(unit, guid));
    };

    link(LinkVictim, unit.GetVictim());
    for (Unit const* attacker : unit.getAttackers())
        link(LinkAttacker, attacker);

    CombatManager const& combat = unit.GetCombatManager();
    for (auto const& [guid, reference] : combat.GetPvECombatRefs())
        link(LinkCombatPvE, reference->GetOther(&unit));
    for (auto const& [guid, reference] : combat.GetPvPCombatRefs())
        link(LinkCombatPvP, reference->GetOther(&unit));

    ThreatManager const& threat = unit.GetThreatMgr();
    if (threat.GetThreatListSize())
        for (ThreatReference const* reference : threat.GetUnsortedThreatList())
            link(LinkThreat, reference->GetVictim());
    for (auto const& [guid, reference] : threat.GetThreatenedByMeList())
        link(LinkThreat, reference->GetOwner());

    linkUnit(LinkOwner, unit.GetOwnerGUID());
    linkUnit(LinkCharmer, unit.GetCharmerGUID());
    linkUnit(LinkCharmed, unit.GetCharmGUID());
    for (Unit const* controlled : unit.m_Controlled)
    {
        if (controlled->IsPet() || controlled->IsTotem())
            continue;
        link(controlled->HasUnitTypeMask(UNIT_MASK_GUARDIAN | UNIT_MASK_MINION) ? LinkGuardian : LinkControlled,
             controlled);
    }

    linkUnit(LinkPet, unit.GetPetGUID());
    for (std::size_t slot = SUMMON_SLOT_TOTEM_FIRE; slot < MAX_SUMMON_SLOT; ++slot)
        linkUnit(LinkTotem, unit.m_SummonSlot[slot]);

    link(LinkVehicle, unit.GetVehicleBase());
    if (Vehicle const* kit = unit.GetVehicleKit())
        for (auto const& [seatId, seat] : kit->Seats)
            linkUnit(LinkVehicle, seat.Passenger.Guid);

    for (Aura const* aura : unit.GetSingleCastAuras())
        link(LinkSingleAura, aura->GetOwner());
    for (auto const& [spellId, application] : unit.GetAppliedAuras())
    {
        ObjectGuid const caster = application->GetBase()->GetCasterGUID();
        if (caster != unit.GetGUID())
            linkObject(LinkAuraCaster, caster);
    }

    for (DynamicObject const* dynObject : unit.GetDynObjects())
        link(LinkDynObj, dynObject);
    for (ObjectGuid const& guid : unit.GetGameObjects())
        link(LinkGameObj, map->GetGameObject(guid));

    MotionMaster const* motion = unit.GetMotionMaster();
    if (motion->GetCurrentMovementGeneratorType() == FOLLOW_MOTION_TYPE)
        if (auto const* follower = dynamic_cast<AbstractFollower const*>(motion->top()))
            link(LinkFollow, follower->GetTarget());

    if (Creature const* creature = unit.ToCreature())
        if (CreatureGroup const* formation = creature->GetFormation())
            for (auto const& [member, info] : formation->GetMembers())
                link(LinkFormation, member);

    link(LinkCombo, unit.GetComboTarget());
    for (Unit const* holder : unit.GetComboPointHolders())
        link(LinkCombo, holder);

    if (TempSummon const* summon = unit.ToTempSummon())
        linkObject(LinkSummoner, summon->GetSummonerGUID());

    if (Player const* player = unit.ToPlayer())
    {
        if (::Group const* group = player->GetGroup())
            for (GroupReference const* member = group->GetFirstMember(); member; member = member->next())
                link(LinkGroupMember, member->GetSource());
        linkUnit(LinkSelection, player->GetTarget());
        link(LinkViewpoint, player->GetSeer());
    }

    std::size_t const walkers = linkWalkerCount.load(std::memory_order_acquire);
    for (std::size_t i = 0; i < walkers; ++i)
        linkWalkers[i](unit, action, tally);
}

void Reset()
{
    std::lock_guard<std::mutex> guard(tableMutex);
    table.clear();
    totalCalls.fill(0);
    totalFar.fill(0);
    lostHits = 0;
}

std::vector<std::string> Report(std::size_t top, std::string const& tsvPath)
{
    std::vector<Hit> hits;
    std::array<std::uint64_t, SiteCount> calls{};
    std::array<std::uint64_t, SiteCount> distant{};
    std::uint64_t lost = 0;
    {
        std::lock_guard<std::mutex> guard(tableMutex);
        hits.reserve(table.size());
        for (auto const& [key, hit] : table)
            hits.push_back(hit);
        calls = totalCalls;
        distant = totalFar;
        lost = lostHits;
    }

    // Entries that resolve to the same two callers are one line
    std::map<std::tuple<std::size_t, std::string, std::string>, Group> groups;
    {
        Resolver resolver;
        for (Hit const& hit : hits)
        {
            std::array<std::string, 2> callers;
            std::size_t found = 0;
            for (std::uintptr_t address : hit.frames)
            {
                if (!address || found == callers.size())
                    continue;
                Frame const& frame = resolver.Resolve(address);
                if (!frame.wrapper)
                    callers[found++] = frame.text;
            }

            Group& group = groups[std::make_tuple(hit.site, callers[0], callers[1])];
            if (!group.count || hit.maximum > group.best.maximum)
                group.best = hit;
            group.site = hit.site;
            group.caller = callers[0];
            group.next = callers[1];
            group.count += hit.count;
        }
    }

    std::vector<Group const*> ranked;
    ranked.reserve(groups.size());
    for (auto const& [key, group] : groups)
        ranked.push_back(&group);
    std::sort(ranked.begin(), ranked.end(), [](Group const* a, Group const* b) { return a->count > b->count; });

    std::vector<std::string> lines;
    std::string sites;
    for (std::size_t site = 0; site < SiteCount; ++site)
        if (calls[site])
            sites += Acore::StringFormat(" {} {}/{}", SiteNames[site], distant[site], calls[site]);
    lines.push_back(Acore::StringFormat("reach: far/lookups by site:{}; unfiled far hits {}; {} callers",
                                        sites.empty() ? " none" : sites, lost, ranked.size()));

    std::ofstream tsv(tsvPath, std::ios::trunc);
    if (tsv)
        tsv << "count\tsite\tmax_yd\tcaller\tnext_caller\tmap\tsubject_type\tsubject_entry\tfound_type\tfound_entry\n";
    for (std::size_t i = 0; i < ranked.size(); ++i)
    {
        Group const& group = *ranked[i];
        Hit const& best = group.best;
        if (tsv)
            tsv << group.count << '\t' << SiteNames[group.site] << '\t' << static_cast<std::uint32_t>(best.maximum)
                << '\t' << group.caller << '\t' << group.next << '\t' << best.mapId << '\t'
                << TypeName(best.subjectType) << '\t' << best.subjectEntry << '\t' << TypeName(best.foundType)
                << '\t' << best.foundEntry << '\n';
        if (i < top)
            lines.push_back(Acore::StringFormat("{} x {} max {:.0f} yd: {} <- {} (map {}: {} {} -> {} {})",
                                                group.count, SiteNames[group.site], best.maximum, group.caller,
                                                group.next.empty() ? "-" : group.next, best.mapId,
                                                TypeName(best.subjectType), best.subjectEntry,
                                                TypeName(best.foundType), best.foundEntry));
    }
    lines.push_back(tsv ? Acore::StringFormat("reach: table written to {}", tsvPath)
                        : Acore::StringFormat("reach: could not write {}", tsvPath));
    return lines;
}
}  // namespace ReachCounter
