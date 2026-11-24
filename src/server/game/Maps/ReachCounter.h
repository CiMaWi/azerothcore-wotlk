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

#ifndef AZEROTHCORE_REACH_COUNTER_H
#define AZEROTHCORE_REACH_COUNTER_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class Map;
class Player;
class Unit;
class WorldObject;

// Lookups by id that reach far. While an entity updates on a continent (maps 0, 1, 530, 571), a lookup that finds
// an object on the same map farther than Cutoff (measured flat) is a far hit, filed under the lookup (the site) and
// the code that asked. Map::Update opens the task buffer, its update loops name the subject, the lookups check.
// Option MapUpdate.ReachCounter; off, a lookup tests one flag.
namespace ReachCounter
{
constexpr float Cutoff = 400.0f;

// The leaf lookups: a wrapper that delegates to one of them is counted there only
enum Site : std::size_t
{
    SiteCreature,            // Map::GetCreature
    SiteGameObject,          // Map::GetGameObject, also through Map::GetTransport
    SitePet,                 // Map::GetPet
    SiteCorpse,              // Map::GetCorpse
    SiteDynamicObject,       // Map::GetDynamicObject
    SitePlayerOnMap,         // ObjectAccessor::GetPlayer(Map const*, ...)
    SiteFindPlayer,          // ObjectAccessor::FindPlayer, also through FindPlayerByLowGUID
    SiteFindConnectedPlayer, // ObjectAccessor::FindConnectedPlayer
    SiteFindPlayerByName,    // ObjectAccessor::FindPlayerByName
    SiteSpawnedCreature,     // ObjectAccessor::GetSpawnedCreatureByDBGUID
    SiteSpawnedGameObject,   // ObjectAccessor::GetSpawnedGameObjectByDBGUID
    SiteCount
};

extern std::atomic<bool> active;

inline bool Enabled() { return active.load(std::memory_order_relaxed); }
void Configure(bool on);
char const* SiteName(std::size_t site);
// The maps whose updates are counted: the four continents, never an instance
bool CountsMap(Map const* map);

// Counts one lookup made while a subject updates (found may be null); out of line, called only while on
void Record(Site site, WorldObject const* found);

inline void Check(Site site, WorldObject const* found)
{
    if (Enabled())
        Record(site, found);
}

// One map task's buffer on this thread, merged into the shared table and the lightweight perf rows when it ends.
// Off, on another map, or inside another task scope it does nothing.
class TaskScope
{
public:
    explicit TaskScope(Map const* map)
    {
        if (Enabled())
            _on = Begin(map);
    }
    ~TaskScope()
    {
        if (_on)
            End();
    }
    TaskScope(TaskScope const&) = delete;
    TaskScope& operator=(TaskScope const&) = delete;

private:
    static bool Begin(Map const* map);
    static void End();

    bool _on = false;
};

// The entity whose update makes the lookups. Scopes nest: an inner one names its subject until it ends. Without
// a task scope on this thread it does nothing.
class Scope
{
public:
    explicit Scope(WorldObject const* subject)
    {
        if (Enabled())
            _on = Enter(subject, _previous);
    }
    ~Scope()
    {
        if (_on)
            Leave(_previous);
    }
    Scope(Scope const&) = delete;
    Scope& operator=(Scope const&) = delete;

private:
    static bool Enter(WorldObject const* subject, WorldObject const*& previous);
    static void Leave(WorldObject const* previous);

    WorldObject const* _previous = nullptr;
    bool _on = false;
};

// Far hits of this thread's open task scope so far, all sites; none open: 0
std::uint64_t TaskFarHits();

// Writes to the lists the whole map shares, counted while a map task runs on a continent; no task buffer on this
// thread (world thread, instances): not counted. Row count_<SharedName>: calls = writes made while the task buffer
// is open (inside a subject scope or not, e.g. the map's own timers), total = those made inside a subject scope,
// the writes that would race once subjects update side by side
enum SharedMember : std::size_t
{
    SharedObjectStoreInsert, // GetObjectsStore().Insert in AddToWorld (creature, pet, gameobject, dynobject, corpse)
    SharedObjectStoreErase,  // GetObjectsStore().Remove in RemoveFromWorld
    SharedUpdateListAdd,     // AddObjectToPendingUpdateList when it inserts, _AddObjectToUpdateList
    SharedUpdateListRemove,  // _RemoveObjectFromUpdateList (the swap-remove), the pending erase
    SharedRemoveList,        // Map::AddObjectToRemoveList
    SharedMoveList,          // Add{Creature,GameObject,DynamicObject}ToMoveList when it appends
    SharedUpdateObjects,     // Map::AddUpdateObject, Map::RemoveUpdateObject
    SharedGuid,              // Map::GenerateLowGuid
    SharedRespawn,           // Save/Remove{Creature,GO}RespawnTime
    SharedGridCreated,       // MapGridManager::CreateGrid when it creates
    SharedGridLoaded,        // MapGridManager::LoadGrid when it loads
    SharedCollisionRebuild,  // BIHWrap::balance when it rebuilds (the map's dynamic collision tree)
    SharedPathSearch,        // PathGenerator::CalculatePath on the map's own nav mesh query
    SharedCount
};

// The row name without "count_" ("shared_guid")
char const* SharedName(std::size_t member);
// Out of line, called only while on
void RecordShared(SharedMember member);

inline void Shared(SharedMember member)
{
    if (Enabled())
        RecordShared(member);
}

// A packet sent to a player's session while a subject updates (WorldSession::SendPacket). Row count_packet_far:
// calls = packets to a player other than the subject, total = those to one on the task's map farther than Cutoff
// (flat); far ones are filed with their caller and opcode like far hits
void RecordPacket(Player const* target, std::uint32_t opcode);

inline void CheckPacket(Player const* target, std::uint32_t opcode)
{
    if (Enabled())
        RecordPacket(target, opcode);
}

// This thread's open task scope so far; none open: 0. inScope: only the writes made inside a subject scope
std::uint64_t TaskShared(SharedMember member, bool inScope);
// farOnly: only the far ones
std::uint64_t TaskPackets(bool farOnly);

// The sampled pass (Map::Update): from here until the task ends, the outermost subject scopes are timed per search
// cell and the pass is appended to chunk-times.csv in the logs folder; without an open task buffer it does nothing
void SampleCells();

// The census of stored links: what a unit keeps of other objects (pointers and ids), by family. It walks the
// families the map-change cleanup tears down (Unit::RemoveFromWorld, CleanupBeforeRemoveFromMap); batch E turns
// the same walk into the cut of far links.
enum class LinkAction : std::uint8_t
{
    Count,
    Cut // Counted like Count until the cut exists
};

// Core families; the printed row is count_link_<name> (LinkFamilyName)
enum LinkFamily : std::size_t
{
    LinkVictim,      // Unit::GetVictim
    LinkAttacker,    // Unit::getAttackers
    LinkCombatPvE,   // CombatManager PvE references
    LinkCombatPvP,   // CombatManager PvP references
    LinkThreat,      // own threat list victims and the threat lists this unit is on
    LinkOwner,       // UNIT_FIELD_SUMMONEDBY
    LinkCharmer,     // UNIT_FIELD_CHARMEDBY
    LinkCharmed,     // UNIT_FIELD_CHARM
    LinkControlled,  // m_Controlled entries that are neither pet, totem nor guardian (charmed units)
    LinkPet,         // summon slot pet
    LinkTotem,       // the other summon slots (totems, mini-pet, quest summon)
    LinkGuardian,    // m_Controlled guardians and minions other than pets and totems
    LinkVehicle,     // the vehicle base ridden and the passengers of the own vehicle kit
    LinkSingleAura,  // single-target auras this unit cast: the unit the aura is on
    LinkAuraCaster,  // the caster of each aura applied to this unit
    LinkDynObj,      // own dynamic objects
    LinkGameObj,     // own game objects
    LinkFollow,      // the target of the current follow movement
    LinkFormation,   // the other members of the creature's formation
    LinkCombo,       // combo target and the units holding combo points on this one
    LinkSummoner,    // TempSummon summoner
    LinkGroupMember, // player: every other member of the group
    LinkSelection,   // player: the selected unit
    LinkViewpoint,   // player: the seer when it is not the player
    LinkFamilyCoreCount
};

// Core families and those modules register; LightweightPerf::LinkFamilyCount rows
constexpr std::size_t LinkFamilyMax = 32;

struct LinkTally
{
    std::array<std::uint32_t, LinkFamilyMax> seen{};
    std::array<std::uint32_t, LinkFamilyMax> distant{};
    std::array<float, LinkFamilyMax> farthest{}; // flat yd of the farthest far link
};

// Walks every stored link of the unit once. A link counts when its other end exists and is on the unit's map (one
// to another map is no reach); it is far when farther than Cutoff (flat). Reads only; its lookups are never counted
// as such. Works on any map: from a map task, or from the world thread for the console.
void Census(Unit& unit, LinkAction action, LinkTally& tally);
// One link of a walker outside Census's own list (a module)
void CountLink(LinkTally& tally, std::size_t family, WorldObject const& subject, WorldObject const* other);
// Module init, before the first map update: the row name without "count_" (e.g. "botlink_master"); full:
// LinkFamilyMax
std::size_t RegisterLinkFamily(char const* name);
// The row name without "count_" ("link_victim"); a free slot: nullptr
char const* LinkFamilyName(std::size_t family);
using LinkWalker = void (*)(Unit&, LinkAction, LinkTally&);
// Module init: Census calls each walker after its own list
void RegisterLinkWalker(LinkWalker walker);

// The console switch: while on, every pass on a continent takes the census, not only the sampled one
extern std::atomic<bool> censusOn;
void SetCensus(bool on);

inline bool CensusDue(bool sampledPass)
{
    return Enabled() && (sampledPass || censusOn.load(std::memory_order_relaxed));
}

// The tally of this thread's open task scope, written to the map's link rows when the task ends; none: nullptr
LinkTally* TaskTally();
// World thread: the far hits ranked by count, callers resolved to function and line; the whole table goes to
// tsvPath. Returns the lines to print (top lines of the table at most)
std::vector<std::string> Report(std::size_t top, std::string const& tsvPath);
void Reset();
}  // namespace ReachCounter

#endif
