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

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class Map;
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
// World thread: the far hits ranked by count, callers resolved to function and line; the whole table goes to
// tsvPath. Returns the lines to print (top lines of the table at most)
std::vector<std::string> Report(std::size_t top, std::string const& tsvPath);
void Reset();
}  // namespace ReachCounter

#endif
