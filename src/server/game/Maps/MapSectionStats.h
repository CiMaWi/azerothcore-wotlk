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

#ifndef AZEROTHCORE_MAP_SECTION_STATS_H
#define AZEROTHCORE_MAP_SECTION_STATS_H

#include "Define.h"
#include <chrono>

// Section timing and block counters of one Map::Update, sent as metric values when it ends (metrics on only).
// A map task runs wholly on one map worker thread and Map::Update never nests, so a thread_local that Map::Update
// resets at its start and sends at its end belongs to that one map update: no locks, no races.

enum MapUpdateSection : uint8
{
    MAP_SECTION_SESSIONS,           // WorldSession::Update of every player
    MAP_SECTION_PLAYER_UPDATE,      // Player::Update of every player (short and full ticks)
    MAP_SECTION_EVENTS,             // Map::Events
    MAP_SECTION_RESPAWNS,           // due respawns
    MAP_SECTION_MARK_CELLS,         // cells marked around players and their viewpoints
    MAP_SECTION_NON_PLAYER_OBJECTS, // creature and object updates
    MAP_SECTION_SEND_OBJECT_UPDATES,
    MAP_SECTION_SCRIPTS,
    MAP_SECTION_MOVES,              // creature, gameobject and dynamic object move lists
    MAP_SECTION_DELAYED_VISIBILITY,
    MAP_SECTION_REST,               // dynamic tree, kick events, weather, corpses, the map update hook
    MAX_MAP_SECTIONS
};

// Index of the block counters: whether the target player's session has a socket.
enum MapBlockTarget : uint8
{
    MAP_BLOCK_SOCKET,
    MAP_BLOCK_NO_SOCKET,
    MAX_MAP_BLOCK_TARGETS
};

struct MapSectionStats
{
    bool on;                                     // inside a Map::Update with metrics enabled
    uint64 sectionNs[MAX_MAP_SECTIONS];          // nanoseconds, sent as microseconds
    uint32 playerUpdates;
    uint32 valuesBuilt[MAX_MAP_BLOCK_TARGETS];   // values blocks (Object::BuildFieldsUpdate)
    uint32 createBuilt[MAX_MAP_BLOCK_TARGETS];   // create blocks (Object::BuildCreateUpdateBlockForPlayer)
};

inline thread_local MapSectionStats mapSectionStats{};

// Lap clock for Map::Update: each Lap adds the time since the previous lap to a section. Off: no clock reads.
class MapSectionClock
{
public:
    explicit MapSectionClock(bool on) : _on(on)
    {
        if (_on)
            _last = std::chrono::steady_clock::now();
    }

    void Lap(MapUpdateSection section)
    {
        if (!_on)
            return;

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        std::chrono::nanoseconds const elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - _last);
        mapSectionStats.sectionNs[section] += uint64(elapsed.count());
        _last = now;
    }

private:
    bool _on;
    std::chrono::steady_clock::time_point _last{};
};

#endif
