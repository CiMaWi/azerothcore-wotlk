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
    WorldPhaseCount
};

constexpr std::size_t MapSectionCount = 11;
enum MapRow : std::size_t
{
    Task = MapSectionCount,
    Queue,
    Cpu,
    Ai,
    Players,
    PlayerUpdates,
    CpuValid,
    CpuInvalid,
    OverflowTasks,
    MapRowCount
};

bool Enabled();
void Configure(bool enabled);
std::uint64_t QueuedAt();
void SetMapQueuedAt(std::uint64_t queuedAt);
void RecordMapSections(std::uint64_t const* sections, bool full, std::uint32_t players, std::uint32_t playerUpdates);
std::string Snapshot(bool reset, bool metricsEnabled);

// One task owns its buffer. Only completion merges into the bounded shared interval.
class MapScope
{
public:
    MapScope(std::uint32_t mapId, bool full);
    ~MapScope();
    MapScope(MapScope const&) = delete;
    MapScope& operator=(MapScope const&) = delete;

    void Add(std::size_t row, std::uint64_t value) { _rows[row].Add(value); }

private:
    bool _on;
    bool _full;
    bool _cpuValid = false;
    std::uint32_t _mapId;
    std::uint64_t _epoch = 0;
    std::uint64_t _start = 0;
    std::uint64_t _cpuStart = 0;
    std::array<Row, MapRowCount> _rows;
};

class AiScope
{
public:
    AiScope();
    ~AiScope();
    AiScope(AiScope const&) = delete;
    AiScope& operator=(AiScope const&) = delete;

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
    std::uint64_t _epoch = 0;
    std::uint64_t _start = 0;
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
