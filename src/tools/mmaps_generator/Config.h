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

#ifndef CONFIG_H
#define CONFIG_H

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/program_options/options_description.hpp>
#include "Define.h"
#include "MapDefines.h"

namespace std
{
    template <>
    struct hash<std::pair<uint32_t, uint32_t>>
    {
        std::size_t operator()(std::pair<uint32_t, uint32_t> const& p) const noexcept
        {
            return std::hash<uint64_t>()((static_cast<uint64_t>(p.first) << 32) | p.second);
        }
    };
}

namespace MMAP
{
    struct ResolvedMeshConfig {
        float walkableSlopeAngle;
        int walkableRadius;
        int walkableHeight;
        int walkableClimb;
        int vertexPerMapEdge;
        int vertexPerTileEdge;
        int tilesPerMapEdge;
        float baseUnitDim;
        float cellSizeHorizontal;
        float cellSizeVertical;
        float maxSimplificationError;

        MmapTileRecastConfig toMMAPTileRecastConfig() const;
    };

    class Config {
    public:
        static std::optional<Config> FromFile(std::string_view configFile);

        ~Config() = default;

        ResolvedMeshConfig GetConfigForTile(uint32 mapID, uint32 tileX, uint32 tileY) const;

        // The global meshSettings alone, with no map or tile override: a vessel's tile (--vessels).
        ResolvedMeshConfig GetGlobalConfig() const;

        bool ShouldSkipLiquid() const { return _skipLiquid; }
        bool ShouldSkipContinents() const { return _skipContinents; }
        bool ShouldSkipJunkMaps() const { return _skipJunkMaps; }
        bool ShouldSkipBattlegrounds() const { return _skipBattlegrounds; }
        bool IsDebugOutputEnabled() const { return _debugOutput; }

        std::string VMapsPath() const { return (_dataDir / "vmaps").string(); }
        std::string MapsPath() const { return (_dataDir / "maps").string(); }
        std::string MMapsPath() const { return (_dataDir / "mmaps").string(); }
        std::string DataDirPath() const { return _dataDir.string(); }

        std::vector<std::string> const& OffMeshConnections() const { return _offmeshConnections; }

        // A marked area (mmapsConfig.markedAreas), in every tile the area touches (the tiles' border overlap keeps
        // them consistent). Walkable acts on the source triangles whose centroid lies inside and in the ledge filter;
        // blocked and plain on the cells of the compact heightfield whose centre and floor lie inside.
        enum class MarkedAreaKind : uint8
        {
            Walkable, // ground, also where the slope rule drops the triangle, tags it steep or the ledge filter cuts it
            Blocked,  // no mesh
            Plain     // ground instead of the steep tag; ground the slope rule dropped stays dropped
        };

        struct MarkedArea
        {
            std::string place;
            uint32 mapId = 0;
            MarkedAreaKind kind = MarkedAreaKind::Walkable;
            // world coordinates; with an outline, x and y are its bounds
            float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f, minZ = 0.0f, maxZ = 0.0f;
            std::vector<std::pair<float, float>> outline; // world (x, y) corners; empty: the box alone

            bool Contains(float x, float y, float z) const;
        };

        std::vector<MarkedArea> const& MarkedAreas() const { return _markedAreas; }

    private:
        explicit Config();

        bool LoadConfig(std::string_view configFile);

        struct TileOverride {
            std::optional<float> walkableSlopeAngle;
            std::optional<int> walkableRadius;
            std::optional<int> walkableHeight;
            std::optional<int> walkableClimb;
            std::optional<float> maxSimplificationError;
        };

        struct MapOverride {
            std::optional<float> walkableSlopeAngle;
            std::optional<int> walkableRadius;
            std::optional<int> walkableHeight;
            std::optional<int> walkableClimb;
            std::optional<int> vertexPerMapEdge;
            std::optional<int> vertexPerTileEdge;
            std::optional<float> maxSimplificationError;

            // The width/depth of each cell in the XZ-plane grid used for voxelization. [Units: world units]
            // A smaller value increases navmesh resolution but also memory and CPU usage.
            // Default is equal to calculated baseUnitDim.
            // Recast reference: https://github.com/recastnavigation/recastnavigation/blob/bd98d84c274ee06842bf51a4088ca82ac71f8c2d/Recast/Include/Recast.h#L231
            std::optional<float> cellSizeHorizontal;

            // The height of each cell in the Y-axis used for voxelization. [Units: world units]
            // Controls how vertical features are represented. Lower values improve accuracy for uneven terrain.
            // Default is equal to calculated baseUnitDim.
            // Recast reference: https://github.com/recastnavigation/recastnavigation/blob/bd98d84c274ee06842bf51a4088ca82ac71f8c2d/Recast/Include/Recast.h#L234
            std::optional<float> cellSizeVertical;

            std::unordered_map<std::pair<uint32, uint32>, TileOverride> tileOverrides;
        };

        struct GlobalConfig {
            // Maximum slope angle (in degrees) NPCs can walk on.
            // Surfaces steeper than this will be considered unwalkable.
            float walkableSlopeAngle = 60.0f;

            // Minimum distance (in cell units) around walkable surfaces.
            // Helps prevent NPCs from clipping into walls and narrow gaps.
            int walkableRadius = 2;

            // Minimum ceiling height (in cell units) NPCs need to pass under an obstacle.
            // Controls how much vertical clearance is required.
            int walkableHeight = 6;

            // Maximum height difference (in cell units) NPCs can step up or down.
            // Higher values allow walking over fences, ledges, or steps.
            int walkableClimb = 6;

            // Number of vertices along one edge of the entire map's navmesh grid.
            // Higher values increase mesh resolution but also CPU/memory usage.
            int vertexPerMapEdge = 2000;

            // Number of vertices along one edge of each tile chunk.
            // Must divide (vertexPerMapEdge - 1) evenly for seamless tiles.
            // A higher vertex count per tile means fewer total tiles,
            // reducing runtime work to load, unload, and manage tiles.
            int vertexPerTileEdge = 80;

            // Tolerance for how much a polygon can deviate from the original geometry when simplified.
            // Higher values produce simpler (faster) meshes but can reduce accuracy.
            float maxSimplificationError = 1.8f;
        };

        // Resolves each value in order: tile override, map override, global.
        ResolvedMeshConfig ResolveConfig(MapOverride const* mapOverride, TileOverride const* tileOverride) const;

        GlobalConfig _global;
        std::unordered_map<uint32, MapOverride> _maps;

        bool _skipLiquid;
        bool _skipContinents;
        bool _skipJunkMaps;
        bool _skipBattlegrounds;
        bool _debugOutput;

        std::filesystem::path _dataDir;

        std::vector<std::string> _offmeshConnections;
        std::vector<MarkedArea> _markedAreas;
    };
}

#endif //CONFIG_H
