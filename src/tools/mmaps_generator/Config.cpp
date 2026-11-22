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

#include "Config.h"
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fkYAML/node.hpp>
#include "PathCommon.h"
#include "TerrainBuilder.h"

namespace MMAP
{
    float ComputeBaseUnitDim(int vertexPerMapEdge)
    {
        return GRID_SIZE / static_cast<float>(vertexPerMapEdge);
    }

    std::pair<uint32, uint32> MakeTileKey(uint32 x, uint32 y)
    {
        return {x, y};
    }

    bool isCurrentDirectory(std::string const& pathStr) {
        try {
            const std::filesystem::path givenPath = std::filesystem::canonical(std::filesystem::absolute(pathStr));
            const std::filesystem::path currentPath = std::filesystem::canonical(std::filesystem::current_path());
            return givenPath == currentPath;
        } catch (std::filesystem::filesystem_error const& e) {
            std::cerr << "Filesystem error: " << e.what() << "\n";
            return false;
        }
    }

    MmapTileRecastConfig ResolvedMeshConfig::toMMAPTileRecastConfig() const {
        MmapTileRecastConfig config;
        config.walkableSlopeAngle = walkableSlopeAngle;
        config.walkableHeight = walkableHeight;
        config.walkableClimb = walkableClimb;
        config.walkableRadius = walkableRadius;
        config.maxSimplificationError = maxSimplificationError;
        config.cellSizeHorizontal = cellSizeHorizontal;
        config.cellSizeVertical = cellSizeVertical;
        config.baseUnitDim = baseUnitDim;
        config.vertexPerMapEdge = vertexPerMapEdge;
        config.vertexPerTileEdge = vertexPerTileEdge;
        config.tilesPerMapEdge = tilesPerMapEdge;
        return config;
    }

    std::optional<Config> Config::FromFile(std::string_view configFile) {
        Config config;
        if (!config.LoadConfig(configFile))
            return std::nullopt;

        return config;
    }

    Config::Config()
    {
    }

    ResolvedMeshConfig Config::GetConfigForTile(uint32 mapID, uint32 tileX, uint32 tileY) const
    {
        MapOverride const* mapOverride = nullptr;
        TileOverride const* tileOverride = nullptr;

        // Lookup map and tile overrides
        if (auto mapIt = _maps.find(mapID); mapIt != _maps.end())
        {
            mapOverride = &mapIt->second;

            auto tileIt = mapOverride->tileOverrides.find(MakeTileKey(tileY, tileX));
            if (tileIt != mapOverride->tileOverrides.end())
                tileOverride = &tileIt->second;
        }

        return ResolveConfig(mapOverride, tileOverride);
    }

    ResolvedMeshConfig Config::GetGlobalConfig() const
    {
        return ResolveConfig(nullptr, nullptr);
    }

    ResolvedMeshConfig Config::ResolveConfig(MapOverride const* mapOverride, TileOverride const* tileOverride) const
    {
        // Helper lambdas to resolve values in order: tile -> map -> global
        auto resolveFloat = [&](auto TileField, auto MapField, float GlobalValue) -> float {
            if (tileOverride && TileField(tileOverride)) return *TileField(tileOverride);
            if (mapOverride && MapField(mapOverride)) return *MapField(mapOverride);
            return GlobalValue;
        };

        auto resolveInt = [&](auto TileField, auto MapField, int GlobalValue) -> int {
            if (tileOverride && TileField(tileOverride)) return *TileField(tileOverride);
            if (mapOverride && MapField(mapOverride)) return *MapField(mapOverride);
            return GlobalValue;
        };

        // Resolve vertex settings
        int vertexPerMap = resolveInt(
            [](TileOverride const*) { return std::optional<int>{}; },
            [](MapOverride const* m) { return m->vertexPerMapEdge; },
            _global.vertexPerMapEdge
        );

        int vertexPerTile = resolveInt(
            [](TileOverride const*) { return std::optional<int>{}; },
            [](MapOverride const* m) { return m->vertexPerTileEdge; },
            _global.vertexPerTileEdge
        );

        ResolvedMeshConfig config;
        config.walkableSlopeAngle = resolveFloat(
            [](TileOverride const* t) { return t->walkableSlopeAngle; },
            [](MapOverride const* m) { return m->walkableSlopeAngle; },
            _global.walkableSlopeAngle
        );

        config.walkableRadius = resolveInt(
            [](TileOverride const* t) { return t->walkableRadius; },
            [](MapOverride const* m) { return m->walkableRadius; },
            _global.walkableRadius
        );

        config.walkableHeight = resolveInt(
            [](TileOverride const* t) { return t->walkableHeight; },
            [](MapOverride const* m) { return m->walkableHeight; },
            _global.walkableHeight
        );

        config.walkableClimb = resolveInt(
            [](TileOverride const* t) { return t->walkableClimb; },
            [](MapOverride const* m) { return m->walkableClimb; },
            _global.walkableClimb
        );

        config.vertexPerMapEdge = vertexPerMap;
        config.vertexPerTileEdge = vertexPerTile;
        config.baseUnitDim = ComputeBaseUnitDim(vertexPerMap);
        config.tilesPerMapEdge = vertexPerMap / vertexPerTile;
        config.maxSimplificationError = resolveFloat(
            [](TileOverride const* t) { return t->maxSimplificationError; },
            [](MapOverride const* m) { return m->maxSimplificationError; },
            _global.maxSimplificationError
        );
        config.cellSizeHorizontal = config.baseUnitDim;
        config.cellSizeVertical = config.baseUnitDim;

        if (mapOverride && mapOverride->cellSizeHorizontal.has_value())
            config.cellSizeHorizontal = *mapOverride->cellSizeHorizontal;

        if (mapOverride && mapOverride->cellSizeVertical.has_value())
            config.cellSizeVertical = *mapOverride->cellSizeVertical;

        return config;
    }

    bool Config::LoadConfig(std::string_view configFile) {
        FILE* f = std::fopen(configFile.data(), "r");
        if (!f)
            return false;

        fkyaml::node root = fkyaml::node::deserialize(f);
        std::fclose(f);

        if (!root.contains("mmapsConfig"))
            return false;

        fkyaml::node mmapsNode = root["mmapsConfig"];

        auto tryFloat = [](fkyaml::node const& n, char const* key, float& out)
        {
            if (n.contains(key)) out = n[key].get_value<float>();
        };
        auto tryInt = [](fkyaml::node const& n, char const* key, int& out)
        {
            if (n.contains(key)) out = n[key].get_value<int>();
        };
        auto tryBoolean = [](fkyaml::node const& n, char const* key, bool& out)
        {
            if (n.contains(key)) out = n[key].get_value<bool>();
        };
        auto tryString = [](fkyaml::node const& n, char const* key, std::string& out)
        {
            if (n.contains(key)) out = n[key].get_value<std::string>();
        };

        tryBoolean(mmapsNode, "skipLiquid", _skipLiquid);
        tryBoolean(mmapsNode, "skipContinents", _skipContinents);
        tryBoolean(mmapsNode, "skipJunkMaps", _skipJunkMaps);
        tryBoolean(mmapsNode, "skipBattlegrounds", _skipBattlegrounds);
        tryBoolean(mmapsNode, "debugOutput", _debugOutput);

        if (mmapsNode.contains("offmeshConnections") && mmapsNode["offmeshConnections"].is_sequence())
        {
            _offmeshConnections = mmapsNode["offmeshConnections"].get_value<std::vector<std::string>>();
        }
        else
        {
            _offmeshConnections.clear();
        }

        // Marked areas: { place, map, kind: walkable|blocked|plain, box: { x: [a, b], y: [a, b], z: [a, b] } } or
        // { place, map, kind, outline: [[x, y], ...], z: [a, b] }; an entry that does not read stops the load.
        _markedAreas.clear();
        if (mmapsNode.contains("markedAreas") && mmapsNode["markedAreas"].is_sequence())
        {
            auto readRange = [](fkyaml::node const& n, char const* key, float& lo, float& hi)
            {
                if (!n.contains(key) || !n[key].is_sequence())
                    return false;
                std::vector<float> const v = n[key].get_value<std::vector<float>>();
                if (v.size() != 2)
                    return false;
                lo = std::min(v[0], v[1]);
                hi = std::max(v[0], v[1]);
                return true;
            };

            for (fkyaml::node const& areaNode : mmapsNode["markedAreas"].as_seq())
            {
                MarkedArea area;
                std::string kind;
                tryString(areaNode, "place", area.place);
                tryString(areaNode, "kind", kind);
                bool ok = areaNode.contains("map");
                if (ok)
                    area.mapId = areaNode["map"].get_value<uint32>();
                if (kind == "walkable")
                    area.kind = MarkedAreaKind::Walkable;
                else if (kind == "blocked")
                    area.kind = MarkedAreaKind::Blocked;
                else if (kind == "plain")
                    area.kind = MarkedAreaKind::Plain;
                else
                    ok = false;

                if (ok && areaNode.contains("outline"))
                {
                    std::vector<std::vector<float>> const corners =
                        areaNode["outline"].get_value<std::vector<std::vector<float>>>();
                    ok = corners.size() >= 3 && readRange(areaNode, "z", area.minZ, area.maxZ);
                    for (std::size_t i = 0; ok && i < corners.size(); ++i)
                    {
                        ok = corners[i].size() == 2;
                        if (ok)
                            area.outline.emplace_back(corners[i][0], corners[i][1]);
                    }
                    if (ok)
                    {
                        area.minX = area.maxX = area.outline[0].first;
                        area.minY = area.maxY = area.outline[0].second;
                        for (auto const& [x, y] : area.outline)
                        {
                            area.minX = std::min(area.minX, x);
                            area.maxX = std::max(area.maxX, x);
                            area.minY = std::min(area.minY, y);
                            area.maxY = std::max(area.maxY, y);
                        }
                    }
                }
                else if (ok)
                {
                    ok = areaNode.contains("box");
                    if (ok)
                    {
                        fkyaml::node const& box = areaNode["box"];
                        ok = readRange(box, "x", area.minX, area.maxX) && readRange(box, "y", area.minY, area.maxY) &&
                             readRange(box, "z", area.minZ, area.maxZ);
                    }
                }

                if (!ok)
                {
                    printf("Marked area '%s' (map %u, kind '%s'): needs map, kind walkable|blocked|plain and a box "
                           "{x, y, z} or an outline of 3 or more [x, y] with z\n",
                           area.place.c_str(), area.mapId, kind.c_str());
                    return false;
                }
                _markedAreas.push_back(std::move(area));
            }
        }

        std::string dataDirPath;
        tryString(mmapsNode, "dataDir", dataDirPath);
        _dataDir = dataDirPath;

        mmapsNode = mmapsNode["meshSettings"];

        // Global config
        tryFloat(mmapsNode, "walkableSlopeAngle", _global.walkableSlopeAngle);
        tryInt(mmapsNode, "walkableHeight", _global.walkableHeight);
        tryInt(mmapsNode, "walkableClimb", _global.walkableClimb);
        tryInt(mmapsNode, "walkableRadius", _global.walkableRadius);
        tryInt(mmapsNode, "verticesPerMapEdge", _global.vertexPerMapEdge);
        tryInt(mmapsNode, "verticesPerTileEdge", _global.vertexPerTileEdge);
        tryFloat(mmapsNode, "maxSimplificationError", _global.maxSimplificationError);

        // Map overrides
        if (mmapsNode.contains("mapsOverrides"))
        {
            fkyaml::node maps = mmapsNode["mapsOverrides"];
            for (auto const& mapEntry : maps.as_map())
            {
                uint32 mapId = std::stoi(mapEntry.first.as_str());

                MapOverride override;
                fkyaml::node mapNode = mapEntry.second;

                if (mapNode.contains("walkableSlopeAngle"))
                    override.walkableSlopeAngle = mapNode["walkableSlopeAngle"].get_value<float>();
                if (mapNode.contains("walkableRadius"))
                    override.walkableRadius = mapNode["walkableRadius"].get_value<int>();
                if (mapNode.contains("walkableHeight"))
                    override.walkableHeight = mapNode["walkableHeight"].get_value<int>();
                if (mapNode.contains("walkableClimb"))
                    override.walkableClimb = mapNode["walkableClimb"].get_value<int>();
                if (mapNode.contains("verticesPerMapEdge"))
                    override.vertexPerMapEdge = mapNode["verticesPerMapEdge"].get_value<int>();
                if (mapNode.contains("verticesPerTileEdge"))
                    override.vertexPerTileEdge = mapNode["verticesPerTileEdge"].get_value<int>();
                if (mapNode.contains("cellSizeHorizontal"))
                    override.cellSizeHorizontal = mapNode["cellSizeHorizontal"].get_value<float>();
                if (mapNode.contains("cellSizeVertical"))
                    override.cellSizeVertical = mapNode["cellSizeVertical"].get_value<float>();
                if (mapNode.contains("maxSimplificationError"))
                    override.maxSimplificationError = mapNode["maxSimplificationError"].get_value<float>();

                // Tile overrides
                if (mapNode.contains("tilesOverrides"))
                {
                    fkyaml::node tiles = mapNode["tilesOverrides"];
                    for (auto const& tileEntry : tiles.as_map())
                    {
                        std::string key = tileEntry.first.as_str();
                        fkyaml::node tileNode = tileEntry.second;

                        size_t comma = key.find(',');
                        if (comma == std::string::npos)
                            continue;

                        uint32 tileX = static_cast<uint32>(std::stoi(key.substr(0, comma)));
                        uint32 tileY = static_cast<uint32>(std::stoi(key.substr(comma + 1)));

                        TileOverride tileOverride;
                        if (tileNode.contains("walkableSlopeAngle"))
                            tileOverride.walkableSlopeAngle = tileNode["walkableSlopeAngle"].get_value<float>();
                        if (tileNode.contains("walkableRadius"))
                            tileOverride.walkableRadius = tileNode["walkableRadius"].get_value<int>();
                        if (tileNode.contains("walkableHeight"))
                            tileOverride.walkableHeight = tileNode["walkableHeight"].get_value<int>();
                        if (tileNode.contains("walkableClimb"))
                            tileOverride.walkableClimb = tileNode["walkableClimb"].get_value<int>();
                        if (tileNode.contains("maxSimplificationError"))
                            tileOverride.maxSimplificationError = tileNode["maxSimplificationError"].get_value<float>();

                        override.tileOverrides[{tileX, tileY}] = std::move(tileOverride);
                    }
                }

                _maps[mapId] = std::move(override);
            }
        }

        // Resolve data dir path. Maybe we need to use an executable path instead of the current dir.
        if (isCurrentDirectory(_dataDir.string()) && !std::filesystem::exists(MapsPath()))
            if (auto execPath = std::filesystem::path(executableDirectoryPath()); std::filesystem::exists(execPath/ "maps"))
                _dataDir = execPath;

        return true;
    }

    bool Config::MarkedArea::Contains(float x, float y, float z) const
    {
        if (x < minX || x > maxX || y < minY || y > maxY || z < minZ || z > maxZ)
            return false;
        if (outline.empty())
            return true;

        // even-odd rule over the outline's edges
        bool inside = false;
        for (std::size_t i = 0, j = outline.size() - 1; i < outline.size(); j = i++)
        {
            auto const& [xi, yi] = outline[i];
            auto const& [xj, yj] = outline[j];
            if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi)
                inside = !inside;
        }
        return inside;
    }
}
