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

#include "MMapMgr.h"
#include "Config.h"
#include "Errors.h"
#include "Log.h"
#include "MapDefines.h"
#include <algorithm>
#include <mutex>
#include <unordered_map>

namespace
{
    // The vessel meshes by display id. An entry without a mesh is a model that was asked for and has no usable file.
    std::mutex vesselNavMeshLock;
    std::unordered_map<uint32, std::shared_ptr<dtNavMesh const>> vesselNavMeshes;

    struct ConstNavMeshDeleter
    {
        void operator()(dtNavMesh const* navMesh) noexcept { dtFreeNavMesh(const_cast<dtNavMesh*>(navMesh)); }
    };

    // There is no .mmap for a vessel: its one tile carries the mesh's parameters and is loaded as a single-tile mesh.
    std::shared_ptr<dtNavMesh const> LoadVesselNavMesh(uint32 displayId)
    {
        std::string const fileName = Acore::StringFormat(MMAP::VESSEL_TILE_FILE_NAME_FORMAT, sConfigMgr->GetOption<std::string>("DataDir", "."), displayId);
        FILE* file = fopen(fileName.c_str(), "rb");
        if (!file)
        {
            // no file or no folder: the model has no mesh, its transports path as before
            LOG_DEBUG("maps", "MMAP:loadVessel: no vessel mesh file '{}'", fileName);
            return nullptr;
        }

        MmapTileHeader fileHeader;
        if (fread(&fileHeader, sizeof(MmapTileHeader), 1, file) != 1 || fileHeader.mmapMagic != MMAP_MAGIC ||
            fileHeader.dtVersion != uint32(DT_NAVMESH_VERSION) || fileHeader.mmapVersion != MMAP_VERSION || !fileHeader.size)
        {
            LOG_ERROR("maps", "MMAP:loadVessel: '{}' is refused: bad header, or built for Detour v{} by generator v{} (expected v{} and v{}). Transports of model {} path without a deck mesh",
                fileName, fileHeader.dtVersion, fileHeader.mmapVersion, uint32(DT_NAVMESH_VERSION), MMAP_VERSION, displayId);
            fclose(file);
            return nullptr;
        }

        unsigned char* data = (unsigned char*)dtAlloc(fileHeader.size, DT_ALLOC_PERM);
        ASSERT(data);

        std::size_t const result = fread(data, fileHeader.size, 1, file);
        fclose(file);
        if (!result)
        {
            LOG_ERROR("maps", "MMAP:loadVessel: '{}' is refused: its data is shorter than its header says. Transports of model {} path without a deck mesh", fileName, displayId);
            dtFree(data);
            return nullptr;
        }

        dtNavMesh* mesh = dtAllocNavMesh();
        ASSERT(mesh);

        // on success the mesh owns the data and frees it with itself; a refused tile was never taken over
        if (dtStatusFailed(mesh->init(data, int(fileHeader.size), DT_TILE_FREE_DATA)))
        {
            LOG_ERROR("maps", "MMAP:loadVessel: '{}' is refused: it does not load as a single-tile navmesh. Transports of model {} path without a deck mesh", fileName, displayId);
            dtFreeNavMesh(mesh);
            dtFree(data);
            return nullptr;
        }

        // Detour clips every polygon query to the tile's bounds. The generator writes the model's height range there,
        // and Recast rounds a walkable span up by up to a cell, so a deck polygon can lie above the tile's top (2454,
        // 7452, 7648) and a query for a point on it misses it. The top is raised to the tile's highest vertex. Only
        // the top: the bounding volume tree is quantised from the bottom, which stays.
        if (dtMeshTile const* tile = static_cast<dtNavMesh const*>(mesh)->getTile(0); tile && tile->header)
        {
            dtMeshHeader* header = tile->header;
            for (int i = 0; i < header->vertCount; ++i)
                header->bmax[1] = std::max(header->bmax[1], tile->verts[i * 3 + 1]);

            for (int i = 0; i < header->detailVertCount; ++i)
                header->bmax[1] = std::max(header->bmax[1], tile->detailVerts[i * 3 + 1]);
        }

        LOG_DEBUG("maps", "MMAP:loadVessel: Loaded vessel mesh {:05}.mmtile", displayId);
        return std::shared_ptr<dtNavMesh const>(mesh, ConstNavMeshDeleter());
    }
}

namespace MMAP
{
    // ######################## MMapMgr ########################
    std::shared_ptr<dtNavMesh> MMapMgr::LoadNavMesh(uint32 mapId)
    {
        // load and init dtNavMesh - read parameters from file
        std::string fileName = Acore::StringFormat(MAP_FILE_NAME_FORMAT, sConfigMgr->GetOption<std::string>("DataDir", "."), mapId);

        FILE* file = fopen(fileName.c_str(), "rb");
        if (!file)
        {
            LOG_DEBUG("maps", "MMAP:loadMapData: Error: Could not open mmap file '{}'", fileName);
            return nullptr;
        }

        dtNavMeshParams params;
        uint32 count = uint32(fread(&params, sizeof(dtNavMeshParams), 1, file));
        fclose(file);
        if (count != 1)
        {
            LOG_DEBUG("maps", "MMAP:loadMapData: Error: Could not read params from file '{}'", fileName);
            return nullptr;
        }

        dtNavMesh* mesh = dtAllocNavMesh();
        ASSERT(mesh);
        if (DT_SUCCESS != mesh->init(&params))
        {
            dtFreeNavMesh(mesh);
            LOG_ERROR("maps", "MMAP:loadMapData: Failed to initialize dtNavMesh for mmap {:03} from file {}", mapId, fileName);
            return nullptr;
        }

        LOG_DEBUG("maps", "MMAP:loadMapData: Loaded {:03}.mmap", mapId);

        std::shared_ptr<dtNavMesh> navMesh = std::shared_ptr<dtNavMesh>(mesh, NavMeshDeleter());
        return navMesh;
    }

    uint32 MMapMgr::packTileID(int32 x, int32 y)
    {
        return uint32(x << 16 | y);
    }

    bool MMapMgr::LoadTile(dtNavMesh* navMesh, uint32 mapId, int32 x, int32 y)
    {
        // load this tile :: mmaps/MMMXXYY.mmtile
        std::string fileName = Acore::StringFormat(TILE_FILE_NAME_FORMAT, sConfigMgr->GetOption<std::string>("DataDir", "."), mapId, x, y);
        FILE* file = fopen(fileName.c_str(), "rb");
        if (!file)
        {
            LOG_DEBUG("maps", "MMAP:loadMap: Could not open mmtile file '{}'", fileName);
            return false;
        }

        // read header
        MmapTileHeader fileHeader;
        if (fread(&fileHeader, sizeof(MmapTileHeader), 1, file) != 1 || fileHeader.mmapMagic != MMAP_MAGIC)
        {
            LOG_ERROR("maps", "MMAP:loadMap: Bad header in mmap {:03}{:02}{:02}.mmtile", mapId, x, y);
            fclose(file);
            return false;
        }

        if (fileHeader.mmapVersion != MMAP_VERSION)
        {
            LOG_ERROR("maps", "MMAP:loadMap: {:03}{:02}{:02}.mmtile was built with generator v{}, expected v{}",
                           mapId, x, y, fileHeader.mmapVersion, MMAP_VERSION);
            fclose(file);
            return false;
        }

        unsigned char* data = (unsigned char*)dtAlloc(fileHeader.size, DT_ALLOC_PERM);
        ASSERT(data);

        std::size_t result = fread(data, fileHeader.size, 1, file);
        if (!result)
        {
            LOG_ERROR("maps", "MMAP:loadMap: Bad header or data in mmap {:03}{:02}{:02}.mmtile", mapId, x, y);
            fclose(file);
            return false;
        }

        fclose(file);

        dtTileRef tileRef = 0;

        // memory allocated for data is now managed by detour, and will be deallocated when the tile is removed
        if (dtStatusSucceed(navMesh->addTile(data, fileHeader.size, DT_TILE_FREE_DATA, 0, &tileRef)))
        {
            dtMeshHeader* header = (dtMeshHeader*)data;
            LOG_DEBUG("maps", "MMAP:loadMap: Loaded mmtile {:03}[{:02},{:02}] into {:03}[{:02},{:02}]", mapId, x, y, mapId, header->x, header->y);
            return true;
        }

        LOG_ERROR("maps", "MMAP:loadMap: Could not load {:03}{:02}{:02}.mmtile into navmesh", mapId, x, y);
        dtFree(data);
        return false;
    }

    ManagedNavMeshQuery MMapMgr::CreateNavMeshQuery(dtNavMesh const* navMesh, uint16 maxNodes)
    {
        // allocate mesh query
        dtNavMeshQuery* query = dtAllocNavMeshQuery();
        ASSERT(query);

        if (dtStatusFailed(query->init(navMesh, maxNodes)))
        {
            dtFreeNavMeshQuery(query);
            return nullptr;
        }

        ManagedNavMeshQuery navMeshQuery = ManagedNavMeshQuery(query);
        return navMeshQuery;
    }

    std::shared_ptr<dtNavMesh const> MMapMgr::GetVesselNavMesh(uint32 displayId)
    {
        //! Critical section, thread safe access: transports are created at startup and on any map's grid load
        std::lock_guard<std::mutex> lock(vesselNavMeshLock);

        auto itr = vesselNavMeshes.find(displayId);
        if (itr == vesselNavMeshes.end())
            itr = vesselNavMeshes.emplace(displayId, LoadVesselNavMesh(displayId)).first;

        return itr->second;
    }
}
