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

#ifndef _MMAP_MANAGER_H
#define _MMAP_MANAGER_H

#include "Common.h"
#include "DetourAlloc.h"
#include "DetourExtended.h"
#include "DetourNavMesh.h"
#include <memory>

//  memory management
inline void* dtCustomAlloc(std::size_t size, dtAllocHint /*hint*/)
{
    return (void*)new unsigned char[size];
}

inline void dtCustomFree(void* ptr)
{
    delete [] (unsigned char*)ptr;
}

//  move map related classes
namespace MMAP
{
    enum MMAP_LOAD_RESULT
    {
        MMAP_LOAD_RESULT_ERROR,
        MMAP_LOAD_RESULT_OK,
        MMAP_LOAD_RESULT_IGNORED,
    };

    static char const* const MAP_FILE_NAME_FORMAT = "{}/mmaps/{:03}.mmap";
    static char const* const TILE_FILE_NAME_FORMAT = "{}/mmaps/{:03}{:02}{:02}.mmtile";
    // a vessel's tile: one navmesh per collision model (display id), written by mmaps_generator --vessels
    static char const* const VESSEL_TILE_FILE_NAME_FORMAT = "{}/mmaps/vessels/{:05}.mmtile";

    struct NavMeshDeleter
    {
        void operator()(dtNavMesh* navMesh) noexcept { dtFreeNavMesh(navMesh); }
    };

    struct NavMeshQueryDeleter
    {
        void operator()(dtNavMeshQuery* query) noexcept { dtFreeNavMeshQuery(query); }
    };

    using ManagedNavMeshQuery = std::unique_ptr<dtNavMeshQuery, NavMeshQueryDeleter>;

    class MMapMgr
    {
    public:
        MMapMgr() = default;
        ~MMapMgr() = default;

        static std::shared_ptr<dtNavMesh> LoadNavMesh(uint32 mapId);
        static bool LoadTile(dtNavMesh* navMesh, uint32 mapId, int32 x, int32 y);
        static ManagedNavMeshQuery CreateNavMeshQuery(dtNavMesh const* navMesh);

        // The deck's navmesh of a vessel (a transport's collision model), in the model's own coordinates: for a
        // transport of size 1 a mesh point is a passenger offset. One mesh per display id for the whole server,
        // read from its file on the first request and never changed afterwards, so every map's thread may read it.
        // Empty for a model without a usable file; that answer is remembered, the disk is asked once per model.
        // Takes a lock and may read the disk: for the creation of a transport, not for a path request.
        static std::shared_ptr<dtNavMesh const> GetVesselNavMesh(uint32 displayId);

    private:
        static uint32 packTileID(int32 x, int32 y);
    };
}

#endif
