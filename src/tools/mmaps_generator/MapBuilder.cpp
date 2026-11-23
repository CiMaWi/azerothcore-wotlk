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

#include "MapBuilder.h"
#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <DetourNavMeshQuery.h>
#include <RecastAlloc.h>
#include "IntermediateValues.h"
#include "MapDefines.h"
#include "MapTree.h"
#include "MMapMgr.h"
#include "ModelInstance.h"
#include "PathCommon.h"
#include "StringFormat.h"
#include "VMapDefinitions.h"
#include "VMapMgr2.h"
#include "WorldModel.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <unordered_map>

namespace MMAP
{
    static void modAlmostUnwalkableTriangles(float const playerSlopeAngle,
                                             float const* verts, int /*nv*/,
                                             int const* tris, int nt,
                                             unsigned char* areas)
    {
        float const walkableThr = std::cos(playerSlopeAngle / 180.0f * static_cast<float>(M_PI));

        float norm[3];

        for (int i = 0; i < nt; ++i)
        {
            if (areas[i] == RC_NULL_AREA)
                continue;

            int const* tri = &tris[i * 3];

            float e0[3], e1[3];
            rcVsub(e0, &verts[tri[1] * 3], &verts[tri[0] * 3]);
            rcVsub(e1, &verts[tri[2] * 3], &verts[tri[0] * 3]);
            rcVcross(norm, e0, e1);
            rcVnormalize(norm);

            if (norm[1] <= walkableThr)
                areas[i] = NAV_GROUND_STEEP;
        }
    }

    // Recast's span merge (rcAddSpan) keeps the higher area id where two surfaces meet within the climb height, so a
    // steep face rising no more than one step would tag the plain ground it meets and a bot's search, which refuses
    // NAV_GROUND_STEEP, would refuse the step. While a tile is rasterized every id is shifted one bit up and the
    // steep tag takes id 1, below plain ground (TrinityCore's order, ddcbc01a65): where a steep face and plain ground
    // share a cell, the cell stays plain ground; a face rising above one step keeps its tag. Liquids keep their order
    // against plain ground and win against steep ground. navArea maps the ids back before they become polygon flags.
    static unsigned char rasterArea(unsigned char area)
    {
        return area == NAV_GROUND_STEEP ? 1 : static_cast<unsigned char>(area << 1);
    }

    static unsigned char navArea(unsigned char area)
    {
        return area == 1 ? static_cast<unsigned char>(NAV_GROUND_STEEP) : static_cast<unsigned char>(area >> 1);
    }

    // The marked areas of a map that meet a tile's bounds (Recast order: world y, z, x) widened by `pad`: the walkable
    // ones, or the blocked and plain ones.
    static std::vector<Config::MarkedArea const*> markedAreasIn(std::vector<Config::MarkedArea> const& areas,
                                                                 uint32 mapID, float const* bmin, float const* bmax,
                                                                 float pad, bool walkable)
    {
        std::vector<Config::MarkedArea const*> found;
        for (Config::MarkedArea const& area : areas)
        {
            if (area.mapId != mapID || (area.kind == Config::MarkedAreaKind::Walkable) != walkable)
                continue;
            if (area.maxX < bmin[2] - pad || area.minX > bmax[2] + pad || area.maxY < bmin[0] - pad ||
                area.minY > bmax[0] + pad || area.maxZ < bmin[1] || area.minZ > bmax[1])
                continue;
            found.push_back(&area);
        }
        return found;
    }

    // The source triangles whose centroid lies in a walkable area: they become ground before they are rasterized, also
    // where the slope rule dropped them or tagged them steep (Recast's marking after rasterization reaches only the
    // ground the slope rule kept).
    static std::vector<int> walkableTriangles(std::vector<Config::MarkedArea const*> const& areas, float const* verts,
                                              int const* tris, int nt)
    {
        std::vector<int> marked;
        if (areas.empty())
            return marked;

        for (int i = 0; i < nt; ++i)
        {
            float c[3] = { 0.0f, 0.0f, 0.0f };
            for (int k = 0; k < 3; ++k)
                rcVadd(c, c, &verts[tris[i * 3 + k] * 3]);
            for (Config::MarkedArea const* area : areas)
            {
                if (area->Contains(c[2] / 3.0f, c[0] / 3.0f, c[1] / 3.0f))
                {
                    marked.push_back(i);
                    break;
                }
            }
        }
        return marked;
    }

    // The area holding a heightfield cell's centre and a floor `top` cells high; the last one if several do
    static Config::MarkedArea const* cellArea(std::vector<Config::MarkedArea const*> const& areas, float const* bmin,
                                              float cs, float ch, int x, int y, int top)
    {
        float const worldX = bmin[2] + (y + 0.5f) * cs;
        float const worldY = bmin[0] + (x + 0.5f) * cs;
        float const worldZ = bmin[1] + top * ch;
        for (auto it = areas.rbegin(); it != areas.rend(); ++it)
            if ((*it)->Contains(worldX, worldY, worldZ))
                return *it;
        return nullptr;
    }

    // Recast's ledge filter drops a span whose reachable neighbours' floors spread more than walkableClimb: a ramp
    // steeper than atan(walkableClimb * ch / (2 * cs)), 63.4 degrees along a grid axis at climb 4 (70.5 diagonally).
    // Inside a walkable area the spans keep their area, so the area carries a ramp until neighbouring floors part by
    // more than walkableClimb (76 degrees along an axis), where the compact heightfield joins them no more.
    static void filterLedgeSpans(rcContext* ctx, std::vector<Config::MarkedArea const*> const& walkable,
                                 int walkableHeight, int walkableClimb, rcHeightfield& hf)
    {
        std::vector<std::pair<rcSpan*, unsigned char>> kept;
        if (!walkable.empty())
            for (int y = 0; y < hf.height; ++y)
                for (int x = 0; x < hf.width; ++x)
                    for (rcSpan* span = hf.spans[x + y * hf.width]; span; span = span->next)
                        if (span->area != RC_NULL_AREA &&
                            cellArea(walkable, hf.bmin, hf.cs, hf.ch, x, y, int(span->smax)))
                            kept.emplace_back(span, span->area);
        rcFilterLedgeSpans(ctx, walkableHeight, walkableClimb, hf);
        for (auto const& [span, area] : kept)
            span->area = area;
    }

    // Blocked and plain, cell by cell on the compact heightfield before the erosion, so they end at the area's edge and
    // the erosion keeps a bot's radius off a blocked edge: blocked takes every span (liquid too), plain turns the steep
    // tag into ground.
    static void markCells(std::vector<Config::MarkedArea const*> const& areas, rcCompactHeightfield& chf)
    {
        for (int y = 0; y < chf.height; ++y)
        {
            for (int x = 0; x < chf.width; ++x)
            {
                rcCompactCell const& cell = chf.cells[x + y * chf.width];
                for (unsigned int i = cell.index, end = cell.index + cell.count; i < end; ++i)
                {
                    if (chf.areas[i] == RC_NULL_AREA)
                        continue;
                    Config::MarkedArea const* area = cellArea(areas, chf.bmin, chf.cs, chf.ch, x, y, chf.spans[i].y);
                    if (!area)
                        continue;
                    if (area->kind == Config::MarkedAreaKind::Blocked)
                        chf.areas[i] = RC_NULL_AREA;
                    else if (area->kind == Config::MarkedAreaKind::Plain &&
                             chf.areas[i] == rasterArea(NAV_GROUND_STEEP))
                        chf.areas[i] = rasterArea(NAV_GROUND);
                }
            }
        }
    }

    // What joinSubTileSeams changed on a tile (counts of the final build pass)
    struct SeamStats
    {
        int snapped = 0;  // border vertices moved onto the other side's vertex (within one cell)
        int inserted = 0; // vertices of the other side inserted into an edge (T-junctions)
        int split = 0;    // polygons split into convex pieces because the insertions took them over nvp vertices
        int open = 0;     // overlapping edge pairs across a line still unlinked after the pass
    };

    // The floor the generator's own steps cut from a tile, in cells of its own area (the border cells excluded,
    // summed over the sub-tiles of the first build pass): written beside the tile as counts only, the tile's bytes
    // do not depend on them. Ledge, low ceiling and the low-obstacle gain count solid spans; radius (the erosion by
    // walkableRadius) and small region (regions under minRegionArea) also count liquid spans.
    struct TileCuts
    {
        long long raster = 0;      // walkable spans after rasterizing the solid triangles
        long long lowObstacle = 0; // made walkable by rcFilterLowHangingWalkableObstacles (a gain)
        long long ledge = 0;       // rcFilterLedgeSpans
        long long lowCeiling = 0;  // rcFilterWalkableLowHeightSpans
        long long compact = 0;     // walkable spans of the compact heightfield (solid and liquid)
        long long radius = 0;      // rcErodeWalkableArea
        long long smallRegion = 0; // rcBuildRegions: walkable spans left without a region
        SeamStats seams;
    };

    // <tile file>.cuts.json beside the tile; a tile without the file was built before the counts existed
    static void writeTileCuts(std::string const& tileFileName, uint32 mapID, uint32 tileX, uint32 tileY, float cellSize,
                              TileCuts const& cuts)
    {
        std::string const cutsFileName = tileFileName.substr(0, tileFileName.rfind('.')) + ".cuts.json";
        FILE* file = fopen(cutsFileName.c_str(), "wb");
        if (!file)
        {
            printf("[Map %03u] Failed to open %s for writing!\n", mapID, cutsFileName.c_str());
            return;
        }
        fprintf(file,
                "{\"map\":%u,\"tileX\":%u,\"tileY\":%u,\"cellSize\":%.6f,\"cells\":{\"raster\":%lld,"
                "\"lowObstacle\":%lld,\"ledge\":%lld,\"lowCeiling\":%lld,\"compact\":%lld,\"radius\":%lld,"
                "\"smallRegion\":%lld},\"seams\":{\"snapped\":%d,\"inserted\":%d,\"split\":%d,\"open\":%d}}\n",
                mapID, tileX, tileY, cellSize, cuts.raster, cuts.lowObstacle, cuts.ledge, cuts.lowCeiling, cuts.compact,
                cuts.radius, cuts.smallRegion, cuts.seams.snapped, cuts.seams.inserted, cuts.seams.split,
                cuts.seams.open);
        fclose(file);
    }

    // ---- Sub-tile seams (pathing PA43) ----
    // A map tile is built in sub-tiles of tileSize cells and merged by rcMergePolyMeshes, which links two polygons
    // only where they share both vertices of an edge. Each sub-tile places its own vertices on the line it shares with
    // the next, so floor that crosses the line is often cut there: a corner one cell off the other side's, or one long
    // edge facing several shorter ones (a T-junction). joinSubTileSeams, after the merge: (1) moves a border vertex
    // onto the other side's within one cell, only where no polygon's edge on the line gets shorter (no floor lost; the
    // floor gained is a sliver of at most one cell beside the other side's floor); (2) inserts the other side's
    // vertices into the edges they lie on (no floor changes), splitting a polygon over nvp vertices into convex pieces
    // whose detail triangles are the polygon's clipped to the piece; (3) rebuilds the adjacency as rcMergePolyMeshes
    // does. A vertex on the tile's border or on two lines is never moved; a tile without an edge to join is untouched.
    struct SeamEdge
    {
        int poly;
        int edge;
        int axis; // 0 or 2: the coordinate that is constant along the line
        int line; // that coordinate, in cells of the merged mesh
        int side; // -1 or 1: the side of the line the polygon lies on
        unsigned short v0;
        unsigned short v1;
    };

    static int polyVertCount(unsigned short const* poly, int nvp)
    {
        int n = 0;
        while (n < nvp && poly[n] != RC_MESH_NULL_IDX)
            ++n;
        return n;
    }

    static bool isPortalEdge(unsigned short neighbour)
    {
        return neighbour != RC_MESH_NULL_IDX && (neighbour & 0x8000);
    }

    // rcMergePolyMeshes' buildMeshAdjacency (Eric Lengyel's edge matching), the portal flags of the tile's border kept
    static void rebuildAdjacency(rcPolyMesh& mesh)
    {
        struct Edge
        {
            unsigned short vert[2];
            int poly[2];
            int polyEdge[2];
        };
        int const nvp = mesh.nvp;
        std::vector<int> firstEdge(mesh.nverts, -1);
        std::vector<int> nextEdge;
        std::vector<Edge> edges;
        for (int i = 0; i < mesh.npolys; ++i)
        {
            unsigned short* poly = &mesh.polys[i * nvp * 2];
            int const n = polyVertCount(poly, nvp);
            for (int j = 0; j < n; ++j)
            {
                if (!isPortalEdge(poly[nvp + j]))
                    poly[nvp + j] = RC_MESH_NULL_IDX;
                unsigned short const v0 = poly[j];
                unsigned short const v1 = poly[(j + 1) % n];
                if (v0 < v1)
                {
                    edges.push_back({ { v0, v1 }, { i, i }, { j, 0 } });
                    nextEdge.push_back(firstEdge[v0]);
                    firstEdge[v0] = int(edges.size()) - 1;
                }
            }
        }
        for (int i = 0; i < mesh.npolys; ++i)
        {
            unsigned short const* poly = &mesh.polys[i * nvp * 2];
            int const n = polyVertCount(poly, nvp);
            for (int j = 0; j < n; ++j)
            {
                unsigned short const v0 = poly[j];
                unsigned short const v1 = poly[(j + 1) % n];
                if (v0 <= v1)
                    continue;
                for (int e = firstEdge[v1]; e != -1; e = nextEdge[e])
                {
                    if (edges[e].vert[1] == v0 && edges[e].poly[0] == edges[e].poly[1])
                    {
                        edges[e].poly[1] = i;
                        edges[e].polyEdge[1] = j;
                        break;
                    }
                }
            }
        }
        for (Edge const& e : edges)
        {
            if (e.poly[0] == e.poly[1])
                continue;
            mesh.polys[e.poly[0] * nvp * 2 + nvp + e.polyEdge[0]] = (unsigned short)e.poly[1];
            mesh.polys[e.poly[1] * nvp * 2 + nvp + e.polyEdge[1]] = (unsigned short)e.poly[0];
        }
    }

    // the open edges lying on a line (both vertices on it), sorted by line
    static std::vector<SeamEdge> seamEdges(rcPolyMesh const& mesh, std::vector<int> const* lines)
    {
        std::vector<SeamEdge> out;
        int const nvp = mesh.nvp;
        for (int i = 0; i < mesh.npolys; ++i)
        {
            unsigned short const* poly = &mesh.polys[i * nvp * 2];
            int const n = polyVertCount(poly, nvp);
            for (int j = 0; j < n; ++j)
            {
                if (poly[nvp + j] != RC_MESH_NULL_IDX)
                    continue;
                unsigned short const v0 = poly[j];
                unsigned short const v1 = poly[(j + 1) % n];
                for (int axis = 0; axis <= 2; axis += 2)
                {
                    int const c = mesh.verts[v0 * 3 + axis];
                    std::vector<int> const& onAxis = lines[axis / 2];
                    if (mesh.verts[v1 * 3 + axis] != c || mesh.verts[v0 * 3 + 2 - axis] == mesh.verts[v1 * 3 + 2 - axis]
                        || !std::binary_search(onAxis.begin(), onAxis.end(), c))
                        continue;
                    long long sum = 0;
                    for (int k = 0; k < n; ++k)
                        sum += mesh.verts[poly[k] * 3 + axis];
                    long long const offset = sum - (long long)c * n;
                    if (offset)
                        out.push_back({ i, j, axis, c, offset < 0 ? -1 : 1, v0, v1 });
                }
            }
        }
        std::stable_sort(out.begin(), out.end(), [](SeamEdge const& a, SeamEdge const& b)
                         { return a.axis != b.axis ? a.axis < b.axis : a.line < b.line; });
        return out;
    }

    static int alongLine(rcPolyMesh const& mesh, SeamEdge const& e, unsigned short v)
    {
        return mesh.verts[v * 3 + 2 - e.axis];
    }

    // the edge's height (cells) where it passes position t along its line
    static float seamEdgeHeight(rcPolyMesh const& mesh, SeamEdge const& e, int t)
    {
        int const t0 = alongLine(mesh, e, e.v0);
        int const t1 = alongLine(mesh, e, e.v1);
        float const y0 = mesh.verts[e.v0 * 3 + 1];
        float const y1 = mesh.verts[e.v1 * 3 + 1];
        return y0 + (y1 - y0) * float(t - t0) / float(t1 - t0);
    }

    // two open edges from the two sides of one line that overlap along it, heights within the climb at both ends
    static bool seamPair(rcPolyMesh const& mesh, SeamEdge const& a, SeamEdge const& b, int climb)
    {
        if (a.axis != b.axis || a.line != b.line || a.side == b.side)
            return false;
        int const lo = std::max(std::min(alongLine(mesh, a, a.v0), alongLine(mesh, a, a.v1)),
                                std::min(alongLine(mesh, b, b.v0), alongLine(mesh, b, b.v1)));
        int const hi = std::min(std::max(alongLine(mesh, a, a.v0), alongLine(mesh, a, a.v1)),
                                std::max(alongLine(mesh, b, b.v0), alongLine(mesh, b, b.v1)));
        if (lo >= hi)
            return false;
        return std::fabs(seamEdgeHeight(mesh, a, lo) - seamEdgeHeight(mesh, b, lo)) <= climb &&
               std::fabs(seamEdgeHeight(mesh, a, hi) - seamEdgeHeight(mesh, b, hi)) <= climb;
    }

    // every pair of the edges (index pairs, the first from side -1); edges are sorted by line
    static std::vector<std::pair<int, int>> seamPairs(rcPolyMesh const& mesh, std::vector<SeamEdge> const& edges,
                                                      int climb)
    {
        std::vector<std::pair<int, int>> pairs;
        for (size_t begin = 0, end = 0; begin < edges.size(); begin = end)
        {
            while (end < edges.size() && edges[end].axis == edges[begin].axis && edges[end].line == edges[begin].line)
                ++end;
            for (size_t i = begin; i < end; ++i)
                for (size_t j = begin; j < end; ++j)
                    if (edges[i].side < 0 && seamPair(mesh, edges[i], edges[j], climb))
                        pairs.emplace_back(int(i), int(j));
        }
        return pairs;
    }

    // the detail triangles of a polygon (world positions) clipped to a convex piece of it (world x, z), as a detail
    // mesh of the piece: its hull vertices first, then the clipped vertices; every triangle uses the clipped ones
    static void clipDetailToPiece(std::vector<std::array<float, 3>> const& srcTris,
                                  std::vector<std::array<float, 3>> const& hull, std::vector<float>& verts,
                                  std::vector<unsigned char>& tris)
    {
        int const nh = int(hull.size());
        double area = 0;
        for (int i = 0; i < nh; ++i)
            area += double(hull[i][0]) * hull[(i + 1) % nh][2] - double(hull[(i + 1) % nh][0]) * hull[i][2];
        double const orient = area < 0 ? -1.0 : 1.0;
        std::map<std::array<float, 3>, int> index;
        std::vector<std::array<float, 3>> extra;
        std::vector<std::array<int, 3>> outTris;
        auto vertexOf = [&](std::array<float, 3> const& p)
        {
            auto it = index.find(p);
            if (it != index.end())
                return it->second;
            extra.push_back(p);
            return index[p] = nh + int(extra.size()) - 1;
        };
        for (size_t t = 0; t + 2 < srcTris.size(); t += 3)
        {
            std::vector<std::array<float, 3>> poly(srcTris.begin() + t, srcTris.begin() + t + 3);
            for (int i = 0; i < nh && poly.size() >= 3; ++i)
            {
                std::array<float, 3> const& a = hull[i];
                std::array<float, 3> const& b = hull[(i + 1) % nh];
                double const len = std::hypot(double(b[0]) - a[0], double(b[2]) - a[2]);
                if (len <= 0)
                    continue;
                auto dist = [&](std::array<float, 3> const& q)
                {
                    return orient * ((double(b[0]) - a[0]) * (double(q[2]) - a[2]) -
                                     (double(b[2]) - a[2]) * (double(q[0]) - a[0])) / len;
                };
                std::vector<std::array<float, 3>> out;
                for (size_t k = 0; k < poly.size(); ++k)
                {
                    std::array<float, 3> const& s = poly[k];
                    std::array<float, 3> const& e = poly[(k + 1) % poly.size()];
                    double const ds = dist(s);
                    double const de = dist(e);
                    if ((ds >= -1e-6) != (de >= -1e-6))
                    {
                        // the crossing from the lower endpoint, so both triangles of an edge get the same vertex
                        bool const swap = e < s;
                        std::array<float, 3> const& p = swap ? e : s;
                        std::array<float, 3> const& q = swap ? s : e;
                        double const dp = swap ? de : ds;
                        double const dq = swap ? ds : de;
                        double const f = dp / (dp - dq);
                        out.push_back({ float(p[0] + (double(q[0]) - p[0]) * f),
                                        float(p[1] + (double(q[1]) - p[1]) * f),
                                        float(p[2] + (double(q[2]) - p[2]) * f) });
                    }
                    if (de >= -1e-6)
                        out.push_back(e);
                }
                poly.swap(out);
            }
            for (size_t k = 1; k + 1 < poly.size(); ++k)
            {
                double const a2 = (double(poly[k][0]) - poly[0][0]) * (double(poly[k + 1][2]) - poly[0][2]) -
                                  (double(poly[k + 1][0]) - poly[0][0]) * (double(poly[k][2]) - poly[0][2]);
                if (std::fabs(a2) > 1e-6)
                    outTris.push_back({ vertexOf(poly[0]), vertexOf(poly[k]), vertexOf(poly[k + 1]) });
            }
        }
        // Detour stores a detail mesh's own vertex and triangle counts in a byte: a piece over that keeps a fan of
        // its hull (heights of the polygon's vertices)
        if (outTris.empty() || extra.size() + nh > 255 || outTris.size() > 255)
        {
            extra.clear();
            outTris.clear();
            for (int k = 1; k + 1 < nh; ++k)
                outTris.push_back({ 0, k, k + 1 });
        }
        for (std::array<float, 3> const& p : hull)
            verts.insert(verts.end(), p.begin(), p.end());
        for (std::array<float, 3> const& p : extra)
            verts.insert(verts.end(), p.begin(), p.end());
        auto at = [&](int i) -> std::array<float, 3> const& { return i < nh ? hull[i] : extra[i - nh]; };
        // Recast's getEdgeFlags: an edge on the piece's hull (both ends within 1 cm of one hull edge) is a boundary
        auto onHull = [&](std::array<float, 3> const& p, std::array<float, 3> const& q)
        {
            using Point = std::array<float, 3>;
            auto nearSegment = [](Point const& v, Point const& a, Point const& b)
            {
                double const dx = double(b[0]) - a[0];
                double const dz = double(b[2]) - a[2];
                double const l2 = dx * dx + dz * dz;
                double t = l2 > 0 ? ((double(v[0]) - a[0]) * dx + (double(v[2]) - a[2]) * dz) / l2 : 0;
                t = std::clamp(t, 0.0, 1.0);
                double const ex = a[0] + t * dx - v[0];
                double const ez = a[2] + t * dz - v[2];
                return ex * ex + ez * ez < 1e-4;
            };
            for (int i = 0; i < nh; ++i)
                if (nearSegment(p, hull[i], hull[(i + 1) % nh]) && nearSegment(q, hull[i], hull[(i + 1) % nh]))
                    return 1;
            return 0;
        };
        for (std::array<int, 3> const& t : outTris)
        {
            unsigned char const flags = (unsigned char)(onHull(at(t[0]), at(t[1])) | onHull(at(t[1]), at(t[2])) << 2 |
                                                        onHull(at(t[2]), at(t[0])) << 4);
            tris.insert(tris.end(), { (unsigned char)t[0], (unsigned char)t[1], (unsigned char)t[2], flags });
        }
    }

    static SeamStats joinSubTileSeams(rcPolyMesh& mesh, rcPolyMeshDetail& dmesh, std::vector<int> const* lines,
                                      int const (*border)[2], int climb)
    {
        SeamStats stats;
        int const nvp = mesh.nvp;
        std::vector<SeamEdge> edges = seamEdges(mesh, lines);
        if (edges.empty())
            return stats;
        std::vector<std::pair<int, int>> pairs = seamPairs(mesh, edges, climb);
        if (pairs.empty())
            return stats;
        auto vy = [&](unsigned short v) { return int(mesh.verts[v * 3 + 1]); };
        auto polyHas = [&](int p, unsigned short v)
        {
            unsigned short const* poly = &mesh.polys[p * nvp * 2];
            return std::find(poly, poly + polyVertCount(poly, nvp), v) != poly + polyVertCount(poly, nvp);
        };

        // (1) snap: a vertex on the border or on lines of both axes stays
        std::vector<char> fixed(mesh.nverts, 0);
        for (int v = 0; v < mesh.nverts; ++v)
        {
            int onLines = 0;
            for (int a = 0; a < 2; ++a)
            {
                int const c = mesh.verts[v * 3 + a * 2];
                onLines += std::binary_search(lines[a].begin(), lines[a].end(), c) ? 1 : 0;
                if (c <= border[a][0] || c >= border[a][1])
                    fixed[v] = 1;
            }
            if (onLines > 1)
                fixed[v] = 1;
        }
        std::vector<std::vector<int>> polysOf(mesh.nverts);
        for (int i = 0; i < mesh.npolys; ++i)
        {
            unsigned short const* poly = &mesh.polys[i * nvp * 2];
            for (int k = 0, n = polyVertCount(poly, nvp); k < n; ++k)
                polysOf[poly[k]].push_back(i);
        }
        struct Snap
        {
            int dist;
            int e; // the side -1 edge
            unsigned short a;
            unsigned short b;
            bool operator<(Snap const& o) const
            {
                return dist != o.dist ? dist < o.dist : (a != o.a ? a < o.a : b < o.b);
            }
        };
        std::vector<Snap> snaps;
        for (auto const& [i, j] : pairs)
            for (unsigned short a : { edges[i].v0, edges[i].v1 })
                for (unsigned short b : { edges[j].v0, edges[j].v1 })
                {
                    int const d = std::abs(alongLine(mesh, edges[i], a) - alongLine(mesh, edges[i], b));
                    if (a != b && d <= 1 && std::abs(vy(a) - vy(b)) <= climb)
                        snaps.push_back({ d, i, a, b });
                }
        std::sort(snaps.begin(), snaps.end());
        std::vector<char> touched(mesh.nverts, 0);
        // moving m (on side `side`) onto k: -1 not allowed (floor lost, a pinch or a fixed vertex), 0 floor kept,
        // 1 a sliver gained
        auto moveCost = [&](SeamEdge const& line, unsigned short m, unsigned short k, int side)
        {
            if (fixed[m])
                return -1;
            for (int p : polysOf[m])
            {
                if (polyHas(p, k))
                    return -1;
                unsigned short const* poly = &mesh.polys[p * nvp * 2];
                int const n = polyVertCount(poly, nvp);
                long long sum = 0;
                for (int q = 0; q < n; ++q)
                    sum += mesh.verts[poly[q] * 3 + line.axis];
                if ((sum - (long long)line.line * n) * side < 0)
                    return -1; // m is shared with the other side
            }
            int const dir = alongLine(mesh, line, k) - alongLine(mesh, line, m);
            if (!dir)
                return 0;
            // the move turns the edges from m into its polygons: floor passes between them, so they must be one kind
            for (int p : polysOf[m])
                if (mesh.areas[p] != mesh.areas[polysOf[m][0]])
                    return -1;
            // every polygon at m keeps its winding and gets no dent with m on k's place: a thin one would turn over
            // or lose its area, a wide one get a corner that points inward
            for (int p : polysOf[m])
            {
                unsigned short const* poly = &mesh.polys[p * nvp * 2];
                int const n = polyVertCount(poly, nvp);
                auto at = [&](int q, bool moved, int axis)
                {
                    unsigned short const v = poly[q % n];
                    return (long long)mesh.verts[(moved && v == m ? k : v) * 3 + axis];
                };
                // twice the corner's turn at vertex q + 1, and their sum over the polygon's corners: twice its area
                auto turn = [&](int q, bool moved)
                {
                    return (at(q + 1, moved, 0) - at(q, moved, 0)) * (at(q + 2, moved, 2) - at(q + 1, moved, 2)) -
                           (at(q + 1, moved, 2) - at(q, moved, 2)) * (at(q + 2, moved, 0) - at(q + 1, moved, 0));
                };
                auto area2 = [&](bool moved)
                {
                    long long sum = 0;
                    for (int q = 0; q < n; ++q)
                        sum += at(q, moved, 0) * at(q + 1, moved, 2) - at(q + 1, moved, 0) * at(q, moved, 2);
                    return sum;
                };
                long long const before = area2(false);
                long long const after = area2(true);
                if (!before || !after || (before > 0) != (after > 0))
                    return -1;
                for (int q = 0; q < n; ++q)
                {
                    long long const was = turn(q, false);
                    long long const now = turn(q, true);
                    if (now && (now > 0) != (before > 0) && (!was || (was > 0) == (before > 0)))
                        return -1;
                }
            }
            bool toward = false;
            bool away = false;
            for (SeamEdge const& e : edges)
            {
                if (e.axis != line.axis || e.line != line.line || e.side != side || (e.v0 != m && e.v1 != m))
                    continue;
                int const other = alongLine(mesh, e, e.v0 == m ? e.v1 : e.v0) - alongLine(mesh, e, m);
                (other * dir > 0 ? toward : away) = true;
            }
            if (toward && !away)
                return -1;
            return away && !toward ? 1 : 0;
        };
        for (Snap const& s : snaps)
        {
            if (touched[s.a] || touched[s.b])
                continue;
            SeamEdge const& line = edges[s.e];
            int const costA = moveCost(line, s.a, s.b, -1);
            int const costB = moveCost(line, s.b, s.a, 1);
            if (costA < 0 && costB < 0)
                continue;
            bool const moveA = costA >= 0 && (costB < 0 || costA <= costB);
            unsigned short const m = moveA ? s.a : s.b;
            unsigned short const k = moveA ? s.b : s.a;
            for (int p : polysOf[m])
            {
                unsigned short* poly = &mesh.polys[p * nvp * 2];
                std::replace(poly, poly + polyVertCount(poly, nvp), m, k);
                polysOf[k].push_back(p);
            }
            polysOf[m].clear();
            touched[s.a] = touched[s.b] = 1;
            ++stats.snapped;
        }
        if (stats.snapped)
        {
            rebuildAdjacency(mesh);
            edges = seamEdges(mesh, lines);
            pairs = seamPairs(mesh, edges, climb);
        }

        // (2) T-junctions: the other side's vertices strictly inside an edge, at its height within the climb
        std::map<std::pair<int, int>, std::vector<unsigned short>> inserts;
        for (auto const& [i, j] : pairs)
        {
            for (int pass = 0; pass < 2; ++pass)
            {
                SeamEdge const& e = edges[pass ? j : i];
                SeamEdge const& f = edges[pass ? i : j];
                int const lo = std::min(alongLine(mesh, e, e.v0), alongLine(mesh, e, e.v1));
                int const hi = std::max(alongLine(mesh, e, e.v0), alongLine(mesh, e, e.v1));
                for (unsigned short w : { f.v0, f.v1 })
                {
                    int const t = alongLine(mesh, e, w);
                    if (t > lo && t < hi && std::fabs(vy(w) - seamEdgeHeight(mesh, e, t)) <= climb &&
                        !polyHas(e.poly, w))
                        inserts[{ e.poly, e.edge }].push_back(w);
                }
            }
        }

        // the new vertex lists; a polygon over nvp vertices becomes a fan of convex pieces from one of its vertices
        struct Origin
        {
            int src;
            int kind; // 0 unchanged, 1 vertices inserted (hullMap: old vertex -> new), 2 a piece (detail clipped)
            std::vector<int> hullMap;
        };
        std::vector<Origin> origin(mesh.npolys);
        std::vector<std::vector<unsigned short>> newPolys(mesh.npolys);
        std::vector<int> changed;
        for (int i = 0; i < mesh.npolys; ++i)
            origin[i] = { i, 0, {} };
        std::vector<char> done(mesh.npolys, 0);
        for (auto const& [key, list] : inserts)
        {
            if (done[key.first])
                continue;
            int const p = key.first;
            done[p] = 1;
            unsigned short const* poly = &mesh.polys[p * nvp * 2];
            int const n = polyVertCount(poly, nvp);
            std::vector<unsigned short> verts;
            std::vector<int> hullMap;
            std::vector<char> original;
            for (int k = 0; k < n; ++k)
            {
                hullMap.push_back(int(verts.size()));
                verts.push_back(poly[k]);
                original.push_back(1);
                auto it = inserts.find({ p, k });
                if (it == inserts.end())
                    continue;
                std::vector<unsigned short> add = it->second;
                std::sort(add.begin(), add.end());
                add.erase(std::unique(add.begin(), add.end()), add.end());
                int const axis = 2 - (mesh.verts[poly[k] * 3] == mesh.verts[poly[(k + 1) % n] * 3] ? 0 : 2);
                std::sort(add.begin(), add.end(), [&](unsigned short a, unsigned short b)
                          { return std::abs(mesh.verts[a * 3 + axis] - mesh.verts[poly[k] * 3 + axis]) <
                                   std::abs(mesh.verts[b * 3 + axis] - mesh.verts[poly[k] * 3 + axis]); });
                for (unsigned short w : add)
                {
                    verts.push_back(w);
                    original.push_back(0);
                    ++stats.inserted;
                }
            }
            changed.push_back(p);
            int const total = int(verts.size());
            if (total <= nvp)
            {
                // detail triangles index their vertices in a byte: where the shifted indices would not fit, clip
                bool const fits = int(dmesh.meshes[p * 4 + 1]) - n + total <= 256;
                origin[p] = { p, fits ? 1 : 2, fits ? hullMap : std::vector<int>() };
                newPolys[p] = verts;
                continue;
            }
            // the fan from the first original vertex whose pieces all have area (cells^2)
            for (int anchor = 0; anchor < total; ++anchor)
            {
                if (!original[anchor])
                    continue;
                std::vector<std::vector<unsigned short>> pieces;
                bool ok = true;
                for (int start = 1; start < total - 1 && ok;)
                {
                    int const end = std::min(start + nvp - 2, total - 1);
                    std::vector<unsigned short> piece{ verts[anchor] };
                    for (int k = start; k <= end; ++k)
                        piece.push_back(verts[(anchor + k) % total]);
                    long long area2 = 0;
                    for (size_t k = 0; k < piece.size(); ++k)
                    {
                        unsigned short const* a = &mesh.verts[piece[k] * 3];
                        unsigned short const* b = &mesh.verts[piece[(k + 1) % piece.size()] * 3];
                        area2 += (long long)a[0] * b[2] - (long long)b[0] * a[2];
                    }
                    ok = area2 != 0;
                    pieces.push_back(piece);
                    start = end;
                }
                if (!ok)
                    continue;
                newPolys[p] = pieces[0];
                origin[p] = { p, 2, {} };
                for (size_t k = 1; k < pieces.size(); ++k)
                {
                    newPolys.push_back(pieces[k]);
                    origin.push_back({ p, 2, {} });
                }
                ++stats.split;
                break;
            }
            if (origin[p].kind == 0)
            {
                stats.inserted -= total - n; // no fan without a degenerate piece: the polygon stays as it was
                changed.pop_back();
            }
        }
        if (changed.empty())
        {
            stats.open = int(pairs.size());
            return stats;
        }

        // the polygon arrays, the changed polygons' border portals kept by their vertex pair
        std::map<std::pair<unsigned short, unsigned short>, unsigned short> portals;
        for (int p : changed)
        {
            unsigned short const* poly = &mesh.polys[p * nvp * 2];
            for (int k = 0, n = polyVertCount(poly, nvp); k < n; ++k)
                if (isPortalEdge(poly[nvp + k]))
                    portals[{ poly[k], poly[(k + 1) % n] }] = poly[nvp + k];
        }
        int const npolys = int(origin.size());
        unsigned short* polys = (unsigned short*)rcAlloc(sizeof(unsigned short) * npolys * 2 * nvp, RC_ALLOC_PERM);
        unsigned short* regs = (unsigned short*)rcAlloc(sizeof(unsigned short) * npolys, RC_ALLOC_PERM);
        unsigned char* areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * npolys, RC_ALLOC_PERM);
        unsigned short* flags = (unsigned short*)rcAlloc(sizeof(unsigned short) * npolys, RC_ALLOC_PERM);
        memset(polys, 0xff, sizeof(unsigned short) * npolys * 2 * nvp);
        for (int i = 0; i < npolys; ++i)
        {
            int const src = origin[i].src;
            regs[i] = mesh.regs[src];
            areas[i] = mesh.areas[src];
            flags[i] = mesh.flags[src];
            unsigned short* poly = &polys[i * nvp * 2];
            if (!origin[i].kind)
            {
                memcpy(poly, &mesh.polys[src * nvp * 2], sizeof(unsigned short) * nvp * 2);
                continue;
            }
            std::vector<unsigned short> const& verts = newPolys[i];
            for (size_t k = 0; k < verts.size(); ++k)
            {
                poly[k] = verts[k];
                auto it = portals.find({ verts[k], verts[(k + 1) % verts.size()] });
                if (it != portals.end())
                    poly[nvp + k] = it->second;
            }
        }

        // the detail meshes: unchanged polygons copied, inserted vertices added to the hull, pieces clipped
        float const* orig = mesh.bmin;
        auto world = [&](unsigned short v) -> std::array<float, 3>
        {
            return { orig[0] + mesh.verts[v * 3] * mesh.cs, orig[1] + mesh.verts[v * 3 + 1] * mesh.ch,
                     orig[2] + mesh.verts[v * 3 + 2] * mesh.cs };
        };
        std::vector<unsigned int> meshes(npolys * 4);
        std::vector<float> dverts;
        std::vector<unsigned char> dtris;
        for (int i = 0; i < npolys; ++i)
        {
            int const src = origin[i].src;
            unsigned int const vb = dmesh.meshes[src * 4];
            unsigned int const nd = dmesh.meshes[src * 4 + 1];
            unsigned int const tb = dmesh.meshes[src * 4 + 2];
            unsigned int const nt = dmesh.meshes[src * 4 + 3];
            int const nvOld = polyVertCount(&mesh.polys[src * nvp * 2], nvp);
            unsigned int const vbase = (unsigned int)(dverts.size() / 3);
            unsigned int const tbase = (unsigned int)(dtris.size() / 4);
            if (origin[i].kind == 0)
            {
                dverts.insert(dverts.end(), dmesh.verts + vb * 3, dmesh.verts + (vb + nd) * 3);
                dtris.insert(dtris.end(), dmesh.tris + tb * 4, dmesh.tris + (tb + nt) * 4);
            }
            else if (origin[i].kind == 1)
            {
                std::vector<unsigned short> const& verts = newPolys[i];
                int const nvNew = int(verts.size());
                for (unsigned short v : verts)
                {
                    std::array<float, 3> const p = world(v);
                    dverts.insert(dverts.end(), p.begin(), p.end());
                }
                dverts.insert(dverts.end(), dmesh.verts + (vb + nvOld) * 3, dmesh.verts + (vb + nd) * 3);
                for (unsigned int t = tb; t < tb + nt; ++t)
                {
                    for (int k = 0; k < 3; ++k)
                    {
                        int const v = dmesh.tris[t * 4 + k];
                        dtris.push_back((unsigned char)(v < nvOld ? origin[i].hullMap[v] : v - nvOld + nvNew));
                    }
                    dtris.push_back(dmesh.tris[t * 4 + 3]);
                }
            }
            else
            {
                std::vector<std::array<float, 3>> srcTris;
                unsigned short const* srcPoly = &mesh.polys[src * nvp * 2];
                for (unsigned int t = tb; t < tb + nt; ++t)
                {
                    for (int k = 0; k < 3; ++k)
                    {
                        int const v = dmesh.tris[t * 4 + k];
                        srcTris.push_back(v < nvOld ? world(srcPoly[v])
                                                    : std::array<float, 3>{ dmesh.verts[(vb + v) * 3],
                                                                            dmesh.verts[(vb + v) * 3 + 1],
                                                                            dmesh.verts[(vb + v) * 3 + 2] });
                    }
                }
                std::vector<std::array<float, 3>> hull;
                for (unsigned short v : newPolys[i])
                    hull.push_back(world(v));
                clipDetailToPiece(srcTris, hull, dverts, dtris);
            }
            meshes[i * 4] = vbase;
            meshes[i * 4 + 1] = (unsigned int)(dverts.size() / 3) - vbase;
            meshes[i * 4 + 2] = tbase;
            meshes[i * 4 + 3] = (unsigned int)(dtris.size() / 4) - tbase;
        }

        rcFree(mesh.polys);
        rcFree(mesh.regs);
        rcFree(mesh.areas);
        rcFree(mesh.flags);
        mesh.polys = polys;
        mesh.regs = regs;
        mesh.areas = areas;
        mesh.flags = flags;
        mesh.npolys = npolys;
        mesh.maxpolys = npolys;
        rcFree(dmesh.meshes);
        rcFree(dmesh.verts);
        rcFree(dmesh.tris);
        dmesh.meshes = (unsigned int*)rcAlloc(sizeof(unsigned int) * meshes.size(), RC_ALLOC_PERM);
        dmesh.verts = (float*)rcAlloc(sizeof(float) * dverts.size(), RC_ALLOC_PERM);
        dmesh.tris = (unsigned char*)rcAlloc(sizeof(unsigned char) * dtris.size(), RC_ALLOC_PERM);
        memcpy(dmesh.meshes, meshes.data(), sizeof(unsigned int) * meshes.size());
        memcpy(dmesh.verts, dverts.data(), sizeof(float) * dverts.size());
        memcpy(dmesh.tris, dtris.data(), sizeof(unsigned char) * dtris.size());
        dmesh.nmeshes = npolys;
        dmesh.nverts = int(dverts.size() / 3);
        dmesh.ntris = int(dtris.size() / 4);

        rebuildAdjacency(mesh);
        stats.open = int(seamPairs(mesh, seamEdges(mesh, lines), climb).size());
        return stats;
    }

    static long long walkableSpans(rcHeightfield const& hf, int border)
    {
        long long n = 0;
        for (int y = border; y < hf.height - border; ++y)
            for (int x = border; x < hf.width - border; ++x)
                for (rcSpan const* span = hf.spans[x + y * hf.width]; span; span = span->next)
                    if (span->area != RC_NULL_AREA)
                        ++n;
        return n;
    }

    static long long walkableSpans(rcCompactHeightfield const& chf, int border, bool withoutRegion)
    {
        long long n = 0;
        for (int y = border; y < chf.height - border; ++y)
        {
            for (int x = border; x < chf.width - border; ++x)
            {
                rcCompactCell const& cell = chf.cells[x + y * chf.width];
                for (unsigned int i = cell.index, end = cell.index + cell.count; i < end; ++i)
                    if (chf.areas[i] != RC_NULL_AREA && (!withoutRegion || chf.spans[i].reg == 0))
                        ++n;
            }
        }
        return n;
    }

    TileBuilder::TileBuilder(MapBuilder* mapBuilder, bool skipLiquid, bool debugOutput) :
            m_debugOutput(debugOutput),
            m_mapBuilder(mapBuilder),
            m_terrainBuilder(nullptr),
            m_workerThread(&TileBuilder::WorkerThread, this),
            m_rcContext(nullptr)
    {
        m_terrainBuilder = new TerrainBuilder(m_mapBuilder->getConfig().DataDirPath(), skipLiquid);
        m_rcContext = new rcContext(false);
    }

    TileBuilder::~TileBuilder()
    {
        WaitCompletion();

        delete m_terrainBuilder;
        delete m_rcContext;
    }

    void TileBuilder::WaitCompletion()
    {
        if (m_workerThread.joinable())
            m_workerThread.join();
    }

    MapBuilder::MapBuilder(Config* config, int mapid, unsigned int threads) :
        m_config             (config),
        m_debugOutput        (config->IsDebugOutputEnabled()),
        m_threads            (threads),
        m_skipContinents     (config->ShouldSkipContinents()),
        m_skipJunkMaps       (config->ShouldSkipJunkMaps()),
        m_skipBattlegrounds  (config->ShouldSkipBattlegrounds()),
        m_skipLiquid         (config->ShouldSkipLiquid()),
        m_mapid              (mapid),
        m_totalTiles         (0u),
        m_totalTilesProcessed(0u),

        _cancelationToken    (false)
    {
        m_terrainBuilder = new TerrainBuilder(config->DataDirPath(), config->ShouldSkipLiquid());

        m_rcContext = new rcContext(false);

        // At least 1 thread is needed
        m_threads = std::max(1u, m_threads);

        discoverTiles();
    }

    /**************************************************************************/
    MapBuilder::~MapBuilder()
    {
        for (auto & m_tile : m_tiles)
        {
            m_tile.m_tiles->clear();
            delete m_tile.m_tiles;
        }

        delete m_terrainBuilder;
        delete m_rcContext;
    }

    /**************************************************************************/
    void MapBuilder::discoverTiles()
    {
        std::vector<std::string> files;
        uint32 mapID, tileX, tileY, tileID, count = 0, fsize = 0;
        char filter[12];

        printf("Discovering maps... ");
        getDirContents(files, m_config->MapsPath());
        for (auto & file : files)
        {
            mapID = uint32(atoi(file.substr(0, file.size() - 8).c_str()));
            if (std::find(m_tiles.begin(), m_tiles.end(), mapID) == m_tiles.end())
            {
                m_tiles.emplace_back(mapID, new std::set<uint32>);
                count++;
            }
        }

        files.clear();
        getDirContents(files, m_config->VMapsPath(), "*.vmtree");
        for (auto & file : files)
        {
            mapID = uint32(atoi(file.substr(0, file.size() - 7).c_str()));
            if (std::find(m_tiles.begin(), m_tiles.end(), mapID) == m_tiles.end())
            {
                m_tiles.emplace_back(mapID, new std::set<uint32>);
                count++;
            }
        }
        printf("found %u.\n", count);

        count = 0;
        printf("Discovering tiles... ");
        for (auto & m_tile : m_tiles)
        {
            std::set<uint32>* tiles = m_tile.m_tiles;
            mapID = m_tile.m_mapId;

            sprintf(filter, "%03u*.vmtile", mapID);
            files.clear();
            getDirContents(files, m_config->VMapsPath(), filter);
            for (auto & file : files)
            {
                fsize = file.size();

                tileY = uint32(atoi(file.substr(fsize - 12, 2).c_str()));
                tileX = uint32(atoi(file.substr(fsize - 9, 2).c_str()));
                tileID = StaticMapTree::packTileID(tileY, tileX);

                tiles->insert(tileID);
                count++;
            }

            sprintf(filter, "%03u*", mapID);
            files.clear();
            getDirContents(files, m_config->MapsPath(), filter);
            for (auto & file : files)
            {
                fsize = file.size();

                tileY = uint32(atoi(file.substr(fsize - 8, 2).c_str()));
                tileX = uint32(atoi(file.substr(fsize - 6, 2).c_str()));
                tileID = StaticMapTree::packTileID(tileX, tileY);

                if (tiles->insert(tileID).second)
                    count++;
            }

            // make sure we process maps which don't have tiles
            if (tiles->empty())
            {
                // convert coord bounds to grid bounds
                uint32 minX, minY, maxX, maxY;
                getGridBounds(mapID, minX, minY, maxX, maxY);

                // add all tiles within bounds to tile list.
                for (uint32 i = minX; i <= maxX; ++i)
                    for (uint32 j = minY; j <= maxY; ++j)
                        if (tiles->insert(StaticMapTree::packTileID(i, j)).second)
                            count++;
            }
        }
        printf("found %u.\n\n", count);

        // Calculate tiles to process in total
        for (auto & m_tile : m_tiles)
        {
            if (!shouldSkipMap(m_tile.m_mapId))
                m_totalTiles += m_tile.m_tiles->size();
        }
    }

    /**************************************************************************/
    std::set<uint32>* MapBuilder::getTileList(uint32 mapID)
    {
        TileList::iterator itr = std::find(m_tiles.begin(), m_tiles.end(), mapID);
        if (itr != m_tiles.end())
            return (*itr).m_tiles;

        std::set<uint32>* tiles = new std::set<uint32>();
        m_tiles.emplace_back(mapID, tiles);
        return tiles;
    }

    /**************************************************************************/
    void MapBuilder::buildMaps(Optional<uint32> mapID)
    {
        printf("Using %u threads to generate mmaps\n", m_threads);

        for (unsigned int i = 0; i < m_threads; ++i)
        {
            m_tileBuilders.push_back(new TileBuilder(this, m_skipLiquid, m_debugOutput));
        }

        if (mapID)
        {
            buildMap(*mapID);
        }
        else
        {
            // Build all maps if no map id has been specified
            for (TileList::iterator it = m_tiles.begin(); it != m_tiles.end(); ++it)
            {
                if (!shouldSkipMap(it->m_mapId))
                    buildMap(it->m_mapId);
            }
        }

        while (!_queue.Empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        _cancelationToken = true;

        _queue.Cancel();

        for (auto& builder : m_tileBuilders)
            delete builder;

        m_tileBuilders.clear();
    }

    /**************************************************************************/
    void MapBuilder::getGridBounds(uint32 mapID, uint32& minX, uint32& minY, uint32& maxX, uint32& maxY) const
    {
        // min and max are initialized to invalid values so the caller iterating the [min, max] range
        // will never enter the loop unless valid min/max values are found
        maxX = 0;
        maxY = 0;
        minX = std::numeric_limits<uint32>::max();
        minY = std::numeric_limits<uint32>::max();

        float bmin[3] = { 0, 0, 0 };
        float bmax[3] = { 0, 0, 0 };
        float lmin[3] = { 0, 0, 0 };
        float lmax[3] = { 0, 0, 0 };
        MeshData meshData;

        // make sure we process maps which don't have tiles
        // initialize the static tree, which loads WDT models
        if (!m_terrainBuilder->loadVMap(mapID, 64, 64, meshData))
            return;

        // get the coord bounds of the model data
        if (meshData.solidVerts.size() + meshData.liquidVerts.size() == 0)
            return;

        // get the coord bounds of the model data
        if (meshData.solidVerts.size() && meshData.liquidVerts.size())
        {
            rcCalcBounds(meshData.solidVerts.getCArray(), meshData.solidVerts.size() / 3, bmin, bmax);
            rcCalcBounds(meshData.liquidVerts.getCArray(), meshData.liquidVerts.size() / 3, lmin, lmax);
            rcVmin(bmin, lmin);
            rcVmax(bmax, lmax);
        }
        else if (meshData.solidVerts.size())
            rcCalcBounds(meshData.solidVerts.getCArray(), meshData.solidVerts.size() / 3, bmin, bmax);
        else
            rcCalcBounds(meshData.liquidVerts.getCArray(), meshData.liquidVerts.size() / 3, lmin, lmax);

        // convert coord bounds to grid bounds
        maxX = 32 - bmin[0] / GRID_SIZE;
        maxY = 32 - bmin[2] / GRID_SIZE;
        minX = 32 - bmax[0] / GRID_SIZE;
        minY = 32 - bmax[2] / GRID_SIZE;
    }

    void MapBuilder::buildMeshFromFile(char* name)
    {
        FILE* file = fopen(name, "rb");
        if (!file)
            return;

        printf("Building mesh from file\n");
        int tileX, tileY, mapId;
        if (fread(&mapId, sizeof(int), 1, file) != 1)
        {
            fclose(file);
            return;
        }
        if (fread(&tileX, sizeof(int), 1, file) != 1)
        {
            fclose(file);
            return;
        }
        if (fread(&tileY, sizeof(int), 1, file) != 1)
        {
            fclose(file);
            return;
        }

        dtNavMesh* navMesh = nullptr;
        buildNavMesh(mapId, navMesh);
        if (!navMesh)
        {
            printf("Failed creating navmesh!              \n");
            fclose(file);
            return;
        }

        uint32 verticesCount, indicesCount;
        if (fread(&verticesCount, sizeof(uint32), 1, file) != 1)
        {
            fclose(file);
            return;
        }

        if (fread(&indicesCount, sizeof(uint32), 1, file) != 1)
        {
            fclose(file);
            return;
        }

        float* verts = new float[verticesCount];
        int* inds = new int[indicesCount];

        if (fread(verts, sizeof(float), verticesCount, file) != verticesCount)
        {
            fclose(file);
            delete[] verts;
            delete[] inds; // cppcheck-suppress uninitdata
            return;
        }

        if (fread(inds, sizeof(int), indicesCount, file) != indicesCount)
        {
            fclose(file);
            delete[] verts;
            delete[] inds;
            return;
        }

        MeshData data;

        for (uint32 i = 0; i < verticesCount; ++i)
            data.solidVerts.append(verts[i]);
        delete[] verts;

        for (uint32 i = 0; i < indicesCount; ++i)
            data.solidTris.append(inds[i]);
        delete[] inds;

        TerrainBuilder::cleanVertices(data.solidVerts, data.solidTris);
        // get bounds of current tile
        float bmin[3], bmax[3];
        getTileBounds(tileX, tileY, data.solidVerts.getCArray(), data.solidVerts.size() / 3, bmin, bmax);

        // build navmesh tile
        TileBuilder tileBuilder = TileBuilder(this, m_skipLiquid, m_debugOutput);
        tileBuilder.buildMoveMapTile(mapId, tileX, tileY, data, bmin, bmax, navMesh);
        fclose(file);
    }

    /**************************************************************************/
    // A vessel (a transport's collision model) gets a navmesh of its own: the model's triangles, untransformed, all as
    // ground, with the global meshSettings. Recast's (rx, ry, rz) is the model's (y, z, x), so a mesh point read as
    // (rz, rx, ry) is a point in the model's frame: for a transport of size 1, a passenger offset.
    void MapBuilder::buildVessels(char const* listFile)
    {
        // the list's first column is the display id; its header and '#' lines start with no digit
        std::vector<uint32> displayIds;
        std::ifstream list(listFile);
        if (!list)
            printf("Cannot read the vessel list %s\n", listFile);

        std::string line;
        while (std::getline(list, line))
            if (!line.empty() && std::isdigit(static_cast<unsigned char>(line[0])))
                displayIds.push_back(static_cast<uint32>(std::stoul(line)));

        // the server's model for a display id (LoadGameObjectModelList): the name and the WMO flag
        struct VesselModel
        {
            std::string name;
            bool isWmo;
        };

        std::unordered_map<uint32, VesselModel> models;
        std::filesystem::path const vmapsPath(m_config->VMapsPath());
        std::string const modelListPath = (vmapsPath / VMAP::GAMEOBJECT_MODELS).string();
        if (FILE* modelList = fopen(modelListPath.c_str(), "rb"))
        {
            char magic[8];
            if (fread(magic, 1, 8, modelList) == 8 && !memcmp(magic, VMAP::VMAP_MAGIC, 8))
            {
                uint32 displayId, nameLength;
                uint8 isWmo;
                char name[500];
                G3D::Vector3 bounds[2];
                while (fread(&displayId, sizeof(uint32), 1, modelList) == 1
                       && fread(&isWmo, sizeof(uint8), 1, modelList) == 1
                       && fread(&nameLength, sizeof(uint32), 1, modelList) == 1 && nameLength < sizeof(name)
                       && fread(name, sizeof(char), nameLength, modelList) == nameLength
                       && fread(bounds, sizeof(G3D::Vector3), 2, modelList) == 2)
                    models.emplace(displayId, VesselModel{std::string(name, nameLength), isWmo != 0});
            }

            fclose(modelList);
        }

        printf("Building %u vessel meshes, %u models in %s\n", uint32(displayIds.size()), uint32(models.size()),
               modelListPath.c_str());
        std::filesystem::create_directories(std::filesystem::path(m_config->MMapsPath()) / "vessels");

        TileBuilder tileBuilder = TileBuilder(this, true, false);
        uint32 built = 0;
        for (uint32 displayId : displayIds)
        {
            auto const it = models.find(displayId);
            if (it == models.end())
            {
                printf("[Vessel %05u]: no row in %s\n", displayId, VMAP::GAMEOBJECT_MODELS);
                continue;
            }

            VesselModel const& model = it->second;
            std::string const modelPath = (vmapsPath / (model.name + ".vmo")).string();
            WorldModel worldModel;
            if (!worldModel.readFile(modelPath))
            {
                printf("[Vessel %05u]: cannot read %s\n", displayId, modelPath.c_str());
                continue;
            }

            // the server loads the model as WMO or M2 by the list's flag (GameObjectModel::initialize); M2 triangles
            // are reversed as the map build does
            bool const nameIsM2 = model.name.find(".m2") != std::string::npos
                                  || model.name.find(".M2") != std::string::npos;
            if (nameIsM2 == model.isWmo)
                printf("[Vessel %05u]: %s is flagged %s in %s\n", displayId, model.name.c_str(),
                       model.isWmo ? "WMO" : "M2", VMAP::GAMEOBJECT_MODELS);

            MeshData meshData;
            std::vector<GroupModel> groupModels;
            worldModel.GetGroupModels(groupModels);
            for (GroupModel& groupModel : groupModels)
            {
                std::vector<G3D::Vector3> vertices;
                std::vector<MeshTriangle> triangles;
                WmoLiquid* liquid = nullptr;
                groupModel.GetMeshData(vertices, triangles, liquid);

                int const offset = meshData.solidVerts.size() / 3;
                TerrainBuilder::copyVertices(vertices, meshData.solidVerts);
                TerrainBuilder::copyIndices(triangles, meshData.solidTris, offset, !model.isWmo);
            }

            if (!meshData.solidTris.size())
            {
                printf("[Vessel %05u]: %s has no triangles\n", displayId, model.name.c_str());
                continue;
            }

            TerrainBuilder::cleanVertices(meshData.solidVerts, meshData.solidTris);
            float bmin[3], bmax[3];
            rcCalcBounds(meshData.solidVerts.getCArray(), meshData.solidVerts.size() / 3, bmin, bmax);
            printf("[Vessel %05u]: %s, %s, %d triangles, bounds (%.2f %.2f %.2f) (%.2f %.2f %.2f)\n", displayId,
                   model.name.c_str(), model.isWmo ? "WMO" : "M2", int(meshData.solidTris.size() / 3), bmin[2], bmin[0],
                   bmin[1], bmax[2], bmax[0], bmax[1]);

            // the vessel's own navmesh: one tile at its model's bounds, as Detour's single-tile init makes it
            dtNavMeshParams navMeshParams;
            memset(&navMeshParams, 0, sizeof(dtNavMeshParams));
            rcVcopy(navMeshParams.orig, bmin);
            navMeshParams.tileWidth = bmax[0] - bmin[0];
            navMeshParams.tileHeight = bmax[2] - bmin[2];
            navMeshParams.maxTiles = 1;
            navMeshParams.maxPolys = 1 << DT_POLY_BITS;

            dtNavMesh* navMesh = dtAllocNavMesh();
            if (!navMesh || dtStatusFailed(navMesh->init(&navMeshParams)))
            {
                printf("[Vessel %05u]: failed creating navmesh\n", displayId);
                dtFreeNavMesh(navMesh);
                continue;
            }

            tileBuilder.buildMoveMapTile(0, 0, 0, meshData, bmin, bmax, navMesh, displayId);
            dtFreeNavMesh(navMesh);

            if (checkVesselTile(displayId, bmin, bmax))
                ++built;
        }

        printf("Built %u of %u vessel meshes\n", built, uint32(displayIds.size()));

        _cancelationToken = true;
        _queue.Cancel();
    }

    /**************************************************************************/
    // Reads a vessel's file back and loads it as the server would: one tile through Detour's single-tile init.
    bool MapBuilder::checkVesselTile(uint32 displayId, float const* bmin, float const* bmax) const
    {
        std::string const fileName = Acore::StringFormat(VESSEL_TILE_FILE_NAME_FORMAT, m_config->DataDirPath(),
                                                         displayId);
        FILE* file = fopen(fileName.c_str(), "rb");
        if (!file)
        {
            printf("[Vessel %05u]: no tile written\n", displayId);
            return false;
        }

        MmapTileHeader header;
        unsigned char* data = nullptr;
        bool const read = fread(&header, sizeof(MmapTileHeader), 1, file) == 1 && header.mmapMagic == MMAP_MAGIC
                          && header.dtVersion == uint32(DT_NAVMESH_VERSION) && header.mmapVersion == MMAP_VERSION
                          && (data = static_cast<unsigned char*>(dtAlloc(header.size, DT_ALLOC_PERM)))
                          && fread(data, header.size, 1, file) == 1;
        fclose(file);
        if (!read)
        {
            printf("[Vessel %05u]: %s has no valid header or data\n", displayId, fileName.c_str());
            dtFree(data);
            return false;
        }

        dtNavMesh* navMesh = dtAllocNavMesh();
        if (!navMesh || dtStatusFailed(navMesh->init(data, int(header.size), DT_TILE_FREE_DATA)))
        {
            printf("[Vessel %05u]: the tile does not load as a single-tile navmesh\n", displayId);
            if (navMesh)
                dtFreeNavMesh(navMesh);
            else
                dtFree(data);
            return false;
        }

        dtMeshTile const* tile = static_cast<dtNavMesh const*>(navMesh)->getTile(0);
        int const polygons = tile && tile->header ? tile->header->polyCount : 0;

        // the polygon nearest the bounds' centre, searched across the whole model
        float center[3], extents[3];
        for (int i = 0; i < 3; ++i)
        {
            center[i] = (bmin[i] + bmax[i]) / 2;
            extents[i] = (bmax[i] - bmin[i]) / 2 + 1.0f;
        }

        dtPolyRef polyRef = 0;
        float nearest[3] = {};
        dtNavMeshQuery* query = dtAllocNavMeshQuery();
        dtQueryFilter filter;
        bool const found = query && dtStatusSucceed(query->init(navMesh, 64))
                           && dtStatusSucceed(query->findNearestPoly(center, extents, &filter, &polyRef, nearest))
                           && polyRef;
        dtFreeNavMeshQuery(query);
        dtFreeNavMesh(navMesh);

        if (found)
            printf("[Vessel %05u]: loads as one tile, %d polygons, %u bytes; nearest to the centre (%.2f %.2f %.2f)\n",
                   displayId, polygons, header.size, nearest[2], nearest[0], nearest[1]);
        else
            printf("[Vessel %05u]: loads as one tile, %d polygons, %u bytes; no polygon near the centre\n", displayId,
                   polygons, header.size);

        return polygons > 0;
    }

    /**************************************************************************/
    void MapBuilder::buildSingleTile(uint32 mapID, uint32 tileX, uint32 tileY)
    {
        dtNavMesh* navMesh = nullptr;
        buildNavMesh(mapID, navMesh);
        if (!navMesh)
        {
            printf("Failed creating navmesh!              \n");
            return;
        }

        /// @todo: delete the old tile as the user clearly wants to rebuild it

        TileBuilder tileBuilder = TileBuilder(this, m_skipLiquid, m_debugOutput);
        tileBuilder.buildTile(mapID, tileX, tileY, navMesh);
        dtFreeNavMesh(navMesh);

        _cancelationToken = true;

        _queue.Cancel();
    }

    void TileBuilder::WorkerThread()
    {
        while (true)
        {
            TileInfo tileInfo;

            m_mapBuilder->_queue.WaitAndPop(tileInfo);

            if (m_mapBuilder->_cancelationToken)
                return;

            dtNavMesh* navMesh = dtAllocNavMesh();
            if (!navMesh->init(&tileInfo.m_navMeshParams))
            {
                printf("[Map %04i] Failed creating navmesh for tile %i,%i !\n", tileInfo.m_mapId, tileInfo.m_tileX, tileInfo.m_tileY);
                dtFreeNavMesh(navMesh);
                return;
            }

            buildTile(tileInfo.m_mapId, tileInfo.m_tileX, tileInfo.m_tileY, navMesh);

            dtFreeNavMesh(navMesh);
        }
    }

    /**************************************************************************/
    void MapBuilder::buildMap(uint32 mapID)
    {
        std::set<uint32>* tiles = getTileList(mapID);

        if (!tiles->empty())
        {
            // build navMesh
            dtNavMesh* navMesh = nullptr;
            buildNavMesh(mapID, navMesh);
            if (!navMesh)
            {
                printf("[Map %03i] Failed creating navmesh!\n", mapID);
                m_totalTilesProcessed += tiles->size();
                return;
            }

            // now start building mmtiles for each tile
            printf("[Map %03i] We have %u tiles.                          \n", mapID, (unsigned int)tiles->size());
            for (unsigned int tile : *tiles)
            {
                uint32 tileX, tileY;

                // unpack tile coords
                StaticMapTree::unpackTileID(tile, tileX, tileY);

                TileInfo tileInfo;
                tileInfo.m_mapId = mapID;
                tileInfo.m_tileX = tileX;
                tileInfo.m_tileY = tileY;
                memcpy(&tileInfo.m_navMeshParams, navMesh->getParams(), sizeof(dtNavMeshParams));
                _queue.Push(tileInfo);
            }

            dtFreeNavMesh(navMesh);
        }
    }

    /**************************************************************************/
    void TileBuilder::buildTile(uint32 mapID, uint32 tileX, uint32 tileY, dtNavMesh* navMesh)
    {
        if (shouldSkipTile(mapID, tileX, tileY))
        {
            ++m_mapBuilder->m_totalTilesProcessed;
            return;
        }

        printf("%u%% [Map %04i] Building tile [%02u,%02u]\n", m_mapBuilder->currentPercentageDone(), mapID, tileX, tileY);

        MeshData meshData;

        // get heightmap data
        m_terrainBuilder->loadMap(mapID, tileX, tileY, meshData);

        // get model data
        m_terrainBuilder->loadVMap(mapID, tileY, tileX, meshData);

        // if there is no data, give up now
        if (!meshData.solidVerts.size() && !meshData.liquidVerts.size())
        {
            ++m_mapBuilder->m_totalTilesProcessed;
            return;
        }

        // remove unused vertices
        TerrainBuilder::cleanVertices(meshData.solidVerts, meshData.solidTris);
        TerrainBuilder::cleanVertices(meshData.liquidVerts, meshData.liquidTris);

        // gather all mesh data for final data check, and bounds calculation
        G3D::Array<float> allVerts;
        allVerts.append(meshData.liquidVerts);
        allVerts.append(meshData.solidVerts);

        if (!allVerts.size())
        {
            ++m_mapBuilder->m_totalTilesProcessed;
            return;
        }

        // get bounds of current tile
        float bmin[3], bmax[3];
        m_mapBuilder->getTileBounds(tileX, tileY, allVerts.getCArray(), allVerts.size() / 3, bmin, bmax);
        m_terrainBuilder->loadOffMeshConnections(mapID, tileX, tileY, meshData, m_mapBuilder->getConfig().OffMeshConnections());

        // build navmesh tile
        buildMoveMapTile(mapID, tileX, tileY, meshData, bmin, bmax, navMesh);

        ++m_mapBuilder->m_totalTilesProcessed;
    }

    /**************************************************************************/
    void MapBuilder::buildNavMesh(uint32 mapID, dtNavMesh*& navMesh)
    {
        std::set<uint32>* tiles = getTileList(mapID);

        // old code for non-statically assigned bitmask sizes:
        ///*** calculate number of bits needed to store tiles & polys ***/
        //int tileBits = dtIlog2(dtNextPow2(tiles->size()));
        //if (tileBits < 1) tileBits = 1;                                     // need at least one bit!
        //int polyBits = sizeof(dtPolyRef)*8 - SALT_MIN_BITS - tileBits;

        int polyBits = DT_POLY_BITS;

        int maxTiles = tiles->size();
        int maxPolysPerTile = 1 << polyBits;

        /***          calculate bounds of map         ***/

        uint32 tileXMin = 64, tileYMin = 64, tileXMax = 0, tileYMax = 0, tileX, tileY;
        for (unsigned int tile : *tiles)
        {
            StaticMapTree::unpackTileID(tile, tileX, tileY);

            if (tileX > tileXMax)
                tileXMax = tileX;
            else if (tileX < tileXMin)
                tileXMin = tileX;

            if (tileY > tileYMax)
                tileYMax = tileY;
            else if (tileY < tileYMin)
                tileYMin = tileY;
        }

        // use Max because '32 - tileX' is negative for values over 32
        float bmin[3], bmax[3];
        getTileBounds(tileXMax, tileYMax, nullptr, 0, bmin, bmax);

        /***       now create the navmesh       ***/

        // navmesh creation params
        dtNavMeshParams navMeshParams;
        memset(&navMeshParams, 0, sizeof(dtNavMeshParams));
        navMeshParams.tileWidth = GRID_SIZE;
        navMeshParams.tileHeight = GRID_SIZE;
        rcVcopy(navMeshParams.orig, bmin);
        navMeshParams.maxTiles = maxTiles;
        navMeshParams.maxPolys = maxPolysPerTile;

        navMesh = dtAllocNavMesh();
        printf("[Map %03i] Creating navMesh...\n", mapID);
        if (!navMesh->init(&navMeshParams))
        {
            printf("[Map %03i] Failed creating navmesh!                \n", mapID);
            return;
        }

        const std::string fileName = Acore::StringFormat(
            MAP_FILE_NAME_FORMAT,
            m_config->DataDirPath(),
            mapID
        );

        FILE* file = fopen(fileName.c_str(), "wb");
        if (!file)
        {
            dtFreeNavMesh(navMesh);
            char message[1024];
            sprintf(message, "[Map %03i] Failed to open %s for writing!\n", mapID, fileName.c_str());
            perror(message);
            return;
        }

        // now that we know navMesh params are valid, we can write them to file
        fwrite(&navMeshParams, sizeof(dtNavMeshParams), 1, file);
        fclose(file);
    }

    /**************************************************************************/
    void TileBuilder::buildMoveMapTile(uint32 mapID, uint32 tileX, uint32 tileY,
                                      MeshData& meshData, float bmin[3], float bmax[3],
                                      dtNavMesh* navMesh, Optional<uint32> vesselDisplayId)
    {
        // console output
        char tileString[20];
        if (vesselDisplayId)
            sprintf(tileString, "[Vessel %05u]: ", *vesselDisplayId);
        else
            sprintf(tileString, "[Map %03i] [%02i,%02i]: ", mapID, tileX, tileY);
        printf("%s Building movemap tiles...\n", tileString);

        IntermediateValues iv;

        float* tVerts = meshData.solidVerts.getCArray();
        int tVertCount = meshData.solidVerts.size() / 3;
        int* tTris = meshData.solidTris.getCArray();
        int tTriCount = meshData.solidTris.size() / 3;

        float* lVerts = meshData.liquidVerts.getCArray();
        int lVertCount = meshData.liquidVerts.size() / 3;
        int* lTris = meshData.liquidTris.getCArray();
        int lTriCount = meshData.liquidTris.size() / 3;
        std::vector<unsigned char> lRasterAreas(lTriCount);
        for (int i = 0; i < lTriCount; ++i)
            lRasterAreas[i] = rasterArea(meshData.liquidType[i]);
        unsigned char* lTriFlags = lRasterAreas.data();

        // a vessel's model takes the global meshSettings: no map or tile override applies to it
        ResolvedMeshConfig cfg = vesselDisplayId ? m_mapBuilder->getConfig().GetGlobalConfig()
                                                 : m_mapBuilder->getConfig().GetConfigForTile(mapID, tileX, tileY);
        int tilesPerMap = cfg.tilesPerMapEdge;
        float baseUnitDim = cfg.baseUnitDim;

        rcConfig config = m_mapBuilder->getRecastConfig(cfg, bmin, bmax);

        // a vessel's deck or lift floor is a lone island smaller than the map's minimum region; keep every island
        if (vesselDisplayId)
            config.minRegionArea = 0;

        // this sets the dimensions of the heightfield - should maybe happen before border padding
        rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

        // a map tile is built in tilesPerMap x tilesPerMap sub-tiles, a vessel in as many as its model spans
        int subTilesX = tilesPerMap;
        int subTilesY = tilesPerMap;
        if (vesselDisplayId)
        {
            float const subTileSize = config.tileSize * config.cs;
            subTilesX = std::max(1, int(std::ceil((bmax[0] - bmin[0]) / subTileSize)));
            subTilesY = std::max(1, int(std::ceil((bmax[2] - bmin[2]) / subTileSize)));
        }

        // the marked areas of the tile (none for a vessel): walkable by source triangle and in the ledge filter,
        // blocked and plain by cell
        std::vector<Config::MarkedArea const*> walkableAreas;
        std::vector<Config::MarkedArea const*> cellAreas;
        if (!vesselDisplayId)
        {
            std::vector<Config::MarkedArea> const& areas = m_mapBuilder->getConfig().MarkedAreas();
            float const pad = config.borderSize * config.cs;
            walkableAreas = markedAreasIn(areas, mapID, config.bmin, config.bmax, pad, true);
            cellAreas = markedAreasIn(areas, mapID, config.bmin, config.bmax, pad, false);
        }
        std::vector<int> const walkableTris = walkableTriangles(walkableAreas, tVerts, tTris, tTriCount);
        TileCuts cuts;

        // Detour links a polygon to another of its tile only below index 0x7fff: it stores the neighbour as
        // index + 1 and reads bit 0x8000 as a border or portal. A merged tile over that many polygons loses links,
        // so it is rebuilt with a larger maxSimplificationError (at least 1.0, then +0.2) until it fits.
        int const maxLinkedPolys = 0x7fff - 1;
        int const maxRebuilds = 10;
        float const firstSimplificationError = cfg.maxSimplificationError;
        float simplificationError = firstSimplificationError;
        int firstPolyCount = 0;
        for (int rebuild = 0; rebuild <= maxRebuilds; ++rebuild)
        {
            // allocate subregions : tiles
            Tile* tiles = new Tile[subTilesX * subTilesY];

            // Initialize per tile config.
            rcConfig tileCfg = config;
            tileCfg.width = config.tileSize + config.borderSize * 2;
            tileCfg.height = config.tileSize + config.borderSize * 2;
            tileCfg.maxSimplificationError = simplificationError;

            // merge per tile poly and detail meshes
            rcPolyMesh** pmmerge = new rcPolyMesh*[subTilesX * subTilesY];
            rcPolyMeshDetail** dmmerge = new rcPolyMeshDetail*[subTilesX * subTilesY];
            int nmerge = 0;

            // build all tiles
            for (int y = 0; y < subTilesY; ++y)
            {
                for (int x = 0; x < subTilesX; ++x)
                {
                    Tile& tile = tiles[x + y * subTilesX];

                    // Calculate the per tile bounding box.
                    tileCfg.bmin[0] = config.bmin[0] + x * float(config.tileSize * config.cs);
                    tileCfg.bmin[2] = config.bmin[2] + y * float(config.tileSize * config.cs);
                    tileCfg.bmax[0] = config.bmin[0] + (x + 1) * float(config.tileSize * config.cs);
                    tileCfg.bmax[2] = config.bmin[2] + (y + 1) * float(config.tileSize * config.cs);

                    tileCfg.bmin[0] -= tileCfg.borderSize * tileCfg.cs;
                    tileCfg.bmin[2] -= tileCfg.borderSize * tileCfg.cs;
                    tileCfg.bmax[0] += tileCfg.borderSize * tileCfg.cs;
                    tileCfg.bmax[2] += tileCfg.borderSize * tileCfg.cs;

                    // build heightfield
                    tile.solid = rcAllocHeightfield();
                    if (!tile.solid || !rcCreateHeightfield(m_rcContext, *tile.solid, tileCfg.width, tileCfg.height, tileCfg.bmin, tileCfg.bmax, tileCfg.cs, tileCfg.ch))
                    {
                        printf("%s Failed building heightfield!            \n", tileString);
                        continue;
                    }

                    // mark all walkable tiles, both liquids and solids

                    unsigned char* triFlags = new unsigned char[tTriCount];
                    memset(triFlags, NAV_GROUND, tTriCount * sizeof(unsigned char));
                    rcClearUnwalkableTriangles(m_rcContext, tileCfg.walkableSlopeAngle, tVerts, tVertCount, tTris, tTriCount, triFlags);
                    // mod_playerbots (bots should not attempt to use paths that includes slopes beyound 50 degrees)
                    // A steep face that rises no more than one step is a step: rasterArea puts the tag below plain
                    // ground, so the cell where the face meets plain ground stays plain ground.
                    modAlmostUnwalkableTriangles(50.0f, tVerts, tVertCount, tTris, tTriCount, triFlags);
                    for (int tri : walkableTris)
                        triFlags[tri] = NAV_GROUND;
                    for (int i = 0; i < tTriCount; ++i)
                        triFlags[i] = rasterArea(triFlags[i]);
                    rcRasterizeTriangles(m_rcContext, tVerts, tVertCount, tTris, triFlags, tTriCount, *tile.solid, config.walkableClimb);
                    delete[] triFlags;

                    // the cut counts: walkable spans before and after each of the generator's own filters
                    bool const countCuts = rebuild == 0;
                    int const border = tileCfg.borderSize;
                    long long walkable = countCuts ? walkableSpans(*tile.solid, border) : 0;
                    cuts.raster += walkable;
                    auto countCut = [&](long long& cut)
                    {
                        if (!countCuts)
                            return;
                        long long const now = walkableSpans(*tile.solid, border);
                        cut += walkable - now;
                        walkable = now;
                    };

                    long long lowObstacleLoss = 0;
                    rcFilterLowHangingWalkableObstacles(m_rcContext, config.walkableClimb, *tile.solid);
                    countCut(lowObstacleLoss);
                    cuts.lowObstacle -= lowObstacleLoss;
                    filterLedgeSpans(m_rcContext, walkableAreas, tileCfg.walkableHeight, tileCfg.walkableClimb,
                                     *tile.solid);
                    countCut(cuts.ledge);
                    rcFilterWalkableLowHeightSpans(m_rcContext, tileCfg.walkableHeight, *tile.solid);
                    countCut(cuts.lowCeiling);

                    // add liquid triangles
                    rcRasterizeTriangles(m_rcContext, lVerts, lVertCount, lTris, lTriFlags, lTriCount, *tile.solid, config.walkableClimb);

                    // compact heightfield spans
                    tile.chf = rcAllocCompactHeightfield();
                    if (!tile.chf || !rcBuildCompactHeightfield(m_rcContext, tileCfg.walkableHeight, tileCfg.walkableClimb, *tile.solid, *tile.chf))
                    {
                        printf("%s Failed compacting heightfield!            \n", tileString);
                        continue;
                    }

                    if (!cellAreas.empty())
                        markCells(cellAreas, *tile.chf);

                    long long const compact = countCuts ? walkableSpans(*tile.chf, border, false) : 0;
                    cuts.compact += compact;

                    // build polymesh intermediates
                    if (!rcErodeWalkableArea(m_rcContext, config.walkableRadius, *tile.chf))
                    {
                        printf("%s Failed eroding area!                    \n", tileString);
                        continue;
                    }
                    if (countCuts)
                        cuts.radius += compact - walkableSpans(*tile.chf, border, false);

                    if (!rcBuildDistanceField(m_rcContext, *tile.chf))
                    {
                        printf("%s Failed building distance field!         \n", tileString);
                        continue;
                    }

                    if (!rcBuildRegions(m_rcContext, *tile.chf, tileCfg.borderSize, tileCfg.minRegionArea, tileCfg.mergeRegionArea))
                    {
                        printf("%s Failed building regions!                \n", tileString);
                        continue;
                    }
                    if (countCuts)
                        cuts.smallRegion += walkableSpans(*tile.chf, border, true);

                    tile.cset = rcAllocContourSet();
                    if (!tile.cset || !rcBuildContours(m_rcContext, *tile.chf, tileCfg.maxSimplificationError, tileCfg.maxEdgeLen, *tile.cset))
                    {
                        printf("%s Failed building contours!               \n", tileString);
                        continue;
                    }

                    // build polymesh
                    tile.pmesh = rcAllocPolyMesh();
                    if (!tile.pmesh || !rcBuildPolyMesh(m_rcContext, *tile.cset, tileCfg.maxVertsPerPoly, *tile.pmesh))
                    {
                        printf("%s Failed building polymesh!               \n", tileString);
                        continue;
                    }

                    tile.dmesh = rcAllocPolyMeshDetail();
                    if (!tile.dmesh || !rcBuildPolyMeshDetail(m_rcContext, *tile.pmesh, *tile.chf, tileCfg.detailSampleDist, tileCfg.detailSampleMaxError, *tile.dmesh))
                    {
                        printf("%s Failed building polymesh detail!        \n", tileString);
                        continue;
                    }

                    // free those up
                    // we may want to keep them in the future for debug
                    // but right now, we don't have the code to merge them
                    rcFreeHeightField(tile.solid);
                    tile.solid = nullptr;
                    rcFreeCompactHeightfield(tile.chf);
                    tile.chf = nullptr;
                    rcFreeContourSet(tile.cset);
                    tile.cset = nullptr;

                    pmmerge[nmerge] = tile.pmesh;
                    dmmerge[nmerge] = tile.dmesh;
                    nmerge++;
                }
            }

            iv.polyMesh = rcAllocPolyMesh();
            if (!iv.polyMesh)
            {
                printf("%s alloc iv.polyMesh FAILED!\n", tileString);
                delete[] pmmerge;
                delete[] dmmerge;
                delete[] tiles;
                return;
            }
            rcMergePolyMeshes(m_rcContext, pmmerge, nmerge, *iv.polyMesh);

            iv.polyMeshDetail = rcAllocPolyMeshDetail();
            if (!iv.polyMeshDetail)
            {
                printf("%s alloc m_dmesh FAILED!\n", tileString);
                delete[] pmmerge;
                delete[] dmmerge;
                delete[] tiles;
                return;
            }
            rcMergePolyMeshDetails(m_rcContext, dmmerge, nmerge, *iv.polyMeshDetail);

            // join the floor the merge left open on the sub-tile lines (a vessel's model is left as it is)
            if (!vesselDisplayId && iv.polyMesh->npolys)
            {
                std::vector<int> lines[2];
                int border[2][2];
                float const step = config.tileSize * config.cs;
                for (int a = 0; a < 2; ++a)
                {
                    int const axis = a * 2;
                    int const count = a ? subTilesY : subTilesX;
                    float const origin = iv.polyMesh->bmin[axis];
                    auto cell = [&](float w) { return int(std::floor((w - origin) / config.cs + 0.5f)); };
                    border[a][0] = cell(config.bmin[axis]);
                    border[a][1] = cell(config.bmin[axis] + count * step);
                    for (int k = 1; k < count; ++k)
                        lines[a].push_back(cell(config.bmin[axis] + k * step));
                }
                cuts.seams = joinSubTileSeams(*iv.polyMesh, *iv.polyMeshDetail, lines, border, config.walkableClimb);
            }

            // free things up
            delete[] pmmerge;
            delete[] dmmerge;
            delete[] tiles;

            if (!rebuild)
                firstPolyCount = iv.polyMesh->npolys;

            if (iv.polyMesh->npolys <= maxLinkedPolys || rebuild == maxRebuilds)
                break;

            rcFreePolyMesh(iv.polyMesh);
            iv.polyMesh = nullptr;
            rcFreePolyMeshDetail(iv.polyMeshDetail);
            iv.polyMeshDetail = nullptr;
            simplificationError = std::max(1.0f, simplificationError + 0.2f);
        }

        if (simplificationError != firstSimplificationError)
        {
            printf("%s Dense tile: %d polygons at maxSimplificationError %.1f, %d at %.1f%s\n", tileString,
                   firstPolyCount, firstSimplificationError, iv.polyMesh->npolys, simplificationError,
                   iv.polyMesh->npolys > maxLinkedPolys ? ", still over the link limit" : "");
            cfg.maxSimplificationError = simplificationError; // the tile header records the value used
        }

        // set polygons as walkable
        /// @todo: special flags for DYNAMIC polygons, ie surfaces that can be turned on and off
        for (int i = 0; i < iv.polyMesh->npolys; ++i)
        {
            iv.polyMesh->areas[i] = navArea(iv.polyMesh->areas[i]);
            if (iv.polyMesh->areas[i] & RC_WALKABLE_AREA)
                iv.polyMesh->flags[i] = iv.polyMesh->areas[i];
        }

        // setup mesh parameters
        dtNavMeshCreateParams params;
        memset(&params, 0, sizeof(params));
        params.verts = iv.polyMesh->verts;
        params.vertCount = iv.polyMesh->nverts;
        params.polys = iv.polyMesh->polys;
        params.polyAreas = iv.polyMesh->areas;
        params.polyFlags = iv.polyMesh->flags;
        params.polyCount = iv.polyMesh->npolys;
        params.nvp = iv.polyMesh->nvp;
        params.detailMeshes = iv.polyMeshDetail->meshes;
        params.detailVerts = iv.polyMeshDetail->verts;
        params.detailVertsCount = iv.polyMeshDetail->nverts;
        params.detailTris = iv.polyMeshDetail->tris;
        params.detailTriCount = iv.polyMeshDetail->ntris;

        params.offMeshConVerts = meshData.offMeshConnections.getCArray();
        params.offMeshConCount = meshData.offMeshConnections.size() / 6;
        params.offMeshConRad = meshData.offMeshConnectionRads.getCArray();
        params.offMeshConDir = meshData.offMeshConnectionDirs.getCArray();
        params.offMeshConAreas = meshData.offMeshConnectionsAreas.getCArray();
        params.offMeshConFlags = meshData.offMeshConnectionsFlags.getCArray();

        params.walkableHeight = baseUnitDim * config.walkableHeight;  // agent height
        params.walkableRadius = baseUnitDim * config.walkableRadius;  // agent radius
        params.walkableClimb = baseUnitDim * config.walkableClimb;    // keep less that walkableHeight (aka agent height)!
        if (vesselDisplayId)
        {
            // a vessel's mesh is one tile at the origin of its own navmesh
            params.tileX = 0;
            params.tileY = 0;
        }
        else
        {
            params.tileX = (((bmin[0] + bmax[0]) / 2) - navMesh->getParams()->orig[0]) / GRID_SIZE;
            params.tileY = (((bmin[2] + bmax[2]) / 2) - navMesh->getParams()->orig[2]) / GRID_SIZE;
        }
        rcVcopy(params.bmin, bmin);
        rcVcopy(params.bmax, bmax);
        params.cs = config.cs;
        params.ch = config.ch;
        params.tileLayer = 0;
        params.buildBvTree = true;

        // will hold final navmesh
        unsigned char* navData = nullptr;
        int navDataSize = 0;

        do
        {
            // these values are checked within dtCreateNavMeshData - handle them here
            // so we have a clear error message
            if (params.nvp > DT_VERTS_PER_POLYGON)
            {
                printf("%s Invalid verts-per-polygon value!        \n", tileString);
                break;
            }
            if (params.vertCount >= 0xffff)
            {
                printf("%s Too many vertices! %d out of %d!        \n", tileString, params.vertCount, 0xffff);
                break;
            }
            if (!params.vertCount || !params.verts)
            {
                // occurs mostly when adjacent tiles have models
                // loaded but those models don't span into this tile

                // message is an annoyance
                printf("%sNo vertices to build tile!              \n", tileString);
                break;
            }
            if (!params.polyCount || !params.polys)
            {
                // we have flat tiles with no actual geometry - don't build those, its useless
                // keep in mind that we do output those into debug info
                printf("%s No polygons to build on tile!              \n", tileString);
                break;
            }
            if (!params.detailMeshes || !params.detailVerts || !params.detailTris)
            {
                printf("%s No detail mesh to build tile!           \n", tileString);
                break;
            }

            printf("%s Building navmesh tile...\n", tileString);
            if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
            {
                printf("%s Failed building navmesh tile!           \n", tileString);
                break;
            }

            dtTileRef tileRef = 0;
            printf("%s Adding tile to navmesh...\n", tileString);
            // DT_TILE_FREE_DATA tells detour to unallocate memory when the tile
            // is removed via removeTile()
            dtStatus dtResult = navMesh->addTile(navData, navDataSize, DT_TILE_FREE_DATA, 0, &tileRef);
            if (!tileRef || dtResult != DT_SUCCESS)
            {
                printf("%s Failed adding tile to navmesh!           \n", tileString);
                break;
            }

            // file output
            const std::string fileName = vesselDisplayId
                ? Acore::StringFormat(VESSEL_TILE_FILE_NAME_FORMAT, m_mapBuilder->getConfig().DataDirPath(),
                                      *vesselDisplayId)
                : Acore::StringFormat(
                    TILE_FILE_NAME_FORMAT,
                    m_mapBuilder->getConfig().DataDirPath(),
                    mapID, tileY, tileX
                );

            FILE* file = fopen(fileName.c_str(), "wb");
            if (!file)
            {
                char message[1024];
                sprintf(message, "[Map %03i] Failed to open %s for writing!\n", mapID, fileName.c_str());
                perror(message);
                navMesh->removeTile(tileRef, nullptr, nullptr);
                break;
            }

            printf("%s Writing to file...\n", tileString);

            // write header
            MmapTileHeader header;
            header.usesLiquids = !vesselDisplayId && m_terrainBuilder->usesLiquids();
            header.size = uint32(navDataSize);
            header.recastConfig = cfg.toMMAPTileRecastConfig();
            fwrite(&header, sizeof(MmapTileHeader), 1, file);

            // write data
            fwrite(navData, sizeof(unsigned char), navDataSize, file);
            fclose(file);

            if (!vesselDisplayId)
                writeTileCuts(fileName, mapID, tileX, tileY, config.cs, cuts);

            // now that tile is written to disk, we can unload it
            navMesh->removeTile(tileRef, nullptr, nullptr);
        } while (false);

        if (m_debugOutput && !vesselDisplayId)
        {
            // restore padding so that the debug visualization is correct
            for (int i = 0; i < iv.polyMesh->nverts; ++i)
            {
                unsigned short* v = &iv.polyMesh->verts[i * 3];
                v[0] += (unsigned short)config.borderSize;
                v[2] += (unsigned short)config.borderSize;
            }

            iv.generateObjFile(m_mapBuilder->getConfig().DataDirPath(), mapID, tileX, tileY, meshData);
            iv.writeIV(m_mapBuilder->getConfig().DataDirPath(), mapID, tileX, tileY);
        }
    }

    /**************************************************************************/
    void MapBuilder::getTileBounds(uint32 tileX, uint32 tileY, float* verts, int vertCount, float* bmin, float* bmax) const
    {
        // this is for elevation
        if (verts && vertCount)
            rcCalcBounds(verts, vertCount, bmin, bmax);
        else
        {
            bmin[1] = FLT_MIN;
            bmax[1] = FLT_MAX;
        }

        // this is for width and depth
        bmax[0] = (32 - int(tileX)) * GRID_SIZE;
        bmax[2] = (32 - int(tileY)) * GRID_SIZE;
        bmin[0] = bmax[0] - GRID_SIZE;
        bmin[2] = bmax[2] - GRID_SIZE;
    }

    /**************************************************************************/
    bool MapBuilder::shouldSkipMap(uint32 mapID) const
    {
        if (m_mapid >= 0)
            return static_cast<uint32>(m_mapid) != mapID;

        if (m_skipContinents)
            if (isContinentMap(mapID))
                return true;

        if (m_skipJunkMaps)
            switch (mapID)
            {
                case 13:    // test.wdt
                case 25:    // ScottTest.wdt
                case 29:    // Test.wdt
                case 42:    // Colin.wdt
                case 169:   // EmeraldDream.wdt (unused, and very large)
                case 451:   // development.wdt
                case 573:   // ExteriorTest.wdt
                case 597:   // CraigTest.wdt
                case 605:   // development_nonweighted.wdt
                case 606:   // QA_DVD.wdt
                    return true;
                default:
                    if (isTransportMap(mapID))
                        return true;
                    break;
            }

        if (m_skipBattlegrounds)
            switch (mapID)
            {
                case 30:    // AV
                case 37:    // ?
                case 489:   // WSG
                case 529:   // AB
                case 566:   // EotS
                case 607:   // SotA
                case 628:   // IoC
                    return true;
                default:
                    break;
            }

        return false;
    }

    /**************************************************************************/
    bool MapBuilder::isTransportMap(uint32 mapID) const
    {
        switch (mapID)
        {
            // transport maps
            case 582:
            case 584:
            case 586:
            case 587:
            case 588:
            case 589:
            case 590:
            case 591:
            case 592:
            case 593:
            case 594:
            case 596:
            case 610:
            case 612:
            case 613:
            case 614:
            case 620:
            case 621:
            case 622:
            case 623:
            case 641:
            case 642:
            case 647:
            case 672:
            case 673:
            case 712:
            case 713:
            case 718:
                return true;
            default:
                return false;
        }
    }

    bool MapBuilder::isContinentMap(uint32 mapID) const
    {
        switch (mapID)
        {
            case 0:
            case 1:
            case 530:
            case 571:
                return true;
            default:
                return false;
        }
    }

    /**************************************************************************/
    bool TileBuilder::shouldSkipTile(uint32 mapID, uint32 tileX, uint32 tileY) const
    {
        const std::string fileName = Acore::StringFormat(
            TILE_FILE_NAME_FORMAT,
            m_mapBuilder->getConfig().DataDirPath(),
            mapID, tileY, tileX
        );

        FILE* file = fopen(fileName.c_str(), "rb");
        if (!file)
            return false;

        MmapTileHeader header;
        int count = fread(&header, sizeof(MmapTileHeader), 1, file);
        fclose(file);
        if (count != 1)
            return false;

        if (header.mmapMagic != MMAP_MAGIC || header.dtVersion != uint32(DT_NAVMESH_VERSION))
            return false;

        if (header.mmapVersion != MMAP_VERSION)
            return false;

        const auto desiredRecastConfig = m_mapBuilder->getConfig().GetConfigForTile(mapID, tileX, tileY).toMMAPTileRecastConfig();
        return header.recastConfig == desiredRecastConfig;
    }

    rcConfig MapBuilder::getRecastConfig(ResolvedMeshConfig const& cfg, float bmin[3], float bmax[3]) const
    {
        rcConfig config;
        memset(&config, 0, sizeof(rcConfig));

        rcVcopy(config.bmin, bmin);
        rcVcopy(config.bmax, bmax);

        config.maxVertsPerPoly = DT_VERTS_PER_POLYGON;
        config.cs = cfg.cellSizeHorizontal;
        config.ch = cfg.cellSizeVertical;
        config.walkableSlopeAngle = cfg.walkableSlopeAngle;
        config.tileSize = cfg.vertexPerTileEdge;
        config.walkableRadius = cfg.walkableRadius;
        config.borderSize = cfg.walkableRadius + 3;
        config.maxEdgeLen = cfg.vertexPerTileEdge + 1;              // anything bigger than tileSize
        config.walkableHeight = cfg.walkableHeight;
        config.walkableClimb = cfg.walkableClimb;
        config.minRegionArea = rcSqr(60);
        config.mergeRegionArea = rcSqr(50);
        config.maxSimplificationError = cfg.maxSimplificationError; // eliminates most jagged edges (tiny polygons)
        config.detailSampleDist = config.cs * 16;
        config.detailSampleMaxError = config.ch * 1;
        return config;
    }

    /**************************************************************************/
    uint32 MapBuilder::percentageDone(uint32 totalTiles, uint32 totalTilesBuilt) const
    {
        if (totalTiles)
            return totalTilesBuilt * 100 / totalTiles;

        return 0;
    }

    uint32 MapBuilder::currentPercentageDone() const
    {
        return percentageDone(m_totalTiles, m_totalTilesProcessed);
    }
}
