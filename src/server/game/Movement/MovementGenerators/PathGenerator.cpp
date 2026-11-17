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

#include "PathGenerator.h"
#include "Creature.h"
#include "DetourCommon.h"
#include "Geometry.h"
#include "Log.h"
#include "MMapMgr.h"
#include "Map.h"
#include "Metric.h"
#include "Player.h"
#include "Random.h"
#include "Transport.h"
#include "VesselFrame.h"
#include "WorldSession.h"

// Blades Edge Arena Ropes normalization
namespace
{
    constexpr float BLADE_EDGE_ROPE_SNAP_DIST = 1.5f;
    constexpr float BLADE_EDGE_ROPE_SNAP_DIST2 = BLADE_EDGE_ROPE_SNAP_DIST * BLADE_EDGE_ROPE_SNAP_DIST;

    struct BladeEdgeArenaRope
    {
        G3D::Vector3 Start;
        G3D::Vector3 End;
        float Sag;
    };

    static const std::array<BladeEdgeArenaRope, 2> BladeEdgeArenaRopes =
    {{
        {
            {6243.1523f, 267.53094f, 10.929295f},
            {6245.9717f, 271.29346f, 10.879172f},
            0.43f
        },
        {
            {6234.3213f, 256.29733f, 11.002348f},
            {6231.3247f, 252.58781f, 10.976968f},
            0.46f
        }
    }};

    bool IsOutsideExpandedXYBounds(G3D::Vector3 const& point, BladeEdgeArenaRope const& rope)
    {
        float const minX = std::min(rope.Start.x, rope.End.x) - BLADE_EDGE_ROPE_SNAP_DIST;
        float const maxX = std::max(rope.Start.x, rope.End.x) + BLADE_EDGE_ROPE_SNAP_DIST;
        float const minY = std::min(rope.Start.y, rope.End.y) - BLADE_EDGE_ROPE_SNAP_DIST;
        float const maxY = std::max(rope.Start.y, rope.End.y) + BLADE_EDGE_ROPE_SNAP_DIST;

        return point.x < minX || point.x > maxX || point.y < minY || point.y > maxY;
    }

    bool GetClosestPointOnBladeEdgeArenaRope(G3D::Vector3 const& point, BladeEdgeArenaRope const& rope, G3D::Vector3& closestPoint)
    {
        G3D::Vector3 const ropeVector = rope.End - rope.Start;

        float const ropeLength2XY = ropeVector.x * ropeVector.x + ropeVector.y * ropeVector.y;
        if (ropeLength2XY < 0.00001f)
            return false;

        G3D::Vector3 const pointVector = point - rope.Start;

        float t = (pointVector.x * ropeVector.x + pointVector.y * ropeVector.y) / ropeLength2XY;
        t = std::clamp(t, 0.0f, 1.0f);

        float const closestX = rope.Start.x + ropeVector.x * t;
        float const closestY = rope.Start.y + ropeVector.y * t;

        float const dx = point.x - closestX;
        float const dy = point.y - closestY;

        // If the point is already too far in XY, it cannot be within the 3D snap radius.
        if (dx * dx + dy * dy >= BLADE_EDGE_ROPE_SNAP_DIST2)
            return false;

        float const linearZ = rope.Start.z + (rope.End.z - rope.Start.z) * t;
        float const sagZ = rope.Sag * std::sin(M_PI * t);

        closestPoint = { closestX, closestY, linearZ - sagZ };
        return true;
    }

    bool TrySnapToBladeEdgeArenaRope(G3D::Vector3& point)
    {
        bool snapped = false;
        float bestDist2 = BLADE_EDGE_ROPE_SNAP_DIST2;
        G3D::Vector3 bestPoint;

        for (BladeEdgeArenaRope const& rope : BladeEdgeArenaRopes)
        {
            if (IsOutsideExpandedXYBounds(point, rope))
                continue;

            G3D::Vector3 closestPoint;
            if (!GetClosestPointOnBladeEdgeArenaRope(point, rope, closestPoint))
                continue;

            float const dist2 = (point - closestPoint).squaredLength();
            if (dist2 < bestDist2)
            {
                bestDist2 = dist2;
                bestPoint = closestPoint;
                snapped = true;
            }
        }

        if (snapped)
            point = bestPoint;

        return snapped;
    }
}

 ////////////////// PathGenerator //////////////////
PathGenerator::PathGenerator(WorldObject const* owner) :
    _polyLength(0), _type(PATHFIND_BLANK), _useStraightPath(false), _forceDestination(false),
    _slopeCheck(false), _pointPathLimit(MAX_POINT_PATH_LENGTH), _useRaycast(false),
    _endPosition(G3D::Vector3::zero()), _source(owner), _navMesh(nullptr),
    _navMeshQuery(nullptr)
{
    memset(_pathPolyRefs, 0, sizeof(_pathPolyRefs));

    //if (sDisableMgr->IsPathfindingEnabled(_sourceUnit->FindMap()))
    {
        _navMesh = _source->GetMap()->GetMapCollisionData().GetMMapData().GetNavMesh();
        _navMeshQuery = _source->GetMap()->GetMapCollisionData().GetMMapData().GetNavMeshQuery();
    }

    CreateFilter();
}

PathGenerator::PathGenerator(dtNavMesh const* navMesh, dtNavMeshQuery const* navMeshQuery, dtQueryFilterExt const& filter) :
    _polyLength(0), _type(PATHFIND_BLANK), _useStraightPath(false), _forceDestination(false),
    _slopeCheck(false), _pointPathLimit(MAX_POINT_PATH_LENGTH), _useRaycast(false),
    _endPosition(G3D::Vector3::zero()), _source(nullptr), _navMesh(navMesh),
    _navMeshQuery(navMeshQuery), _filter(filter)
{
    memset(_pathPolyRefs, 0, sizeof(_pathPolyRefs));
}

PathGenerator::~PathGenerator()
{
}

bool PathGenerator::CalculatePath(float destX, float destY, float destZ, bool forceDest)
{
    float x, y, z;
    _source->GetPosition(x, y, z);

    return CalculatePath(x, y, z, destX, destY, destZ, forceDest);
}

bool PathGenerator::CalculatePath(float x, float y, float z, float destX, float destY, float destZ, bool forceDest)
{
    if (!Acore::IsValidMapCoord(destX, destY, destZ) || !Acore::IsValidMapCoord(x, y, z))
        return false;

    METRIC_DETAILED_EVENT("mmap_events", "CalculatePath", "");

    G3D::Vector3 dest(destX, destY, destZ);
    SetEndPosition(dest);

    G3D::Vector3 start(x, y, z);
    SetStartPosition(start);

    _forceDestination = forceDest;

    // Vessel mode is decided at each call: a movement generator keeps its path object over many calls, and its unit
    // boards or leaves between two of them.
    if (CalculateVesselPath(start, dest))
        return true;

    // make sure navMesh works - we can run on map w/o mmap
    // check if the start and end point have a .mmtile loaded (can we pass via not loaded tile on the way?)
    Unit const* _sourceUnit = _source->ToUnit();
    if (!_navMesh || !_navMeshQuery || (_sourceUnit && _sourceUnit->HasUnitState(UNIT_STATE_IGNORE_PATHFINDING)) ||
        !HaveTile(start) || !HaveTile(dest))
    {
        BuildShortcut();
        _type = PathType(PATHFIND_NORMAL | PATHFIND_NOT_USING_PATH);
        return true;
    }

    UpdateFilter();

    BuildPolyPath(start, dest);
    return true;
}

// Vessel mode: a unit aboard a transport whose model has a deck mesh gets its path on that mesh, in the vessel's own
// coordinates, and the answer in world points. Returns false when the call is not one for vessel mode; the world's
// search then runs as it always did.
//
// Rule 1, no mesh: a transport without a mesh (no file, no folder, a size other than 1) takes the world's code path
// unchanged, straight line included. A unit on a vehicle and a unit on the ground are untouched. A unit on the dock
// that asks for a point on a deck paths on the world's mesh.
//
// Rule 2, deck to dock: an end that is not on the deck (a point on the dock, in the water) is PATHFIND_NOPATH, with
// no straight line; so is a start off the deck's mesh. The rule is this answer only: what a caller does with it is
// the caller's (the point generator and MoveSplineInit::MoveTo start a straight two-point move on no path, as for a
// creature on the ground).
//
// None of the world's lookups runs on the vessel's coordinates, where they would read the world at an offset or
// decide a straight line off the deck: the tile check, the player's shortcut over a hole in the mesh, the water at
// the ends with the swim, fly and fall shortcut, the ground height of each point (NormalizePath, also in
// BuildShortcut), the slope check with its water test, and the forced destination. BuildVesselPath says what each
// case answers instead. UpdateFilter reads the unit's own world position and runs as on the ground.
//
// The spline tells "aboard" by the unit's movement flag and transport guid (MoveSplineInit::Launch), this test by
// the transport pointer. The two can differ while a unit boards or leaves. That is harmless: the points returned are
// world points, right for both, and the spline turns them into offsets itself when it moves the unit aboard.
bool PathGenerator::CalculateVesselPath(G3D::Vector3 const& start, G3D::Vector3 const& dest)
{
    Unit const* unit = _source->ToUnit();
    if (!unit || unit->GetVehicle())
        return false;

    Transport const* transport = unit->GetTransport();
    if (!transport || transport->FindMap() != unit->FindMap())
        return false;

    // a unit told to ignore the mesh gets the world's straight line, as before
    std::shared_ptr<dtNavMesh const> const& navMesh = transport->GetVesselNavMesh();
    if (!navMesh || unit->HasUnitState(UNIT_STATE_IGNORE_PATHFINDING))
        return false;

    // a vessel sails from map to map: the query is the one of the map the unit is on now
    dtNavMeshQuery const* navMeshQuery = unit->GetMap()->GetMapCollisionData().GetMMapData().GetVesselNavMeshQuery(transport->GetDisplayId(), navMesh);
    if (!navMeshQuery)
        return false;

    // The vessel's pose at this call turns both ways. A call that starts at the unit takes the unit's own offset,
    // which is exact; its world position is that offset turned by the pose of the vessel's last move.
    VesselFrame::Pose const pose{ transport->GetPositionX(), transport->GetPositionY(), transport->GetPositionZ(), transport->GetOrientation() };

    G3D::Vector3 startOffset = start;
    if (start.x == unit->GetPositionX() && start.y == unit->GetPositionY() && start.z == unit->GetPositionZ())
        startOffset = G3D::Vector3(unit->GetTransOffsetX(), unit->GetTransOffsetY(), unit->GetTransOffsetZ());
    else
        VesselFrame::ToOffset(pose, startOffset.x, startOffset.y, startOffset.z);

    G3D::Vector3 endOffset = dest;
    VesselFrame::ToOffset(pose, endOffset.x, endOffset.y, endOffset.z);

    UpdateFilter();

    Movement::PointsArray points;
    PathType const type = FindVesselPath(navMesh.get(), navMeshQuery, _filter, startOffset, endOffset, points, _pointPathLimit, _useStraightPath, _useRaycast);

    // The polygon path kept from earlier calls is one on the world's mesh, and this call ran on another mesh: it is
    // dropped, so the next call on the world's mesh starts afresh. The vessel search keeps none.
    Clear();

    if ((type & PATHFIND_NOPATH) || points.size() < 2)
    {
        // As on the ground, no path still carries the two ends, since callers read GetPath() before the type. It is
        // an answer, not a line to walk: the type says no path.
        _pathPoints.resize(2);
        _pathPoints[0] = start;
        _pathPoints[1] = dest;
        _type = PATHFIND_NOPATH;
        return true;
    }

    _pathPoints.resize(points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        G3D::Vector3 point = points[i];
        VesselFrame::ToWorld(pose, point.x, point.y, point.z);
        _pathPoints[i] = point;
    }

    SetActualEndPosition(_pathPoints.back());
    _type = type;
    return true;
}

PathType PathGenerator::FindVesselPath(dtNavMesh const* navMesh, dtNavMeshQuery const* navMeshQuery, dtQueryFilterExt const& filter,
    G3D::Vector3 const& start, G3D::Vector3 const& end, Movement::PointsArray& points, uint32 pointPathLimit, bool useStraightPath, bool useRaycast)
{
    points.clear();
    if (!navMesh || !navMeshQuery)
        return PATHFIND_NOPATH;

    // The search runs in a path object of its own, without an owner: it shares the world's polygon lookup and
    // smoothing (GetPolyByLocation, FindSmoothPath) and cannot reach a world lookup. The slope check stays off.
    PathGenerator search(navMesh, navMeshQuery, filter);
    search._pointPathLimit = std::min<uint32>(pointPathLimit, MAX_POINT_PATH_LENGTH);
    search._useStraightPath = useStraightPath;
    search._useRaycast = useRaycast;
    return search.BuildVesselPath(start, end, points);
}

namespace
{
    // Detour's random callback for a deck place: numbers in [0, 1)
    float VesselRandom()
    {
        return static_cast<float>(rand_norm());
    }

    // the deck polygon nearest a point in Detour's order, by the search's two boxes (GetPolyByLocation)
    dtPolyRef FindVesselPoly(dtNavMeshQuery const* navMeshQuery, dtQueryFilterExt const& filter, float const* point, float* closestPoint)
    {
        float const nearExtents[VERTEX_SIZE] = { 3.0f, 5.0f, 3.0f };
        float const farExtents[VERTEX_SIZE] = { 3.0f, 50.0f, 3.0f };

        dtPolyRef polyRef = INVALID_POLYREF;
        if (dtStatusSucceed(navMeshQuery->findNearestPoly(point, nearExtents, &filter, &polyRef, closestPoint)) && polyRef != INVALID_POLYREF)
            return polyRef;

        polyRef = INVALID_POLYREF;
        if (dtStatusSucceed(navMeshQuery->findNearestPoly(point, farExtents, &filter, &polyRef, closestPoint)) && polyRef != INVALID_POLYREF)
            return polyRef;

        return INVALID_POLYREF;
    }
}

bool PathGenerator::FindVesselNearestPoint(dtNavMeshQuery const* navMeshQuery, dtQueryFilterExt const& filter, G3D::Vector3 const& offset, G3D::Vector3& point)
{
    if (!navMeshQuery)
        return false;

    float const askedPoint[VERTEX_SIZE] = { offset.y, offset.z, offset.x };
    float closest[VERTEX_SIZE] = { 0.0f, 0.0f, 0.0f };
    if (FindVesselPoly(navMeshQuery, filter, askedPoint, closest) == INVALID_POLYREF)
        return false;

    point = G3D::Vector3(closest[2], closest[0], closest[1]);
    return true;
}

bool PathGenerator::FindVesselRandomPoint(dtNavMeshQuery const* navMeshQuery, dtQueryFilterExt const& filter, G3D::Vector3 const& offset, float radius,
    G3D::Vector3& point, float (*random)())
{
    if (!navMeshQuery)
        return false;

    float const centre[VERTEX_SIZE] = { offset.y, offset.z, offset.x };
    float closest[VERTEX_SIZE] = { 0.0f, 0.0f, 0.0f };
    dtPolyRef const startRef = FindVesselPoly(navMeshQuery, filter, centre, closest);
    if (startRef == INVALID_POLYREF)
        return false;

    dtPolyRef randomRef = INVALID_POLYREF;
    float randomPoint[VERTEX_SIZE] = { 0.0f, 0.0f, 0.0f };
    if (dtStatusFailed(navMeshQuery->findRandomPointAroundCircle(startRef, closest, radius, &filter, random ? random : VesselRandom, &randomRef, randomPoint)) ||
        randomRef == INVALID_POLYREF)
        return false;

    point = G3D::Vector3(randomPoint[2], randomPoint[0], randomPoint[1]);
    return true;
}

// The search on a vessel's mesh, in offsets. Where the world's search (BuildPolyPath, BuildPointPath) looks at the
// world or draws a straight line, this one answers as follows:
// - An end is on the deck when the world's two search boxes (3 x 5 x 3 yd, then 3 x 50 x 3 yd) find a deck polygon
//   and it is no farther than the 7 yd the world calls near. Otherwise there is no path (rule 2), and the same for
//   the start. The world gives a player or a flyer a straight line over a hole, and a far end an incomplete path
//   toward the closest polygon or, to a unit that swims, flies or falls, a straight line; none of these exists here,
//   and neither does a far-from-polygon flag.
// - An end on the deck but beside its polygon (up to 3 yd, the box: the margin the mesh keeps from a rail, the dock
//   beside a gangway) is moved onto the polygon: the path ends on the deck and stays PATHFIND_NORMAL, as the world's
//   does for an end as close to its mesh, without the world's last step off the polygon.
// - Heights are the deck's as the mesh gives them, since no ground lookup follows (the world's search lifts each
//   point by half a yard and then drops it onto the ground). The end keeps the height asked for when that is within
//   the mesh's own step height of the surface: the caller's foot is more exact than the mesh.
// - A deck can have surfaces no polygon joins (a castle, an envelope's top). An end on another surface than the
//   start gives PATHFIND_INCOMPLETE and a path to the closest polygon of the start's surface, as in the world.
// - No slope check: what is walkable aboard is the mesh's decision, and the check's water test asks the world.
// - No forced destination: the path's end is never replaced by the point asked for, nor the path by a straight line.
// - A path longer than the length limit is cut at the limit and is PATHFIND_INCOMPLETE | PATHFIND_SHORT, where the
//   world returns a straight line.
// - A smoothing that runs out of points (it goes back and forth at a sharp turn) gives the corridor's corners, where
//   the world answers no path.
// - With the ray cast (how far a straight line goes, Map::CheckCollisionAndGetValidCoords) the line runs on the
//   mesh from a start on the deck toward the end, on the deck or not, and stops where the deck ends: two points,
//   PATHFIND_NORMAL when it reaches an end on the deck, else PATHFIND_INCOMPLETE.
PathType PathGenerator::BuildVesselPath(G3D::Vector3 const& start, G3D::Vector3 const& end, Movement::PointsArray& points)
{
    float distToStartPoly, distToEndPoly;
    float startPoint[VERTEX_SIZE] = { start.y, start.z, start.x };
    float endPoint[VERTEX_SIZE] = { end.y, end.z, end.x };

    dtPolyRef const startPoly = GetPolyByLocation(startPoint, &distToStartPoly);
    dtPolyRef const endPoly = GetPolyByLocation(endPoint, &distToEndPoly);

    bool const startOnDeck = startPoly != INVALID_POLYREF && distToStartPoly <= 7.0f;
    bool const endOnDeck = endPoly != INVALID_POLYREF && distToEndPoly <= 7.0f;

    // rule 2
    if (!startOnDeck || (!endOnDeck && !_useRaycast))
        return PATHFIND_NOPATH;

    if (endOnDeck)
    {
        dtMeshTile const* tile = _navMesh->getTile(0);
        float const stepHeight = tile && tile->header ? tile->header->walkableClimb : 0.0f;

        float closestPoint[VERTEX_SIZE];
        bool overPoly = false;
        if (dtStatusSucceed(_navMeshQuery->closestPointOnPoly(endPoly, endPoint, closestPoint, &overPoly)) &&
            (!overPoly || std::fabs(endPoint[1] - closestPoint[1]) > stepHeight))
            dtVcopy(endPoint, closestPoint);
    }

    if (_useRaycast)
    {
        float hit = 0.0f;
        float hitNormal[VERTEX_SIZE] = { 0.0f, 0.0f, 0.0f };

        dtStatus const rayResult = _navMeshQuery->raycast(startPoly, startPoint, endPoint, &_filter, &hit, hitNormal, _pathPolyRefs, (int*)&_polyLength, MAX_PATH_LENGTH);
        if (!_polyLength || dtStatusFailed(rayResult))
            return PATHFIND_NOPATH;

        // raycast() sets hit to FLT_MAX if the ray reaches the end; else walk back a bit from the hit, as the
        // world's search does, to stay inside the polygon
        bool const stopped = hit != FLT_MAX;
        float lastPoint[VERTEX_SIZE];
        if (stopped)
            dtVlerp(lastPoint, startPoint, endPoint, hit * 0.99f);
        else
            dtVcopy(lastPoint, endPoint);

        dtPolyRef const lastPoly = _pathPolyRefs[_polyLength - 1];
        if ((stopped || !endOnDeck) && dtStatusFailed(_navMeshQuery->getPolyHeight(lastPoly, lastPoint, &lastPoint[1])))
            _navMeshQuery->closestPointOnPolyBoundary(lastPoly, lastPoint, lastPoint);

        points.resize(2);
        points[0] = start;
        points[1] = G3D::Vector3(lastPoint[2], lastPoint[0], lastPoint[1]);
        return (stopped || !endOnDeck) ? PATHFIND_INCOMPLETE : PATHFIND_NORMAL;
    }

    dtStatus const pathResult = _navMeshQuery->findPath(startPoly, endPoly, startPoint, endPoint, &_filter, _pathPolyRefs, (int*)&_polyLength, MAX_PATH_LENGTH);
    if (!_polyLength || dtStatusFailed(pathResult))
        return PATHFIND_NOPATH;

    PathType type = _pathPolyRefs[_polyLength - 1] == endPoly ? PATHFIND_NORMAL : PATHFIND_INCOMPLETE;

    float pathPoints[MAX_POINT_PATH_LENGTH * VERTEX_SIZE];
    uint32 pointCount = 0;
    dtStatus pointResult = DT_FAILURE;
    if (_useStraightPath)
        pointResult = _navMeshQuery->findStraightPath(startPoint, endPoint, _pathPolyRefs, _polyLength, pathPoints, nullptr, nullptr, (int*)&pointCount, _pointPathLimit);
    else
        pointResult = FindSmoothPath(startPoint, endPoint, _pathPolyRefs, _polyLength, pathPoints, (int*)&pointCount, _pointPathLimit);

    // The smoothing steps 4 yd toward the next corner of the corridor, past the corner when it is near, sliding along
    // the walls. At a turn onto a deck's narrow stair or gangway that step can end in a polygon the corridor has just
    // left; the corridor's fix-up turns it round, the next step overshoots again, and the smoothing goes back and
    // forth until it runs out of points, which it reports as a failure. The world gives no path there; aboard, where
    // the corridor is whole, the corridor's corners are the path.
    if (!_useStraightPath && dtStatusFailed(pointResult) && pointCount >= MAX_POINT_PATH_LENGTH)
    {
        pointCount = 0;
        pointResult = _navMeshQuery->findStraightPath(startPoint, endPoint, _pathPolyRefs, _polyLength, pathPoints, nullptr, nullptr, (int*)&pointCount, _pointPathLimit);
    }

    // Start and end very close to each other: the first point is the start, append the end. The smoothing stops at
    // the start when every corner and the end lie within its slop; on a deck's small polygons two such ends often
    // lie on two polygons, where the world's test (a single polygon) would answer no path for a move of a step.
    if (pointCount == 1 && (_polyLength == 1 || (dtStatusSucceed(pointResult) && dtVdist2D(startPoint, endPoint) <= SMOOTH_PATH_SLOP)))
    {
        dtVcopy(&pathPoints[1 * VERTEX_SIZE], endPoint);
        pointCount++;
    }
    else if (pointCount < 2 || dtStatusFailed(pointResult))
        return PATHFIND_NOPATH;
    else if (pointCount >= _pointPathLimit)
        type = PathType(PATHFIND_INCOMPLETE | PATHFIND_SHORT);

    points.resize(pointCount);
    for (uint32 i = 0; i < pointCount; ++i)
        points[i] = G3D::Vector3(pathPoints[i * VERTEX_SIZE + 2], pathPoints[i * VERTEX_SIZE], pathPoints[i * VERTEX_SIZE + 1]);

    return type;
}

dtPolyRef PathGenerator::GetPathPolyByPosition(dtPolyRef const* polyPath, uint32 polyPathSize, float const* point, float* distance) const
{
    if (!polyPath || !polyPathSize)
        return INVALID_POLYREF;

    dtPolyRef nearestPoly = INVALID_POLYREF;
    float minDist = FLT_MAX;

    for (uint32 i = 0; i < polyPathSize; ++i)
    {
        float closestPoint[VERTEX_SIZE];
        if (dtStatusFailed(_navMeshQuery->closestPointOnPoly(polyPath[i], point, closestPoint, nullptr)))
            continue;

        float d = dtVdistSqr(point, closestPoint);
        if (d < minDist)
        {
            minDist = d;
            nearestPoly = polyPath[i];
        }

        if (minDist < 1.0f) // shortcut out - close enough for us
        {
            break;
        }
    }

    if (distance)
    {
        *distance = dtMathSqrtf(minDist);
    }

    return (minDist < 3.0f) ? nearestPoly : INVALID_POLYREF;
}

dtPolyRef PathGenerator::GetPolyByLocation(float const* point, float* distance) const
{
    // first we check the current path
    // if the current path doesn't contain the current poly,
    // we need to use the expensive navMesh.findNearestPoly
    dtPolyRef polyRef = GetPathPolyByPosition(_pathPolyRefs, _polyLength, point, distance);
    if (polyRef != INVALID_POLYREF)
        return polyRef;

    // we don't have it in our old path
    // try to get it by findNearestPoly()
    // first try with low search box
    float extents[VERTEX_SIZE] = { 3.0f, 5.0f, 3.0f };    // bounds of poly search area
    float closestPoint[VERTEX_SIZE] = { 0.0f, 0.0f, 0.0f };
    if (dtStatusSucceed(_navMeshQuery->findNearestPoly(point, extents, &_filter, &polyRef, closestPoint)) && polyRef != INVALID_POLYREF)
    {
        *distance = dtVdist(closestPoint, point);
        return polyRef;
    }

    // still nothing ..
    // try with bigger search box
    // Note that the extent should not overlap more than 128 polygons in the navmesh (see dtNavMeshQuery::findNearestPoly)
    extents[1] = 50.0f;

    if (dtStatusSucceed(_navMeshQuery->findNearestPoly(point, extents, &_filter, &polyRef, closestPoint)) && polyRef != INVALID_POLYREF)
    {
        *distance = dtVdist(closestPoint, point);
        return polyRef;
    }

    *distance = FLT_MAX;
    return INVALID_POLYREF;
}

void PathGenerator::BuildPolyPath(G3D::Vector3 const& startPos, G3D::Vector3 const& endPos)
{
    // *** getting start/end poly logic ***

    float distToStartPoly, distToEndPoly;
    float startPoint[VERTEX_SIZE] = { startPos.y, startPos.z, startPos.x };
    float endPoint[VERTEX_SIZE] = { endPos.y, endPos.z, endPos.x };

    dtPolyRef startPoly = GetPolyByLocation(startPoint, &distToStartPoly);
    dtPolyRef endPoly = GetPolyByLocation(endPoint, &distToEndPoly);

    _type = PathType(PATHFIND_NORMAL);

    Creature const* creature = _source->ToCreature();

    // we have a hole in our mesh
    // make shortcut path and mark it as NOPATH ( with flying and swimming exception )
    // its up to caller how he will use this info
    if (startPoly == INVALID_POLYREF || endPoly == INVALID_POLYREF)
    {
        BuildShortcut();

        bool canSwim = creature ? creature->CanSwim() : true;
        bool path = creature ? creature->CanFly() : true;
        bool waterPath = IsWaterPath(_pathPoints);
        if (path || (waterPath && canSwim))
        {
            _type = PathType(PATHFIND_NORMAL | PATHFIND_NOT_USING_PATH);
            return;
        }

        // raycast doesn't need endPoly to be valid
        if (!_useRaycast)
        {
            _type = PATHFIND_NOPATH;
            return;
        }
    }

    // we may need a better number here
    bool startFarFromPoly = distToStartPoly > 7.0f;
    bool endFarFromPoly = distToEndPoly > 7.0f;

    // create a shortcut if the path begins or end too far
    // away from the desired path points.
    // swimming creatures should not use a shortcut
    // because exiting the water must be done following a proper path
    // we just need to remove/normalize paths between 2 adjacent points
    if (startFarFromPoly || endFarFromPoly)
    {
        bool buildShortcut = false;

        auto liquidDataStart = _source->GetMap()->GetLiquidData(_source->GetPhaseMask(), startPos.x, startPos.y, startPos.z, _source->GetCollisionHeight(), {});
        auto liquidDataEnd = _source->GetMap()->GetLiquidData(_source->GetPhaseMask(), endPos.x, endPos.y, endPos.z, _source->GetCollisionHeight(), {});

        bool startUnderWaterEndInWater = liquidDataStart.Status == LIQUID_MAP_UNDER_WATER &&
                                         (liquidDataEnd.Status & MAP_LIQUID_STATUS_IN_CONTACT) != 0;
        bool startInWaterEndUnderWater = (liquidDataStart.Status & MAP_LIQUID_STATUS_IN_CONTACT) != 0 &&
                                         liquidDataEnd.Status == LIQUID_MAP_UNDER_WATER;
        bool waterPath = startUnderWaterEndInWater || startInWaterEndUnderWater;
        Unit const* _sourceUnit = _source->ToUnit();

        if (_sourceUnit)
        {
            bool isWater = (_sourceUnit->CanSwim() && waterPath);

            if (isWater || _sourceUnit->CanFly() || (_sourceUnit->IsFalling() && endPos.z < startPos.z))
            {
                buildShortcut = true;
            }
        }

        if (buildShortcut)
        {
            BuildShortcut();
            _type = PathType(PATHFIND_NORMAL | PATHFIND_NOT_USING_PATH);

            AddFarFromPolyFlags(startFarFromPoly, endFarFromPoly);

            return;
        }
        else
        {
            float closestPoint[VERTEX_SIZE];
            // we may want to use closestPointOnPolyBoundary instead
            if (dtStatusSucceed(_navMeshQuery->closestPointOnPoly(endPoly, endPoint, closestPoint, nullptr)))
            {
                dtVcopy(endPoint, closestPoint);
                SetActualEndPosition(G3D::Vector3(endPoint[2], endPoint[0], endPoint[1]));
            }

            _type = PathType(PATHFIND_INCOMPLETE);

            AddFarFromPolyFlags(startFarFromPoly, endFarFromPoly);
        }
    }

    // *** poly path generating logic ***

    // start and end are on same polygon
    // handle this case as if they were 2 different polygons, building a line path split in some few points
    if (startPoly == endPoly && !_useRaycast)
    {
        _pathPolyRefs[0] = startPoly;
        _polyLength = 1;

        if (startFarFromPoly || endFarFromPoly)
        {
            _type = PathType(PATHFIND_INCOMPLETE);

            AddFarFromPolyFlags(startFarFromPoly, endFarFromPoly);
        }
        else
            _type = PATHFIND_NORMAL;

        BuildPointPath(startPoint, endPoint);
        return;
    }

    // look for startPoly/endPoly in current path
    /// @todo we can merge it with getPathPolyByPosition() loop
    bool startPolyFound = false;
    bool endPolyFound = false;
    uint32 pathStartIndex = 0;
    uint32 pathEndIndex = 0;

    if (_polyLength)
    {
        for (; pathStartIndex < _polyLength; ++pathStartIndex)
        {
            // here to catch few bugs
            if (_pathPolyRefs[pathStartIndex] == INVALID_POLYREF)
            {
                break;
            }

            if (_pathPolyRefs[pathStartIndex] == startPoly)
            {
                startPolyFound = true;
                break;
            }
        }

        for (pathEndIndex = _polyLength - 1; pathEndIndex > pathStartIndex; --pathEndIndex)
        {
            if (_pathPolyRefs[pathEndIndex] == endPoly)
            {
                endPolyFound = true;
                break;
            }
        }
    }

    if (startPolyFound && endPolyFound)
    {
        // we moved along the path and the target did not move out of our old poly-path
        // our path is a simple subpath case, we have all the data we need
        // just "cut" it out

        _polyLength = pathEndIndex - pathStartIndex + 1;
        memmove(_pathPolyRefs, _pathPolyRefs + pathStartIndex, _polyLength * sizeof(dtPolyRef));
    }
    else if (startPolyFound && !endPolyFound)
    {
        // we are moving on the old path but target moved out
        // so we have atleast part of poly-path ready

        _polyLength -= pathStartIndex;

        // try to adjust the suffix of the path instead of recalculating entire length
        // at given interval the target cannot get too far from its last location
        // thus we have less poly to cover
        // sub-path of optimal path is optimal

    // take ~80% of the original length
        /// @todo play with the values here
        uint32 prefixPolyLength = uint32(_polyLength * 0.8f + 0.5f);
        memmove(_pathPolyRefs, _pathPolyRefs + pathStartIndex, prefixPolyLength * sizeof(dtPolyRef));

        dtPolyRef suffixStartPoly = _pathPolyRefs[prefixPolyLength - 1];

        // we need any point on our suffix start poly to generate poly-path, so we need last poly in prefix data
        float suffixEndPoint[VERTEX_SIZE];
        if (dtStatusFailed(_navMeshQuery->closestPointOnPoly(suffixStartPoly, endPoint, suffixEndPoint, nullptr)))
        {
            // we can hit offmesh connection as last poly - closestPointOnPoly() don't like that
            // try to recover by using prev polyref
            --prefixPolyLength;
            suffixStartPoly = _pathPolyRefs[prefixPolyLength - 1];
            if (dtStatusFailed(_navMeshQuery->closestPointOnPoly(suffixStartPoly, endPoint, suffixEndPoint, nullptr)))
            {
                // suffixStartPoly is still invalid, error state
                BuildShortcut();
                _type = PATHFIND_NOPATH;
                return;
            }
        }

        // generate suffix
        uint32 suffixPolyLength = 0;

        dtStatus dtResult;
        if (_useRaycast)
        {
            BuildShortcut();
            _type = PATHFIND_NOPATH;
            return;
        }
        else
        {
            dtResult = _navMeshQuery->findPath(
                suffixStartPoly,    // start polygon
                endPoly,            // end polygon
                suffixEndPoint,     // start position
                endPoint,           // end position
                &_filter,            // polygon search filter
                _pathPolyRefs + prefixPolyLength - 1,    // [out] path
                (int*)&suffixPolyLength,
                MAX_PATH_LENGTH - prefixPolyLength); // max number of polygons in output path
        }

        if (!suffixPolyLength || dtStatusFailed(dtResult))
        {
            // this is probably an error state, but we'll leave it
            // and hopefully recover on the next Update
            // we still need to copy our preffix
            LOG_ERROR("movement", "PathGenerator::BuildPolyPath: Path Build failed {}", _source->GetGUID().ToString());
        }

        // new path = prefix + suffix - overlap
        _polyLength = prefixPolyLength + suffixPolyLength - 1;
    }
    else
    {
        // either we have no path at all -> first run
        // or something went really wrong -> we aren't moving along the path to the target
        // just generate new path

        // free and invalidate old path data
        Clear();

        dtStatus dtResult;
        if (_useRaycast)
        {
            float hit = 0;
            float hitNormal[3];
            memset(hitNormal, 0, sizeof(hitNormal));

            dtResult = _navMeshQuery->raycast(
                startPoly,
                startPoint,
                endPoint,
                &_filter,
                &hit,
                hitNormal,
                _pathPolyRefs,
                (int*)&_polyLength,
                MAX_PATH_LENGTH);

            if (!_polyLength || dtStatusFailed(dtResult))
            {
                BuildShortcut();
                _type = PATHFIND_NOPATH;
                AddFarFromPolyFlags(startFarFromPoly, endFarFromPoly);
                return;
            }

            // raycast() sets hit to FLT_MAX if there is a ray between start and end
            if (hit != FLT_MAX)
            {
                float hitPos[3];

                // Walk back a bit from the hit point to make sure it's in the mesh (sometimes the point is actually outside of the polygons due to float precision issues)
                hit *= 0.99f;
                dtVlerp(hitPos, startPoint, endPoint, hit);

                // if it fails again, clamp to poly boundary
                if (dtStatusFailed(_navMeshQuery->getPolyHeight(_pathPolyRefs[_polyLength - 1], hitPos, &hitPos[1])))
                    _navMeshQuery->closestPointOnPolyBoundary(_pathPolyRefs[_polyLength - 1], hitPos, hitPos);

                _pathPoints.resize(2);
                _pathPoints[0] = GetStartPosition();
                _pathPoints[1] = G3D::Vector3(hitPos[2], hitPos[0], hitPos[1]);

                NormalizePath();
                _type = PATHFIND_INCOMPLETE;
                AddFarFromPolyFlags(startFarFromPoly, false);
                return;
            }
            else
            {
                // clamp to poly boundary if we fail to get the height
                if (dtStatusFailed(_navMeshQuery->getPolyHeight(_pathPolyRefs[_polyLength - 1], endPoint, &endPoint[1])))
                    _navMeshQuery->closestPointOnPolyBoundary(_pathPolyRefs[_polyLength - 1], endPoint, endPoint);

                _pathPoints.resize(2);
                _pathPoints[0] = GetStartPosition();
                _pathPoints[1] = G3D::Vector3(endPoint[2], endPoint[0], endPoint[1]);

                NormalizePath();
                if (startFarFromPoly || endFarFromPoly)
                {
                    _type = PathType(PATHFIND_INCOMPLETE);

                    AddFarFromPolyFlags(startFarFromPoly, endFarFromPoly);
                }
                else
                    _type = PATHFIND_NORMAL;
                return;
            }
        }
        else
        {
            dtResult = _navMeshQuery->findPath(
                startPoly,          // start polygon
                endPoly,            // end polygon
                startPoint,         // start position
                endPoint,           // end position
                &_filter,           // polygon search filter
                _pathPolyRefs,     // [out] path
                (int*)&_polyLength,
                MAX_PATH_LENGTH);   // max number of polygons in output path
        }

        if (!_polyLength || dtStatusFailed(dtResult))
        {
            // only happens if we passed bad data to findPath(), or navmesh is messed up
            LOG_ERROR("movement", "PathGenerator::BuildPolyPath: {} Path Build failed: 0 length path", _source->GetGUID().ToString());
            BuildShortcut();
            _type = PATHFIND_NOPATH;
            return;
        }
    }

    if (!_polyLength)
    {
        LOG_ERROR("movement", "PathGenerator::BuildPolyPath: {} Path Build failed: 0 length path", _source->GetGUID().ToString());
        BuildShortcut();
        _type = PATHFIND_NOPATH;
        return;
    }

    // by now we know what type of path we can get
    if (_pathPolyRefs[_polyLength - 1] == endPoly && !(_type & PATHFIND_INCOMPLETE))
    {
        _type = PATHFIND_NORMAL;
    }
    else
    {
        _type = PATHFIND_INCOMPLETE;
    }

    AddFarFromPolyFlags(startFarFromPoly, endFarFromPoly);

    // generate the point-path out of our up-to-date poly-path
    BuildPointPath(startPoint, endPoint);
}

void PathGenerator::BuildPointPath(float const* startPoint, float const* endPoint)
{
    float pathPoints[MAX_POINT_PATH_LENGTH * VERTEX_SIZE];
    uint32 pointCount = 0;
    dtStatus dtResult = DT_FAILURE;
    if (_useRaycast)
    {
        // _straightLine uses raycast and it currently doesn't support building a point path, only a 2-point path with start and hitpoint/end is returned
        LOG_ERROR("movement", "PathGenerator::BuildPointPath() called with _useRaycast for unit {}", _source->GetGUID().ToString());
        BuildShortcut();
        _type = PATHFIND_NOPATH;
        return;
    }
    else if (_useStraightPath)
    {
        dtResult = _navMeshQuery->findStraightPath(
            startPoint,         // start position
            endPoint,           // end position
            _pathPolyRefs,     // current path
            _polyLength,       // lenth of current path
            pathPoints,         // [out] path corner points
            nullptr,               // [out] flags
            nullptr,               // [out] shortened path
            (int*)&pointCount,
            _pointPathLimit);   // maximum number of points/polygons to use
    }
    else
    {
        dtResult = FindSmoothPath(
            startPoint,         // start position
            endPoint,           // end position
            _pathPolyRefs,     // current path
            _polyLength,       // length of current path
            pathPoints,         // [out] path corner points
            (int*)&pointCount,
            _pointPathLimit);    // maximum number of points
    }

    // Special case with start and end positions very close to each other
    if (_polyLength == 1 && pointCount == 1 && !(dtResult & DT_SLOPE_TOO_STEEP))
    {
        // First point is start position, append end position
        dtVcopy(&pathPoints[1 * VERTEX_SIZE], endPoint);
        pointCount++;
    }
    else if (pointCount < 2 || dtStatusFailed(dtResult))
    {
        // If its too steep, just return incomplete path.
        if (pointCount > 0 && dtResult & DT_SLOPE_TOO_STEEP)
        {
            _pathPoints.resize(pointCount);
            for (uint32 i = 0; i < pointCount; ++i)
                _pathPoints[i] = G3D::Vector3(pathPoints[i * VERTEX_SIZE + 2], pathPoints[i * VERTEX_SIZE], pathPoints[i * VERTEX_SIZE + 1]);

            NormalizePath();

            // first point is always our current location - we need the next one
            SetActualEndPosition(_pathPoints[pointCount - 1]);

            _type = PathType(_type | PATHFIND_INCOMPLETE);
            return;
        }

        // only happens if pass bad data to findStraightPath or navmesh is broken
        // single point paths can be generated here
        /// @todo check the exact cases
        BuildShortcut();
        _type = PathType(_type | PATHFIND_NOPATH);
        return;
    }
    else if (pointCount >= _pointPathLimit)
    {
        BuildShortcut();
        _type = PathType(_type | PATHFIND_SHORT);
        return;
    }

    _pathPoints.resize(pointCount);
    for (uint32 i = 0; i < pointCount; ++i)
        _pathPoints[i] = G3D::Vector3(pathPoints[i * VERTEX_SIZE + 2], pathPoints[i * VERTEX_SIZE], pathPoints[i * VERTEX_SIZE + 1]);

    NormalizePath();

    // first point is always our current location - we need the next one
    SetActualEndPosition(_pathPoints[pointCount - 1]);

    // force the given destination, if needed
    if (_forceDestination &&
        (!(_type & PATHFIND_NORMAL) || !InRange(GetEndPosition(), GetActualEndPosition(), 1.0f, 1.0f)))
    {
        // we may want to keep partial subpath
        if (Dist3DSqr(GetActualEndPosition(), GetEndPosition()) < 0.3f * Dist3DSqr(GetStartPosition(), GetEndPosition()))
        {
            SetActualEndPosition(GetEndPosition());
            _pathPoints[_pathPoints.size() - 1] = GetEndPosition();
        }
        else
        {
            SetActualEndPosition(GetEndPosition());
            BuildShortcut();
        }

        _type = PathType(PATHFIND_NORMAL | PATHFIND_NOT_USING_PATH);
    }
}

void PathGenerator::NormalizePath()
{
    bool const snapBladeEdgeArenaRopes = _source->GetMapId() == MAP_BLADES_EDGE_ARENA;
    for (G3D::Vector3& point : _pathPoints)
    {
        if (snapBladeEdgeArenaRopes && TrySnapToBladeEdgeArenaRope(point))
            continue;

        _source->UpdateAllowedPositionZ(point.x, point.y, point.z);
    }
}

void PathGenerator::BuildShortcut()
{
    Clear();

    // make two point path, our curr pos is the start, and dest is the end
    _pathPoints.resize(2);

    // set start and a default next position
    _pathPoints[0] = GetStartPosition();
    _pathPoints[1] = GetActualEndPosition();

    NormalizePath();

    _type = PATHFIND_SHORTCUT;
}

void PathGenerator::CreateFilter()
{
    uint16 includeFlags = 0;
    uint16 excludeFlags = 0;
    bool isHeadless = false;

    if (_source->IsCreature())
    {
        Creature* creature = (Creature*)_source;
        if (creature->CanWalk())
            includeFlags |= (NAV_GROUND | NAV_GROUND_STEEP);

        // creatures don't take environmental damage
        if (creature->CanEnterWater())
            includeFlags |= (NAV_WATER | NAV_MAGMA);
    }
    else // assume Player
    {
        // Bots navigate with a stricter filter: include ground + water but exclude lava/slime and
        // NAV_GROUND_STEEP (the 50-60deg slopes the extractor tags via modAlmostUnwalkableTriangles), so
        // they keep off steep mountainsides and follow gentle ground/roads. Real players are unchanged and
        // may still path across steep terrain.
        Player const* player = _source->ToPlayer();
        if (player && player->GetSession() && player->GetSession()->IsHeadless())
        {
            includeFlags |= (NAV_GROUND | NAV_WATER);
            excludeFlags |= (NAV_MAGMA | NAV_SLIME | NAV_GROUND_STEEP);
            isHeadless = true;
        }
        else
        {
            // perfect support not possible, just stay 'safe'
            includeFlags |= (NAV_GROUND | NAV_GROUND_STEEP | NAV_WATER | NAV_MAGMA);
        }
    }

    _filter.setIncludeFlags(includeFlags);
    _filter.setExcludeFlags(excludeFlags);

    // Bots bias their routes away from deep water (swim only when necessary). poly.area == poly.flags ==
    // NavTerrain, so NAV_WATER doubles as the water area index. Real players and creatures assign no cost.
    if (isHeadless)
        _filter.setAreaCost(NAV_WATER, 5.0f);

    UpdateFilter();
}

void PathGenerator::UpdateFilter()
{
    // allow creatures to cheat and use different movement types if they are moved
    // forcefully into terrain they can't normally move in
    if (Unit const* _sourceUnit = _source->ToUnit())
    {
        if (_sourceUnit->IsInWater() || _sourceUnit->IsUnderWater())
        {
            uint16 includedFlags = _filter.getIncludeFlags();
            includedFlags |= GetNavTerrain(_source->GetPositionX(),
                _source->GetPositionY(),
                _source->GetPositionZ());

            _filter.setIncludeFlags(includedFlags);
        }

        /*if (Creature const* _sourceCreature = _source->ToCreature())
            if (_sourceCreature->IsInCombat() || _sourceCreature->IsInEvadeMode())
                _filter.setIncludeFlags(_filter.getIncludeFlags() | NAV_GROUND_STEEP);*/
    }
}

NavTerrain PathGenerator::GetNavTerrain(float x, float y, float z) const
{
    LiquidData const& liquidData = _source->GetMap()->GetLiquidData(_source->GetPhaseMask(), x, y, z, _source->GetCollisionHeight(), {});
    if (liquidData.Status == LIQUID_MAP_NO_WATER)
        return NAV_GROUND;

    switch (liquidData.Flags)
    {
        case MAP_LIQUID_TYPE_WATER:
        case MAP_LIQUID_TYPE_OCEAN:
            return NAV_WATER;
        case MAP_LIQUID_TYPE_MAGMA:
        case MAP_LIQUID_TYPE_SLIME:
            return NAV_MAGMA;
        default:
            return NAV_GROUND;
    }
}

bool PathGenerator::HaveTile(G3D::Vector3 const& p) const
{
    int tx = -1, ty = -1;
    float point[VERTEX_SIZE] = { p.y, p.z, p.x };

    _navMesh->calcTileLoc(point, &tx, &ty);

    /// Workaround
    /// For some reason, often the tx and ty variables wont get a valid value
    /// Use this check to prevent getting negative tile coords and crashing on getTileAt
    if (tx < 0 || ty < 0)
        return false;

    return (_navMesh->getTileAt(tx, ty, 0) != nullptr);
}

uint32 PathGenerator::FixupCorridor(dtPolyRef* path, uint32 npath, uint32 maxPath, dtPolyRef const* visited, uint32 nvisited)
{
    int32 furthestPath = -1;
    int32 furthestVisited = -1;

    // Find furthest common polygon.
    for (int32 i = npath - 1; i >= 0; --i)
    {
        bool found = false;
        for (int32 j = nvisited - 1; j >= 0; --j)
        {
            if (path[i] == visited[j])
            {
                furthestPath = i;
                furthestVisited = j;
                found = true;
            }
        }
        if (found)
            break;
    }

    // If no intersection found just return current path.
    if (furthestPath == -1 || furthestVisited == -1)
        return npath;

    // Concatenate paths.

    // Adjust beginning of the buffer to include the visited.
    uint32 req = nvisited - furthestVisited;
    uint32 orig = uint32(furthestPath + 1) < npath ? furthestPath + 1 : npath;
    uint32 size = npath > orig ? npath - orig : 0;
    if (req + size > maxPath)
        size = maxPath - req;

    if (size)
        memmove(path + req, path + orig, size * sizeof(dtPolyRef));

    // Store visited
    for (uint32 i = 0; i < req; ++i)
        path[i] = visited[(nvisited - 1) - i];

    return req + size;
}

bool PathGenerator::GetSteerTarget(float const* startPos, float const* endPos,
    float minTargetDist, dtPolyRef const* path, uint32 pathSize,
    float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef)
{
    // Find steer target.
    static const uint32 MAX_STEER_POINTS = 3;
    float steerPath[MAX_STEER_POINTS * VERTEX_SIZE];
    unsigned char steerPathFlags[MAX_STEER_POINTS];
    dtPolyRef steerPathPolys[MAX_STEER_POINTS];
    uint32 nsteerPath = 0;
    dtStatus dtResult = _navMeshQuery->findStraightPath(startPos, endPos, path, pathSize,
        steerPath, steerPathFlags, steerPathPolys, (int*)&nsteerPath, MAX_STEER_POINTS);
    if (!nsteerPath || dtStatusFailed(dtResult))
        return false;

    // Find vertex far enough to steer to.
    uint32 ns = 0;
    while (ns < nsteerPath)
    {
        // Stop at Off-Mesh link or when point is further than slop away.
        if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
            !InRangeYZX(&steerPath[ns * VERTEX_SIZE], startPos, minTargetDist, 1000.0f))
            break;

        ns++;
    }
    // Failed to find good point to steer to.
    if (ns >= nsteerPath)
        return false;

    dtVcopy(steerPos, &steerPath[ns * VERTEX_SIZE]);
    steerPos[1] = startPos[1];  // keep Z value
    steerPosFlag = steerPathFlags[ns];
    steerPosRef = steerPathPolys[ns];

    return true;
}

dtStatus PathGenerator::FindSmoothPath(float const* startPos, float const* endPos,
    dtPolyRef const* polyPath, uint32 polyPathSize,
    float* smoothPath, int* smoothPathSize, uint32 maxSmoothPathSize)
{
    *smoothPathSize = 0;
    uint32 nsmoothPath = 0;

    dtPolyRef polys[MAX_PATH_LENGTH];
    memcpy(polys, polyPath, sizeof(dtPolyRef) * polyPathSize);
    uint32 npolys = polyPathSize;

    float iterPos[VERTEX_SIZE], targetPos[VERTEX_SIZE];

    if (polyPathSize > 1)
    {
        // Pick the closest points on poly border
        if (dtStatusFailed(_navMeshQuery->closestPointOnPolyBoundary(polys[0], startPos, iterPos)))
        {
            return DT_FAILURE;
        }

        if (dtStatusFailed(_navMeshQuery->closestPointOnPolyBoundary(polys[npolys - 1], endPos, targetPos)))
        {
            return DT_FAILURE;
        }
    }
    else
    {
        // Case where the path is on the same poly
        dtVcopy(iterPos, startPos);
        dtVcopy(targetPos, endPos);
    }

    dtVcopy(&smoothPath[nsmoothPath * VERTEX_SIZE], iterPos);
    nsmoothPath++;

    // Move towards target a small advancement at a time until target reached or
    // when ran out of memory to store the path.
    while (npolys && nsmoothPath < maxSmoothPathSize)
    {
        // Find location to steer towards.
        float steerPos[VERTEX_SIZE];
        unsigned char steerPosFlag;
        dtPolyRef steerPosRef = INVALID_POLYREF;

        if (!GetSteerTarget(iterPos, targetPos, SMOOTH_PATH_SLOP, polys, npolys, steerPos, steerPosFlag, steerPosRef))
            break;

        bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) != 0;
        bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;

        // Find movement delta.
        float delta[VERTEX_SIZE];
        dtVsub(delta, steerPos, iterPos);
        float len = dtMathSqrtf(dtVdot(delta, delta));
        // If the steer target is end of path or off-mesh link, do not move past the location.
        if ((endOfPath || offMeshConnection) && len < SMOOTH_PATH_STEP_SIZE)
            len = 1.0f;
        else
            len = SMOOTH_PATH_STEP_SIZE / len;

        float moveTgt[VERTEX_SIZE];
        dtVmad(moveTgt, iterPos, delta, len);

        // Move
        float result[VERTEX_SIZE];
        const static uint32 MAX_VISIT_POLY = 16;
        dtPolyRef visited[MAX_VISIT_POLY];

        uint32 nvisited = 0;
        if (dtStatusFailed(_navMeshQuery->moveAlongSurface(polys[0], iterPos, moveTgt, &_filter, result, visited, (int*)&nvisited, MAX_VISIT_POLY)))
        {
            return DT_FAILURE;
        }
        npolys = FixupCorridor(polys, npolys, MAX_PATH_LENGTH, visited, nvisited);

        // A vessel search has no owner, and nothing drops its points onto the ground afterwards (NormalizePath asks
        // the world): a point keeps the deck's height as the mesh gives it.
        if (IsVesselSearch())
            _navMeshQuery->getPolyHeight(polys[0], result, &result[1]);
        else
        {
            if (dtStatusFailed(_navMeshQuery->getPolyHeight(polys[0], result, &result[1])))
                LOG_DEBUG("maps", "PathGenerator::FindSmoothPath: Cannot find height at position X: {} Y: {} Z: {} for {}",
                    result[2], result[0], result[1], _source->GetGUID().ToString());
            result[1] += 0.5f;
        }
        dtVcopy(iterPos, result);

        bool canCheckSlope = _slopeCheck && (GetPathType() & ~(PATHFIND_NOT_USING_PATH));

        if (canCheckSlope && !IsSwimmableSegment(iterPos, steerPos) && !IsWalkableClimb(iterPos, steerPos))
        {
            nsmoothPath--;
            *smoothPathSize = nsmoothPath;
            return DT_FAILURE | DT_SLOPE_TOO_STEEP;
        }

        // Handle end of path and off-mesh links when close enough.
        if (endOfPath && InRangeYZX(iterPos, steerPos, SMOOTH_PATH_SLOP, 1.0f))
        {
            // Reached end of path.
            dtVcopy(iterPos, targetPos);
            if (nsmoothPath < maxSmoothPathSize)
            {
                dtVcopy(&smoothPath[nsmoothPath * VERTEX_SIZE], iterPos);
                nsmoothPath++;
            }
            break;
        }
        else if (offMeshConnection && InRangeYZX(iterPos, steerPos, SMOOTH_PATH_SLOP, 1.0f))
        {
            // Advance the path up to and over the off-mesh connection.
            dtPolyRef prevRef = INVALID_POLYREF;
            dtPolyRef polyRef = polys[0];
            uint32 npos = 0;
            while (npos < npolys && polyRef != steerPosRef)
            {
                prevRef = polyRef;
                polyRef = polys[npos];
                npos++;
            }

            for (uint32 i = npos; i < npolys; ++i)
                polys[i - npos] = polys[i];

            npolys -= npos;

            // Handle the connection.
            float connectionStartPos[VERTEX_SIZE], connectionEndPos[VERTEX_SIZE];
            if (dtStatusSucceed(_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, connectionStartPos, connectionEndPos)))
            {
                if (nsmoothPath < maxSmoothPathSize)
                {
                    dtVcopy(&smoothPath[nsmoothPath * VERTEX_SIZE], connectionStartPos);
                    nsmoothPath++;
                }
                // Move position at the other side of the off-mesh link.
                dtVcopy(iterPos, connectionEndPos);
                if (dtStatusFailed(_navMeshQuery->getPolyHeight(polys[0], iterPos, &iterPos[1])))
                    return DT_FAILURE;
                if (!IsVesselSearch())
                    iterPos[1] += 0.5f;
            }
        }

        // Store results.
        if (nsmoothPath < maxSmoothPathSize)
        {
            dtVcopy(&smoothPath[nsmoothPath * VERTEX_SIZE], iterPos);
            nsmoothPath++;
        }
    }

    *smoothPathSize = nsmoothPath;

    // this is most likely a loop
    return nsmoothPath < MAX_POINT_PATH_LENGTH ? DT_SUCCESS : DT_FAILURE;
}

bool PathGenerator::IsWalkableClimb(float const* v1, float const* v2) const
{
    return IsWalkableClimb(v1[2], v1[0], v1[1], v2[2], v2[0], v2[1]);
}

bool PathGenerator::IsWalkableClimb(float x, float y, float z, float destX, float destY, float destZ) const
{
    return IsWalkableClimb(x, y, z, destX, destY, destZ, _source->GetCollisionHeight());
}

/**
 * @brief Check if a slope can be climbed based on source height
 * This method is meant for short distances or linear paths
 *
 * @param x start x coord
 * @param y start y coord
 * @param z start z coord
 * @param destX destination x coord
 * @param destY destination y coord
 * @param destZ destination z coord
 * @param sourceHeight height of the source
 * @return bool check if you can climb the path
 */
bool PathGenerator::IsWalkableClimb(float x, float y, float z, float destX, float destY, float destZ, float sourceHeight)
{
    float diffHeight = std::abs(destZ - z);
    float reqHeight = GetRequiredHeightToClimb(x, y, z, destX, destY, destZ, sourceHeight);
    // check walkable slopes, based on unit height
    return diffHeight <= reqHeight;
}

/**
 * @brief Return the height of a slope that can be climbed based on source height
 * This method is meant for short distances or linear paths
 *
 * @param x start x coord
 * @param y start y coord
 * @param z start z coord
 * @param destX destination x coord
 * @param destY destination y coord
 * @param destZ destination z coord
 * @param sourceHeight height of the source
 * @return float the maximum height that a source can climb based on slope angle
 */
float PathGenerator::GetRequiredHeightToClimb(float x, float y, float z, float destX, float destY, float destZ, float sourceHeight)
{
    float slopeAngle = getSlopeAngleAbs(x, y, z, destX, destY, destZ);
    float slopeAngleDegree = (slopeAngle * 180.0f / M_PI);
    float climbableHeight = sourceHeight - (sourceHeight * (slopeAngleDegree / 100));
    return climbableHeight;
}

bool PathGenerator::InRangeYZX(float const* v1, float const* v2, float r, float h) const
{
    const float dx = v2[0] - v1[0];
    const float dy = v2[1] - v1[1]; // elevation
    const float dz = v2[2] - v1[2];
    return (dx * dx + dz * dz) < r * r && fabsf(dy) < h;
}

bool PathGenerator::InRange(G3D::Vector3 const& p1, G3D::Vector3 const& p2, float r, float h) const
{
    G3D::Vector3 d = p1 - p2;
    return (d.x * d.x + d.y * d.y) < r * r && fabsf(d.z) < h;
}

float PathGenerator::Dist3DSqr(G3D::Vector3 const& p1, G3D::Vector3 const& p2) const
{
    return (p1 - p2).squaredLength();
}

void PathGenerator::ShortenPathUntilDist(G3D::Vector3 const& target, float dist)
{
    if (GetPathType() == PATHFIND_BLANK || _pathPoints.size() < 2)
    {
        LOG_ERROR("movement", "PathGenerator::ReducePathLengthByDist called before path was successfully built");
        return;
    }

    float const distSq = dist * dist;

    // the first point of the path must be outside the specified range
    // (this should have really been checked by the caller...)
    if ((_pathPoints[0] - target).squaredLength() < distSq)
        return;

    // check if we even need to do anything
    if ((*_pathPoints.rbegin() - target).squaredLength() >= distSq)
        return;

    std::size_t i = _pathPoints.size() - 1;
    float x, y, z, collisionHeight = _source->GetCollisionHeight();
    // find the first i s.t.:
    //  - _pathPoints[i] is still too close
    //  - _pathPoints[i-1] is too far away
    // => the end point is somewhere on the line between the two
    while (1)
    {
        // we know that pathPoints[i] is too close already (from the previous iteration)
        if ((_pathPoints[i - 1] - target).squaredLength() >= distSq)
            break; // bingo!

        bool canCheckSlope = _slopeCheck && (GetPathType() & ~(PATHFIND_NOT_USING_PATH));

        // check if the shortened path is still in LoS with the target and it is walkable
        _source->GetHitSpherePointFor({ _pathPoints[i - 1].x, _pathPoints[i - 1].y, _pathPoints[i - 1].z + collisionHeight }, x, y, z);
        if (!_source->GetMap()->isInLineOfSight(x, y, z, _pathPoints[i - 1].x, _pathPoints[i - 1].y, _pathPoints[i - 1].z + collisionHeight,
            _source->GetPhaseMask(), LINEOFSIGHT_ALL_CHECKS, VMAP::ModelIgnoreFlags::Nothing) || (canCheckSlope &&
                !IsSwimmableSegment(_source->GetPositionX(), _source->GetPositionY(), _source->GetPositionZ(), _pathPoints[i - 1].x, _pathPoints[i - 1].y, _pathPoints[i - 1].z) &&
                !IsWalkableClimb(_source->GetPositionX(), _source->GetPositionY(), _source->GetPositionZ(), _pathPoints[i - 1].x, _pathPoints[i - 1].y, _pathPoints[i - 1].z)))
        {
            // whenver we find a point that is not valid anymore, simply use last valid path
            _pathPoints.resize(i + 1);
            return;
        }

        if (!--i)
        {
            // no point found that fulfills the condition
            _pathPoints[0] = _pathPoints[1];
            _pathPoints.resize(2);
            return;
        }
    }

    // ok, _pathPoints[i] is too close, _pathPoints[i-1] is not, so our target point is somewhere between the two...
    //   ... settle for a guesstimate since i'm not confident in doing trig on every chase motion tick...
    // (@todo review this)
    _pathPoints[i] += (_pathPoints[i - 1] - _pathPoints[i]).direction() * (dist - (_pathPoints[i] - target).length());
    _pathPoints.resize(i + 1);
}

bool PathGenerator::IsInvalidDestinationZ(Unit const* target) const
{
    return (target->GetPositionZ() - GetActualEndPosition().z) > 5.0f;
}

void PathGenerator::AddFarFromPolyFlags(bool startFarFromPoly, bool endFarFromPoly)
{
    if (startFarFromPoly)
    {
        _type = PathType(_type | PATHFIND_FARFROMPOLY_START);
    }
    if (endFarFromPoly)
    {
        _type = PathType(_type | PATHFIND_FARFROMPOLY_END);
    }
}

/**
 * @brief predict if a certain segment is underwater and the unit can swim
 * Must only be used for very short segments since this check doesn't work on
 * long paths that alternate terrain and water.
 *
 * @param v1
 * @param v2
 * @return true
 * @return false
 */
bool PathGenerator::IsSwimmableSegment(float const* v1, float const* v2, bool checkSwim) const
{
    return IsSwimmableSegment(v1[2], v1[0], v1[1], v2[2], v2[0], v2[1], checkSwim);
}

/**
 * @brief predict if a certain segment is underwater and the unit can swim
 * Must only be used for very short segments since this check doesn't work on
 * long paths that alternate terrain and water.
 *
 * @param x
 * @param y
 * @param z
 * @param destX
 * @param destY
 * @param destZ
 * @param checkSwim also check if the unit can swim
 * @return true if there's water at the end AND at the start of the segment
 * @return false if there's no water at the end OR at the start of the segment
 */
bool PathGenerator::IsSwimmableSegment(float x, float y, float z, float destX, float destY, float destZ, bool checkSwim) const
{
    Creature const* _sourceCreature = _source->ToCreature();
    return _source->GetMap()->IsInWater(_source->GetPhaseMask(), x, y, z, _source->GetCollisionHeight()) &&
        _source->GetMap()->IsInWater(_source->GetPhaseMask(), destX, destY, destZ, _source->GetCollisionHeight()) &&
        (!checkSwim || !_sourceCreature || _sourceCreature->CanSwim());
}

bool PathGenerator::IsWaterPath(Movement::PointsArray pathPoints) const
{
    bool waterPath = true;
    // Check both start and end points, if they're both in water, then we can *safely* let the creature move
    for (uint32 i = 0; i < pathPoints.size(); ++i)
    {
        NavTerrain terrain = GetNavTerrain(pathPoints[i].x, pathPoints[i].y, pathPoints[i].z);
        // One of the points is not in the water
        if (terrain != NAV_MAGMA && terrain != NAV_WATER)
        {
            waterPath = false;
            break;
        }
    }

    return waterPath;
}
