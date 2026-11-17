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

#ifndef _VESSEL_FRAME_H
#define _VESSEL_FRAME_H

#include <cmath>

// The turn between world coordinates and a vessel's own (a passenger's offset), for the path finder's vessel mode.
// No game include on purpose: the turn is the mode's invariant and is proven alone, outside the server
// (mod-playerbots, tests/vessel-turn-selftest.cpp). The formulas are those of
// TransportBase::CalculatePassengerPosition and CalculatePassengerOffset (VehicleDefines.h), term for term, so a point
// turned here and one turned by the spline (TransportPathTransform) agree to the last bit.
namespace VesselFrame
{
    // where the vessel is: its position and its heading (radians)
    struct Pose
    {
        float x;
        float y;
        float z;
        float o;
    };

    // an offset on the vessel -> the world point it is at
    inline void ToWorld(Pose const& pose, float& x, float& y, float& z)
    {
        float const inx = x, iny = y, inz = z;

        x = pose.x + inx * std::cos(pose.o) - iny * std::sin(pose.o);
        y = pose.y + iny * std::cos(pose.o) + inx * std::sin(pose.o);
        z = pose.z + inz;
    }

    // a world point -> its offset on the vessel
    inline void ToOffset(Pose const& pose, float& x, float& y, float& z)
    {
        z -= pose.z;
        y -= pose.y;
        x -= pose.x;
        float const inx = x, iny = y;
        y = (iny - inx * std::tan(pose.o)) / (std::cos(pose.o) + std::sin(pose.o) * std::tan(pose.o));
        x = (inx + iny * std::tan(pose.o)) / (std::cos(pose.o) + std::sin(pose.o) * std::tan(pose.o));
    }
}

#endif
