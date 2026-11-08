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

#ifndef WORLDDAY_H
#define WORLDDAY_H

#include <algorithm>
#include <cmath>
#include <cstdint>

// The world day (World.DayHours): every periodic reset counts world days of running time. A world day of 24 hours
// keeps the calendar schedule (reset hours, real months); any other length schedules each reset one period after
// the previous one, with no hour of day. Pure maths only, so a standalone harness can include this header.
namespace Acore::WorldDay
{
    constexpr std::int64_t RealDaySeconds = 86400;
    constexpr float MinDayHours = 0.05f;
    constexpr float MaxDayHours = 720.0f;

    constexpr std::uint32_t DailyDays = 1;
    constexpr std::uint32_t WeeklyDays = 7;
    constexpr std::uint32_t MonthlyDays = 30;

    // The arena points checker runs every 10 minutes on a real day, and ten times per period on a shorter one.
    constexpr std::uint32_t RealCheckIntervalMs = 600000;
    constexpr std::uint32_t MinCheckIntervalMs = 10000;

    inline bool IsValidDayHours(float dayHours)
    {
        return dayHours >= MinDayHours && dayHours <= MaxDayHours;
    }

    // The world day in whole seconds; 24 hours give exactly 86400.
    inline std::int64_t DaySeconds(float dayHours)
    {
        return std::int64_t(std::llround(double(dayHours) * 3600.0));
    }

    inline bool IsRealDay(std::int64_t daySeconds)
    {
        return daySeconds == RealDaySeconds;
    }

    inline std::int64_t PeriodSeconds(std::int64_t daySeconds, std::uint32_t worldDays)
    {
        return daySeconds * std::int64_t(worldDays);
    }

    // The first reset when nothing is stored: one period from now.
    inline std::int64_t FirstNext(std::int64_t now, std::int64_t period)
    {
        return now + period;
    }

    // The reset after the one that just fired: the first time on the grid previousNext + k * period (k >= 1) later
    // than now. The schedule does not drift, and a catch-up after missed periods fires once, not once per period.
    inline std::int64_t NextAfter(std::int64_t now, std::int64_t previousNext, std::int64_t period)
    {
        if (period <= 0)
            return now;

        if (previousNext <= 0)
            return FirstNext(now, period);

        std::int64_t const k = previousNext > now ? 1 : (now - previousNext) / period + 1;
        return previousNext + k * period;
    }

    // A stored next reset at load: nothing stored starts one period from now; a value further away than one
    // period (stored under a longer world day, e.g. the calendar schedule) is pulled in to one period from now;
    // anything else, also an overdue value, stays as stored.
    inline std::int64_t ClampNext(std::int64_t now, std::int64_t stored, std::int64_t period)
    {
        if (stored <= 0)
            return FirstNext(now, period);

        return std::min(stored, now + period);
    }

    inline std::uint32_t CheckIntervalMs(std::int64_t period)
    {
        std::int64_t const tenth = period * 100; // period / 10 in milliseconds
        return std::uint32_t(std::clamp<std::int64_t>(tenth, MinCheckIntervalMs, RealCheckIntervalMs));
    }

    // Instance lockouts. The real period is today's: the map's reset time times Rate.InstanceResetTime, at least one
    // day (the same float expression, so the value is identical).
    inline std::uint32_t RealInstancePeriod(std::uint32_t resetTime, float rate)
    {
        std::uint32_t period = std::uint32_t(((resetTime * rate) / RealDaySeconds) * RealDaySeconds);
        if (period < RealDaySeconds)
            period = std::uint32_t(RealDaySeconds);

        return period;
    }

    // The lockout period in world days: the real period scaled from real days to world days, at least one world day.
    inline std::uint32_t InstancePeriod(std::uint32_t resetTime, float rate, std::int64_t daySeconds)
    {
        std::uint32_t const real = RealInstancePeriod(resetTime, rate);
        if (IsRealDay(daySeconds))
            return real;

        std::int64_t const scaled = std::int64_t(real) * daySeconds / RealDaySeconds;
        return std::uint32_t(std::max(scaled, daySeconds));
    }

    // The reset after the one scheduled at resetTime. A real day aligns it to the reset hour (hourOffset seconds
    // after midnight) as today; a world day anchors it on the scheduled time.
    inline std::int64_t NextInstanceReset(std::int64_t resetTime, std::uint32_t period, std::uint32_t hourOffset,
        std::int64_t daySeconds)
    {
        if (IsRealDay(daySeconds))
        {
            std::int64_t const day = (resetTime + 60) / RealDaySeconds * RealDaySeconds;
            return std::int64_t(std::uint32_t(day + period + hourOffset));
        }

        return resetTime + period;
    }

    // The first warning (1 = an hour before ... 5 = the reset itself) whose time is still ahead, as LoadResetTimes
    // picks it: on a short world day the early warnings are already past and are skipped. delays holds five values
    // (InstanceSaveMgr::ResetTimeDelay).
    inline std::uint8_t FirstWarningType(std::int64_t now, std::int64_t resetAt, std::uint16_t const* delays)
    {
        std::uint8_t type;
        for (type = 1; type < 5; ++type)
            if (now + delays[type - 1] < resetAt)
                break;

        return type;
    }
}

#endif
