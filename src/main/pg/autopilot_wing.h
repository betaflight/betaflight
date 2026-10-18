/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef enum {
    WING_LOITER_RIGHT = 0,
    WING_LOITER_LEFT,
} wingLoiterDirection_e;

// +1 for a turn to the right, clockwise seen from above, -1 for one to the left.
static inline int8_t wingTurnSign(uint8_t direction)
{
    return (direction == WING_LOITER_LEFT) ? -1 : 1;
}

typedef struct autopilotWingConfig_s {
    uint8_t cruiseThrottle;         // percent, level flight at the cruise speed
    uint8_t minThrottle;            // percent, at the steepest dive
    uint8_t maxThrottle;            // percent, at the steepest climb
    uint8_t bankThrottle;           // percent added at 45 degrees of bank, scaling with tan^2 of the bank
    uint8_t maxClimbAngle;          // degrees nose-up from trim
    uint8_t maxDiveAngle;           // degrees nose-down from trim
    uint8_t maxClimbRate;           // dm/s
    uint8_t maxSinkRate;            // dm/s
    uint8_t altTimeConstant;        // ds, altitude error to climb rate
    uint8_t climbP;                 // 0.1 degree of pitch per m/s of climb rate error
    uint8_t climbI;                 // 0.1 degree of pitch per metre of accumulated climb rate error
    uint8_t vertAccel;              // dm/s^2, bounds how fast the pitch demand moves
    uint16_t cruiseSpeed;           // dm/s, the airspeed the pitch and turn geometry assume
    uint8_t turnPitchFf;            // percent of the pitch rate a level turn needs
    uint8_t maxBank;                // degrees, the steepest bank lateral guidance commands
    uint16_t l1Period;              // ds, the period lateral guidance settles onto a track with
    uint8_t l1Damping;              // 0.01, damping ratio of that settling
    uint16_t loiterRadius;          // m
    uint8_t loiterDirection;        // wingLoiterDirection_e
    int16_t landHeading;            // degrees, the heading landings are flown on, -1 to choose one
    uint8_t landSide;               // wingLoiterDirection_e, the way the approach pattern turns
    uint16_t landFinalLength;       // m
    uint8_t landGlideAngle;         // degrees
    uint8_t landApproachAlt;        // m above the touchdown, where the pattern is flown
    uint16_t landFlareHeight;       // cm
    uint8_t landFlarePitch;         // degrees nose-up from trim, the most the flare raises the nose to
    uint8_t landFlareSink;          // dm/s, the sink the flare settles at
    uint8_t landFinalBank;          // degrees
    uint8_t landSlopeTolerance;     // m off the glide slope before going around
    uint8_t landAttempts;           // go-arounds before a landing is committed to
    uint8_t landMaxTailwind;        // dm/s of tailwind on the landing heading before turning it round, 0 never
    uint16_t landLaunchHeight;      // cm above the ground a thrown launch was armed at
} autopilotWingConfig_t;

PG_DECLARE(autopilotWingConfig_t, autopilotWingConfig);

#endif // USE_WING
