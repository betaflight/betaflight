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

#include "platform.h"

#ifdef USE_WING

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "autopilot_wing.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autopilotWingConfig_t, autopilotWingConfig, PG_AUTOPILOT_WING, 0);

PG_RESET_TEMPLATE(autopilotWingConfig_t, autopilotWingConfig,
    .cruiseThrottle = 50,
    .minThrottle = 25,
    .maxThrottle = 90,
    .bankThrottle = 10,
    .maxClimbAngle = 15,
    .maxDiveAngle = 12,
    .maxClimbRate = 30,
    .maxSinkRate = 30,
    .altTimeConstant = 40,
    .climbP = 10,
    .climbI = 10,
    .vertAccel = 50,
    .cruiseSpeed = 150,
    .turnPitchFf = 100,
    .maxBank = 35,
    .l1Period = 130,
    .l1Damping = 75,
    .loiterRadius = 60,
    .loiterDirection = WING_LOITER_RIGHT,
    .landHeading = -1,
    .landSide = WING_LOITER_LEFT,
    .landFinalLength = 150,
    .landGlideAngle = 8,
    .landApproachAlt = 30,
    .landFlareHeight = 300,
    .landFlarePitch = 5,
    .landFlareSink = 5,
    .landFinalBank = 15,
    .landSlopeTolerance = 10,
    .landAttempts = 3,
    .landMaxTailwind = 30,
    .landLaunchHeight = 150,
);

#endif // USE_WING
