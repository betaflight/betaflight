/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#ifdef USE_WING

#include "build/debug.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"

#include "pg/autopilot.h"
#include "autopilot.h"

float autopilotAngle[RP_AXIS_COUNT];

void resetPositionControl(const gpsLocation_t *initialTargetLocation, unsigned taskRateHz)
{
    // from pos_hold.c (or other client) when initiating position hold at target location
    UNUSED(initialTargetLocation);
    UNUSED(taskRateHz);
}

void autopilotInit(void)
{
}

void resetAltitudeControl (void) {
}

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeStep)
{
    UNUSED(targetAltitudeCm);
    UNUSED(taskIntervalS);
    UNUSED(targetAltitudeStep);
}

void setSticksActiveStatus(bool areSticksActive)
{
    UNUSED(areSticksActive);
}

bool positionControl(void)
{
    return false;
}

bool isBelowLandingAltitude(void)
{
    return false;
}

float getAutopilotThrottle(void)
{
    return 0.0f;
}

bool isAutopilotInControl(void)
{
    return false;
}

#endif // USE_WING
