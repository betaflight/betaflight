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
#include <math.h>

#include "platform.h"

#ifdef USE_WING
#ifdef USE_GPS_RESCUE

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "io/gps.h"
#include "rx/rx.h"
#include "pg/autopilot.h"
#include "sensors/acceleration.h"

#include "gps_rescue.h"

float gpsRescueAngle[RP_AXIS_COUNT] = { 0, 0 };

void gpsRescueInit(void)
{
}

void gpsRescueUpdate(void)
// runs at gpsRescueTaskIntervalSeconds, and runs whether or not rescue is active
{
}

float gpsRescueGetYawRate(void)
{
    return 0.0f; // the control yaw value for rc.c to be used while flightMode gps_rescue is active.
}

bool gpsRescueIsConfigured(void)
{
    return false;
}

bool gpsRescueIsAvailable(void)
{
    return true;
}

bool gpsRescueIsHeadingOK(void)
{
    return true;
}

bool gpsRescueIsOK(void)
{
    return true;
}

#ifdef USE_MAG
bool gpsRescueDisableMag(void)
{
    return true;
}
#endif

#endif // USE_GPS_RESCUE

#endif // USE_WING
