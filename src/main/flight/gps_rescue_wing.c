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

#include <math.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_WING
#ifdef USE_GPS_RESCUE

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/position_estimator.h"

#include "io/gps.h"

#include "gps_rescue.h"

static struct {
    float maxAltitudeCm;
    bool isAvailable;
} rescueState;

void gpsRescueInit(void)
{
    rescueState.isAvailable = true;
}

// While disarmed the maximum altitude is zero, unless set_home_point_once keeps it until a power cycle
void gpsRescueNoteMaxAltitude(void)
{
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        rescueState.maxAltitudeCm = 0.0f;
        return;
    }
    rescueState.maxAltitudeCm = fmaxf(getAltitudeCmControl(), rescueState.maxAltitudeCm);
}

// The rescue is flown as an autopilot mission; this only keeps its availability. A wing steers by
// its course over the ground, so it needs no heading to fly home.
void gpsRescueUpdate(void)
{
    rescueState.isAvailable = STATE(GPS_FIX_HOME) && gpsIsHealthy() && isAltitudeAvailable() && positionEstimatorIsValidXY();
}

float gpsRescueGetMaxAltitudeCm(void)
{
    return rescueState.maxAltitudeCm;
}

bool gpsRescueIsConfigured(void)
{
#if ENABLE_RESCUE_PLAN
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE);
#else
    return false;
#endif
}

bool gpsRescueIsAvailable(void)
{
    return rescueState.isAvailable;
}

bool gpsRescueIsHeadingOK(void)
{
    return true;
}

bool gpsRescueIsOK(void)
{
    return true;
}

#endif // USE_GPS_RESCUE

#endif // USE_WING
