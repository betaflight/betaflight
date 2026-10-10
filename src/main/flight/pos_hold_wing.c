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

#include "platform.h"

#ifdef USE_WING

#ifdef USE_POSITION_HOLD

#include "common/maths.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"
#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/landing_wing.h"
#include "flight/position_estimator.h"
#include "scheduler/scheduler.h"

#include "pg/pos_hold.h"
#include "pos_hold.h"

// Event driven off positionEstimatorUpdate(); without the estimator the task falls back to
// periodic scheduling so that mode entry and exit are still serviced.
#define POSHOLD_FALLBACK_PERIOD_US (2 * TASK_PERIOD_HZ(POSHOLD_TASK_RATE_HZ))

static bool isEnabled;

void posHoldInit(void)
{
    isEnabled = false;
}

bool posHoldUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);

    if (positionEstimatorTakeUpdate(POS_EST_CONSUMER_POSHOLD)) {
        return true;
    }

    return currentDeltaTimeUs >= POSHOLD_FALLBACK_PERIOD_US;
}

// The aircraft loiters about where position hold engaged. The roll stick takes over the steering,
// and the loiter starts again about wherever it is let go; the pitch stick belongs to altitude hold.
void updatePosHold(timeUs_t currentTimeUs)
{
    landingWingNoteDepartureCourse(currentTimeUs);
#ifdef USE_GPS_RESCUE
    gpsRescueNoteMaxAltitude();
#endif

    if (!FLIGHT_MODE(POS_HOLD_MODE)) {
        if (isEnabled) {
            resetPositionControl(POSHOLD_TASK_RATE_HZ);
        }
        isEnabled = false;
        return;
    }

    if (!isEnabled) {
        resetPositionControl(POSHOLD_TASK_RATE_HZ);
        isEnabled = true;
    }
    setSticksActiveStatus(!failsafeIsActive() && getRcDeflectionAbs(FD_ROLL) > posHoldConfig()->deadband * 0.01f);
    positionControl();
}

bool posHoldFailure(void)
{
    return FLIGHT_MODE(POS_HOLD_MODE) && !positionEstimatorIsValidXY();
}

bool posHoldReady(void)
{
    return positionEstimatorIsValidXY();
}

#endif // USE_POSITION_HOLD

#endif // USE_WING
