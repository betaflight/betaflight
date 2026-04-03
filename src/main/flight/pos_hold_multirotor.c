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

#ifndef USE_WING

#ifdef USE_POSITION_HOLD

#include "math.h"
#include "build/debug.h"
#include "common/maths.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "rx/rx.h"

#include "pg/pos_hold.h"
#include "pos_hold.h"

typedef struct posHoldState_s {
    bool isEnabled;
    bool isControlOk;
    bool areSensorsOk;
    float deadband;
} posHoldState_t;

static posHoldState_t posHold;

void posHoldInit(void)
{
    posHold.deadband = posHoldConfig()->deadband * 0.01f;
}

static void posHoldCheckSticks(void)
{
    if (failsafeIsActive()) {
        setSticksActiveStatus(false);
        return;
    }
    const bool sticksDeflected =
        (getRcDeflectionAbs(FD_ROLL) > posHold.deadband) || (getRcDeflectionAbs(FD_PITCH) > posHold.deadband);
    setSticksActiveStatus(sticksDeflected);
}

static bool sensorsOk(void)
{
    // Source-agnostic: the estimator determines validity based on which
    // sensors are available and the user's source settings.
    return positionEstimatorIsValidXY();
}

void updatePosHold(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHold.isEnabled) {
            resetPositionControl(POSHOLD_TASK_RATE_HZ);
            posHold.isControlOk = true;
            posHold.isEnabled = true;
        }
    } else {
        if (posHold.isEnabled) {
            for (unsigned i = 0; i < RP_AXIS_COUNT; i++) {
                autopilotAngle[i] = 0.0f;
            }
            setSticksActiveStatus(false);
        }
        posHold.isEnabled = false;
    }

    if (posHold.isEnabled && posHold.isControlOk) {
        posHoldCheckSticks();
        posHold.areSensorsOk = sensorsOk();
        if (posHold.areSensorsOk) {
            posHold.isControlOk = positionControl();
        } else {
            for (unsigned i = 0; i < RP_AXIS_COUNT; i++) {
                autopilotAngle[i] = 0.0f;
            }
        }
    }
}

bool posHoldFailure(void)
{
    return FLIGHT_MODE(POS_HOLD_MODE) && (!posHold.isControlOk || !posHold.areSensorsOk);
}

#endif // USE_POSITION_HOLD

#endif // !USE_WING
