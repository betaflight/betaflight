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
#include "sensors/compass.h"
#include "sensors/opticalflow.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"
#ifdef USE_GPS
#include "io/gps.h"
#endif

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
    const bool sticksDeflected = (getRcDeflectionAbs(FD_ROLL) > posHold.deadband) || (getRcDeflectionAbs(FD_PITCH) > posHold.deadband);
    setSticksActiveStatus(sticksDeflected);
}

static bool sensorsOk(void)
{
    const uint8_t posSource = posHoldConfig()->positionSource;

#ifdef USE_OPTICALFLOW
    // Check if optical flow is available and configured
    if (posSource == POSHOLD_SOURCE_AUTO || posSource == POSHOLD_SOURCE_OPTICALFLOW_ONLY) {
        if (sensors(SENSOR_OPTICALFLOW) && isOpticalflowHealthy() &&
            sensors(SENSOR_RANGEFINDER) && rangefinderIsHealthy()) {
            const positionEstimate_t *flowPos = getOpticalFlowPosition();
            if (flowPos->isValid && isOpticalFlowPositionValid()) {
                // Optical flow is available and valid
                setActivePositionSource(POSITION_SOURCE_OPTICALFLOW);
                return true;
            }
        }
    }

    // If optical flow only mode and not available, fail
    if (posSource == POSHOLD_SOURCE_OPTICALFLOW_ONLY) {
        return false;
    }
#endif

    // Check GPS availability
#ifdef USE_GPS
    if (!STATE(GPS_FIX)) {
        return false;
    }
    if (
#ifdef USE_MAG
        !compassIsHealthy() &&
#endif
        (!posHoldConfig()->posHoldWithoutMag || !canUseGPSHeading)) {
        return false;
    }

#ifdef USE_OPTICALFLOW
    setActivePositionSource(POSITION_SOURCE_GPS);
#endif
    return true;
#else
    return false;
#endif
}

void updatePosHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHold.isEnabled) {
#ifdef USE_OPTICALFLOW
            const uint8_t posSource = posHoldConfig()->positionSource;
            // Check if we should use optical flow
            if ((posSource == POSHOLD_SOURCE_AUTO || posSource == POSHOLD_SOURCE_OPTICALFLOW_ONLY) &&
                sensors(SENSOR_OPTICALFLOW) && isOpticalflowHealthy() &&
                sensors(SENSOR_RANGEFINDER) && rangefinderIsHealthy()) {

                // Initialize optical flow position estimator
                positionEstimatorInit();
                vector2_t startPos = {{0, 0}};
                resetOpticalFlowPosition(&startPos);
                setActivePositionSource(POSITION_SOURCE_OPTICALFLOW);

                // Reset position control (GPS target used for fallback)
#ifdef USE_GPS
                resetPositionControl(&gpsSol.llh, POSHOLD_TASK_RATE_HZ);
#else
                {
                    static const gpsLocation_t defaultLoc = {0};
                    resetPositionControl(&defaultLoc, POSHOLD_TASK_RATE_HZ);
                }
#endif
            } else
#endif
            {
#ifdef USE_GPS
                resetPositionControl(&gpsSol.llh, POSHOLD_TASK_RATE_HZ);
#endif
#ifdef USE_OPTICALFLOW
                setActivePositionSource(POSITION_SOURCE_GPS);
#endif
            }
            posHold.isControlOk = true;
            posHold.isEnabled = true;
        }
    } else {
        if (posHold.isEnabled) {
            // Reset autopilot angles when exiting position hold
            for (unsigned i = 0; i < RP_AXIS_COUNT; i++) {
                autopilotAngle[i] = 0.0f;
            }
            setSticksActiveStatus(false);  // Ensure autopilot releases control
        }
        posHold.isEnabled = false;
    }

    if (posHold.isEnabled && posHold.isControlOk) {
#ifdef USE_OPTICALFLOW
        updateOpticalFlowPosition();
#endif
        posHold.areSensorsOk = sensorsOk();
        if (posHold.areSensorsOk) {
            posHoldCheckSticks();
            posHold.isControlOk = positionControl(); // false only on sanity check failure
        }
    }
}

bool posHoldFailure(void) {
    // used only to display warning in OSD if requested but failing
    return FLIGHT_MODE(POS_HOLD_MODE) && (!posHold.isControlOk || !posHold.areSensorsOk);
}

#endif // USE_POSITION_HOLD

#endif // !USE_WING
