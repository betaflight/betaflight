/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/position_estimator.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/rangefinder.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

static float displayAltitudeCm = 0.0f;

static pt2Filter_t altitudeLpf;
static pt2Filter_t altitudeDerivativeLpf;

static float filteredAltitudeCm = 0.0f;
static float filteredAltitudeDerivative = 0.0f;

static float controlAltitudeCm = 0.0f;
static float controlAltitudeDerivative = 0.0f;
static float controlAltitudeAcceleration = 0.0f;
#if defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER) || defined(USE_OPTICALFLOW)
static bool wasArmed = false;
#endif
#ifdef USE_VARIO
static int16_t estimatedVario = 0;
#endif

static void positionResetAltitudeState(void)
{
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_POSITION_RATE_HZ);

    const float altitudeCutoffHz = positionConfig()->altitude_lpf / 100.0f;
    const float altitudeGain = pt2FilterGain(altitudeCutoffHz, sampleTimeS);
    pt2FilterInit(&altitudeLpf, altitudeGain);

    const float altitudeDerivativeCutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    const float altitudeDerivativeGain = pt2FilterGain(altitudeDerivativeCutoffHz, sampleTimeS);
    pt2FilterInit(&altitudeDerivativeLpf, altitudeDerivativeGain);

    filteredAltitudeCm = 0.0f;
    displayAltitudeCm = 0.0f;
    filteredAltitudeDerivative = 0.0f;
    controlAltitudeCm = 0.0f;
    controlAltitudeDerivative = 0.0f;
    controlAltitudeAcceleration = 0.0f;
}

void positionInit(void)
{
    positionResetAltitudeState();
#if defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER) || defined(USE_OPTICALFLOW)
    wasArmed = ARMING_FLAG(ARMED);
#endif

    positionEstimatorInit();
}

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 7);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source = ALTITUDE_SOURCE_DEFAULT,
    // How far to trust the barometer against the other altitude sources. Range 0-100,
    // default 50. It scales the baro's measurement noise, so a lower value means less
    // trust: 100 leaves the noise as-is, 50 doubles it, 20 is 5x. Trust is clamped at the
    // bottom, so anything at or below 10 is 10x - 0 does not switch the baro off. The
    // scaling is applied in feedBaroMeasurements().
    .altitude_prefer_baro = 50,
    .altitude_lpf = 300,
    .altitude_d_lpf = 300,
    .rangefinder_max_range_cm = 400,
);

#if defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER) || defined(USE_OPTICALFLOW)
// The altitude presentation layer: display smoothing and vario, derived from the
// Z axis of the estimate. A consumer of the estimator, not the reason it runs.
static void updateAltitudeFromEstimate(void)
{
    // Get raw KF altitude estimate
    const float kfAltCm = positionEstimatorGetAltitudeCm();
    const float kfVelZCm = positionEstimatorGetVerticalVelocity();
    const float kfAccelZCm = positionEstimatorGetVerticalAcceleration();

    // Keep altitude estimate updating while disarmed so sensors/debug views show live data.
    // Arming-specific references are handled in estimator sensor offsets/reset logic.
    filteredAltitudeCm = pt2FilterApply(&altitudeLpf, kfAltCm);
    displayAltitudeCm = filteredAltitudeCm;

    controlAltitudeCm = kfAltCm;
    controlAltitudeDerivative = kfVelZCm;
    controlAltitudeAcceleration = kfAccelZCm;

    filteredAltitudeDerivative = pt2FilterApply(&altitudeDerivativeLpf, controlAltitudeDerivative);

#ifdef USE_VARIO
    estimatedVario = lrintf(filteredAltitudeDerivative);
    estimatedVario = applyDeadband(estimatedVario, 10);
#endif

    // DEBUG_ALTITUDE layout:
    // 0 = relative rangefinder altitude       (written in feedRangefinderMeasurements)
    // 1 = relative barometer altitude          (written in feedBaroMeasurements)
    // 2 = relative GPS altitude                (written in feedGPSMeasurements)
    // 3 = KF altitude
    // 4 = GPS vertical velocity                (written in feedGPSMeasurements)
    // 5 = KF vertical velocity
    // 6 = Vertical accelerometer               (written in positionEstimatorUpdate)
    // 7 = KF vertical acceleration

    DEBUG_SET(DEBUG_ALTITUDE, 3, lrintf(kfAltCm));
    DEBUG_SET(DEBUG_ALTITUDE, 5, lrintf(kfVelZCm));
    DEBUG_SET(DEBUG_ALTITUDE, 7, lrintf(kfAccelZCm));

    DEBUG_SET(DEBUG_RTH, 1, lrintf(displayAltitudeCm));
}

// TASK_POSITION: run the Kalman filter estimator (prediction plus all sensor
// measurement updates), then refresh the layers derived from it. The estimate
// spans all three axes and feeds altitude hold, position hold, nav and rescue,
// so this is the fusion step for the whole craft, not an altitude calculation.
void positionUpdate(void)
{
    const bool isArmed = ARMING_FLAG(ARMED);
    if (isArmed != wasArmed) {
        positionEstimatorResetZ();
        positionResetAltitudeState();
        wasArmed = isArmed;
    }

    positionEstimatorUpdate();

    updateAltitudeFromEstimate();
}

#endif // defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER) || defined(USE_OPTICALFLOW)

float getAltitudeCm(void)
{
    return filteredAltitudeCm;
}

float getAltitudeDerivative(void)
{
    return filteredAltitudeDerivative;
}

float getAltitudeCmControl(void)
{
    return controlAltitudeCm; // un-filtered current altitude in cm
}

float getAltitudeDerivativeControl(void)
{
    return controlAltitudeDerivative;
}
float getAltitudeAccelerationControl(void)
{
    return controlAltitudeAcceleration;
}


bool isAltitudeAvailable(void)
{
    return positionEstimatorIsValidZ();
}

int32_t getEstimatedAltitudeCm(void)
{
    return lrintf(displayAltitudeCm);
}

#ifdef USE_GPS
float getAltitudeAsl(void)
{
    return gpsSol.llh.altCm;
}
#endif

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
