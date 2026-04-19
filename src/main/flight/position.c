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
#if defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER)
static bool wasArmed = false;
#endif
#ifdef USE_VARIO
static int16_t estimatedVario = 0;
#endif

static void positionResetAltitudeState(void)
{
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

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
}

void positionInit(void)
{
    positionResetAltitudeState();
    wasArmed = ARMING_FLAG(ARMED);

    positionEstimatorInit();
}

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 7);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source = ALTITUDE_SOURCE_DEFAULT,
    .altitude_prefer_baro = 100,
    .altitude_lpf = 300,
    .altitude_d_lpf = 300,
    .rangefinder_max_range_cm = 400,
);

#if defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER)
void calculateEstimatedAltitude(void)
{
    const bool isArmed = ARMING_FLAG(ARMED);

    if (isArmed != wasArmed) {
        positionEstimatorResetZ();
        positionResetAltitudeState();
        wasArmed = isArmed;
    }

    // Run the Kalman filter estimator (prediction + all sensor measurement updates)
    positionEstimatorUpdate();

    // Get raw KF altitude estimate
    const float kfAltCm = positionEstimatorGetAltitudeCm();
    const float kfVelCm = positionEstimatorGetAltitudeDerivative();

    // Keep altitude estimate updating while disarmed so sensors/debug views show live data.
    // Arming-specific references are handled in estimator sensor offsets/reset logic.
    filteredAltitudeCm = pt2FilterApply(&altitudeLpf, kfAltCm);
    displayAltitudeCm = filteredAltitudeCm;
    controlAltitudeCm = kfAltCm;
    controlAltitudeDerivative = kfVelCm;

    filteredAltitudeDerivative = pt2FilterApply(&altitudeDerivativeLpf, controlAltitudeDerivative);

#ifdef USE_VARIO
    estimatedVario = lrintf(filteredAltitudeDerivative);
    estimatedVario = applyDeadband(estimatedVario, 10);
#endif

    DEBUG_SET(DEBUG_ALTITUDE, 0, lrintf(positionEstimatorGetEstimate()->trustZ * 100));
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf(kfAltCm / 10.0f));
    DEBUG_SET(DEBUG_ALTITUDE, 2, lrintf(filteredAltitudeCm / 10.0f));
#ifdef USE_VARIO
    DEBUG_SET(DEBUG_ALTITUDE, 3, estimatedVario);
#endif
    DEBUG_SET(DEBUG_RTH, 1, lrintf(displayAltitudeCm / 10.0f));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 2, lrintf(filteredAltitudeCm));

    wasArmed = armed;
}

#endif // defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER)

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
    return controlAltitudeCm;
}

float getAltitudeDerivativeControl(void)
{
    return controlAltitudeDerivative;
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
