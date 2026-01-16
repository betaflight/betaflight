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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_OPTICALFLOW

#include "build/debug.h"

#include "common/maths.h"
#include "common/vector.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "flight/imu.h"
#include "flight/position_estimator.h"

#include "sensors/opticalflow.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

#include "fc/runtime_config.h"

#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
#include "pg/pos_hold_multirotor.h"
#endif

#define OPTICALFLOW_POSITION_TIMEOUT_US 200000  // 200ms timeout

static positionEstimate_t opticalFlowPosition = {
    .position = {{0, 0}},
    .velocity = {{0, 0}},
    .targetPosition = {{0, 0}},
    .lastUpdateUs = 0,
    .isValid = false,
    .trust = 0.0f
};

static positionSource_e currentPositionSource = POSITION_SOURCE_NONE;

void positionEstimatorInit(void)
{
    opticalFlowPosition.position.x = 0;
    opticalFlowPosition.position.y = 0;
    opticalFlowPosition.velocity.x = 0;
    opticalFlowPosition.velocity.y = 0;
    opticalFlowPosition.targetPosition.x = 0;
    opticalFlowPosition.targetPosition.y = 0;
    opticalFlowPosition.lastUpdateUs = 0;
    opticalFlowPosition.isValid = false;
    opticalFlowPosition.trust = 0.0f;
    currentPositionSource = POSITION_SOURCE_NONE;
}

void updateOpticalFlowPosition(void)
{
    if (!sensors(SENSOR_OPTICALFLOW) || !isOpticalflowHealthy()) {
        opticalFlowPosition.isValid = false;
        opticalFlowPosition.trust = 0.0f;
        return;
    }

    if (!sensors(SENSOR_RANGEFINDER) || !rangefinderIsHealthy()) {
        opticalFlowPosition.isValid = false;
        opticalFlowPosition.trust = 0.0f;
        return;
    }

    // Get altitude from rangefinder for scaling
    const int32_t altitudeCm = rangefinderGetLatestAltitude();
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    const int32_t maxRangeCm = posHoldConfig()->opticalflowMaxRange;
#else
    const int32_t maxRangeCm = 400; // Default max range in cm
#endif

    if (altitudeCm < 10 || altitudeCm > maxRangeCm) {
        opticalFlowPosition.isValid = false;
        opticalFlowPosition.trust = 0.0f;
        return;
    }

    // Get optical flow data
    const opticalflow_t *flow = getOpticalFlowData();
    if (flow == NULL) {
        opticalFlowPosition.isValid = false;
        opticalFlowPosition.trust = 0.0f;
       return;
    }

    // Check quality threshold
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    const int minQuality = posHoldConfig()->opticalflowQualityMin;
#else
    const int minQuality = 30; // Default minimum quality
#endif
    if (flow->quality < minQuality) {
        opticalFlowPosition.isValid = false;
        opticalFlowPosition.trust = 0.0f;
        return;
    }

    // Convert flow rates (rad/s) to velocity (cm/s)
    // The flow rates are already processed (rotated and filtered)
    // velocity = flow_rate * altitude
    vector2_t velocityBF;  // Body frame
    velocityBF.x = flow->processedFlowRates.x * altitudeCm;
    velocityBF.y = flow->processedFlowRates.y * altitudeCm;

    // Integrate velocity to get position
    const uint32_t nowUs = micros();
    const uint32_t deltaTimeUs = cmp32(nowUs, opticalFlowPosition.lastUpdateUs);

    if (deltaTimeUs > 0 && deltaTimeUs < 100000) {  // Sanity check: 10Hz - 1000Hz
        const float dt = deltaTimeUs / 1000000.0f;
        opticalFlowPosition.position.x += velocityBF.x * dt;
        opticalFlowPosition.position.y += velocityBF.y * dt;
    }

    opticalFlowPosition.velocity = velocityBF;
    opticalFlowPosition.lastUpdateUs = nowUs;

    // Calculate trust based on quality, ramping linearly from 0 at minQuality to 1 at quality 50
    const int quality = flow->quality;
    const float denom = 50.0f - minQuality;
    if (denom <= 0.0f) {
        opticalFlowPosition.trust = (quality >= 50) ? 1.0f : 0.0f;
    } else {
        opticalFlowPosition.trust = constrainf((float)(quality - minQuality) / denom, 0.0f, 1.0f);
    }

    opticalFlowPosition.isValid = (opticalFlowPosition.trust > 0.0f);

    // Debug output
    DEBUG_SET(DEBUG_OPTICALFLOW_POS, 0, lrintf(opticalFlowPosition.position.x*10));  // Position X (cm×10)
    DEBUG_SET(DEBUG_OPTICALFLOW_POS, 1, lrintf(opticalFlowPosition.position.y*10));  // Position Y (cm×10)
    DEBUG_SET(DEBUG_OPTICALFLOW_POS, 2, lrintf(opticalFlowPosition.targetPosition.x*10));  // Position X (cm×10)
    DEBUG_SET(DEBUG_OPTICALFLOW_POS, 3, lrintf(opticalFlowPosition.targetPosition.y*10));  // Position Y (cm×10)
    DEBUG_SET(DEBUG_OPTICALFLOW_POS, 4, lrintf(opticalFlowPosition.velocity.x*100));  // Velocity X (cm/s×100)
    DEBUG_SET(DEBUG_OPTICALFLOW_POS, 5, lrintf(opticalFlowPosition.velocity.y*100));  // Velocity Y (cm/s×100)
    DEBUG_SET(DEBUG_OPTICALFLOW_POS, 6, lrintf(opticalFlowPosition.trust * 100.0f));  // Trust (0-100%)
}

void resetOpticalFlowPosition(const vector2_t *currentPos)
{
    if (currentPos) {
        opticalFlowPosition.position = *currentPos;
        opticalFlowPosition.targetPosition = *currentPos;
    } else {
        opticalFlowPosition.position.x = 0;
        opticalFlowPosition.position.y = 0;
        opticalFlowPosition.targetPosition.x = 0;
        opticalFlowPosition.targetPosition.y = 0;
    }
    opticalFlowPosition.velocity.x = 0;
    opticalFlowPosition.velocity.y = 0;
    opticalFlowPosition.lastUpdateUs = micros();
}

positionEstimate_t* getOpticalFlowPosition(void)
{
    return &opticalFlowPosition;
}

positionSource_e getActivePositionSource(void)
{
    return currentPositionSource;
}

void setActivePositionSource(positionSource_e source)
{
    currentPositionSource = source;
}

bool isOpticalFlowPositionValid(void)
{
    // Check if data is recent and valid
    if (!opticalFlowPosition.isValid) {
        return false;
    }

    const uint32_t nowUs = micros();
    const uint32_t deltaTimeUs = cmp32(nowUs, opticalFlowPosition.lastUpdateUs);

    return (deltaTimeUs < OPTICALFLOW_POSITION_TIMEOUT_US);
}

void setOpticalFlowTarget(const vector2_t *target)
{
    if (target) {
        opticalFlowPosition.targetPosition = *target;
    }
}

void updateOpticalFlowTargetByAxis(axis_e axis, float value)
{
    if (axis == X) {  // LON/X
        opticalFlowPosition.targetPosition.x = value;
    } else {  // LAT/Y
        opticalFlowPosition.targetPosition.y = value;
    }
}

#endif // USE_OPTICALFLOW
