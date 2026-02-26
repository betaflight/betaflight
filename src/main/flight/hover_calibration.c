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

#ifdef USE_HOVER_CALIBRATION

#include <math.h>

#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"

#include "config/config.h"

#include "drivers/time.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"

#include "io/beeper.h"

#include "pg/autopilot.h"

#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "hover_calibration.h"

// Calibration parameters
#define HOVER_CAL_MIN_ALTITUDE_CM       50      // Must be >50cm to avoid ground effect/prop wash
#define HOVER_CAL_VELOCITY_THRESHOLD    30.0f   // cm/s - vertical velocity threshold for "stable"
#define HOVER_CAL_TILT_COS_THRESHOLD    0.985f  // ~10 degrees max tilt (cos(10deg) ≈ 0.985)
#define HOVER_CAL_STABILITY_TIME_MS     500     // Must be stable for 500ms before sampling starts
#define HOVER_CAL_SAMPLE_TIME_MS        2000    // Collect samples for 2 seconds
#define HOVER_CAL_THROTTLE_MIN          1100    // Sanity bounds for result
#define HOVER_CAL_THROTTLE_MAX          1700
#define HOVER_CAL_PROGRESS_BEEP_MS      500     // Beep every 500ms during sampling

typedef struct {
    hoverCalibrationStatus_e status;
    hoverCalibrationFailReason_e failReason;
    uint16_t sampleCount;
    float throttleSum;
    timeMs_t stabilityStartTime;
    timeMs_t samplingStartTime;
    timeMs_t lastProgressBeepTime;
    uint16_t result;
} hoverCalibrationState_t;

static hoverCalibrationState_t hoverCal;

void hoverCalibrationInit(void)
{
    hoverCal.status = HOVER_CAL_STATUS_IDLE;
    hoverCal.failReason = HOVER_CAL_FAIL_NONE;
    hoverCal.sampleCount = 0;
    hoverCal.throttleSum = 0.0f;
    hoverCal.stabilityStartTime = 0;
    hoverCal.samplingStartTime = 0;
    hoverCal.lastProgressBeepTime = 0;
    hoverCal.result = 0;
}

void hoverCalibrationStart(void)
{
    // Allow restart from IDLE, COMPLETE, or FAILED states
    if (hoverCal.status == HOVER_CAL_STATUS_WAITING_STABLE || 
        hoverCal.status == HOVER_CAL_STATUS_SAMPLING) {
        return;  // Already in progress
    }

    hoverCal.status = HOVER_CAL_STATUS_WAITING_STABLE;
    hoverCal.failReason = HOVER_CAL_FAIL_NONE;
    hoverCal.sampleCount = 0;
    hoverCal.throttleSum = 0.0f;
    hoverCal.stabilityStartTime = millis();
    hoverCal.samplingStartTime = 0;
    hoverCal.lastProgressBeepTime = 0;
    hoverCal.result = 0;

    beeper(BEEPER_RX_SET);  // Short beep to confirm start
}

static void hoverCalibrationFail(hoverCalibrationFailReason_e reason)
{
    hoverCal.status = HOVER_CAL_STATUS_FAILED;
    hoverCal.failReason = reason;
    beeper(BEEPER_ACC_CALIBRATION_FAIL);  // 2 longer beeps for failure
}

void hoverCalibrationAbort(void)
{
    if (hoverCal.status == HOVER_CAL_STATUS_IDLE) {
        return;
    }

    hoverCalibrationFail(HOVER_CAL_FAIL_DISARMED);
}

// Returns fail reason, or HOVER_CAL_FAIL_NONE if stable
static hoverCalibrationFailReason_e checkHoverStability(void)
{
    // Must be armed
    if (!ARMING_FLAG(ARMED)) {
        return HOVER_CAL_FAIL_WAITING_ARM;
    }

    // Must NOT be in altitude hold or GPS rescue (we want manual throttle)
#ifdef USE_ALTITUDE_HOLD
    if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        return HOVER_CAL_FAIL_ALTHOLD_MODE;
    }
#endif
#ifdef USE_GPS_RESCUE
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        return HOVER_CAL_FAIL_ALTHOLD_MODE;
    }
#endif

    // Check altitude sensor is available
    if (!isAltitudeAvailable()) {
        return HOVER_CAL_FAIL_NO_ALTITUDE;
    }

    // Check altitude is above minimum (avoid ground effect/prop wash)
    if (getAltitudeCm() < HOVER_CAL_MIN_ALTITUDE_CM) {
        return HOVER_CAL_FAIL_TOO_LOW;
    }

    // Check vertical velocity is low (hovering, not climbing/descending)
    const float verticalVelocity = fabsf(getAltitudeDerivative());
    if (verticalVelocity > HOVER_CAL_VELOCITY_THRESHOLD) {
        return HOVER_CAL_FAIL_MOVING;
    }

    // Check attitude is level
    const float tiltCos = getCosTiltAngle();
    if (tiltCos < HOVER_CAL_TILT_COS_THRESHOLD) {
        return HOVER_CAL_FAIL_NOT_LEVEL;
    }

    return HOVER_CAL_FAIL_NONE;  // Stable!
}

void hoverCalibrationUpdate(void)
{
    // Only process if calibration is active
    if (hoverCal.status == HOVER_CAL_STATUS_IDLE ||
        hoverCal.status == HOVER_CAL_STATUS_COMPLETE ||
        hoverCal.status == HOVER_CAL_STATUS_FAILED) {
        return;
    }

    const timeMs_t currentTime = millis();

    // Check stability and get specific failure reason
    const hoverCalibrationFailReason_e stabilityResult = checkHoverStability();
    const bool stable = (stabilityResult == HOVER_CAL_FAIL_NONE);
    const bool isDisarmed = (stabilityResult == HOVER_CAL_FAIL_DISARMED);

    // Track current instability reason for status display
    if (!stable) {
        hoverCal.failReason = stabilityResult;
    }

    // Debug output
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 0, hoverCal.status);
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 1, lrintf(getAltitudeCm()));
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 2, lrintf(fabsf(getAltitudeDerivative())));
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 3, lrintf(rcCommand[THROTTLE]));
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 4, hoverCal.sampleCount);
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 5, getHoverCalibrationProgress());
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 6, hoverCal.result);
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 7, stabilityResult);

    // Handle disarmed state (WAITING_ARM)
    if (isDisarmed) {
        if (hoverCal.status == HOVER_CAL_STATUS_SAMPLING && hoverCal.sampleCount > 0) {
            // Was sampling and got some data - complete with what we have
            // This handles the case where user lands after hovering
            uint16_t avgThrottle = lrintf(hoverCal.throttleSum / hoverCal.sampleCount);
            if (avgThrottle >= HOVER_CAL_THROTTLE_MIN && avgThrottle <= HOVER_CAL_THROTTLE_MAX) {
                hoverCal.result = avgThrottle;
                autopilotConfigMutable()->hoverThrottle = avgThrottle;
                hoverCal.status = HOVER_CAL_STATUS_COMPLETE;
                hoverCal.failReason = HOVER_CAL_FAIL_NONE;
                beeper(BEEPER_ACC_CALIBRATION);
            } else {
                hoverCalibrationFail(HOVER_CAL_FAIL_RESULT_RANGE);
            }
        } else if (hoverCal.status == HOVER_CAL_STATUS_SAMPLING) {
            // Was sampling but no data collected yet - fail
            hoverCalibrationFail(HOVER_CAL_FAIL_DISARMED);
        }
        // If still in WAITING_STABLE, just keep waiting for arm - don't fail
        // User needs to arm, fly, and hover to calibrate
        return;
    }

    if (!stable) {
        // Lost stability while armed - reset to waiting state
        if (hoverCal.status == HOVER_CAL_STATUS_SAMPLING) {
            // Was sampling, now lost stability - reset samples
            hoverCal.sampleCount = 0;
            hoverCal.throttleSum = 0.0f;
        }
        hoverCal.status = HOVER_CAL_STATUS_WAITING_STABLE;
        hoverCal.stabilityStartTime = currentTime;
        return;
    }
    
    // Clear fail reason when stable
    hoverCal.failReason = HOVER_CAL_FAIL_NONE;

    // Stable - check which phase we're in
    if (hoverCal.status == HOVER_CAL_STATUS_WAITING_STABLE) {
        // Check if stable long enough to start sampling
        if (cmpTimeMs(currentTime, hoverCal.stabilityStartTime) >= HOVER_CAL_STABILITY_TIME_MS) {
            // Transition to sampling
            hoverCal.status = HOVER_CAL_STATUS_SAMPLING;
            hoverCal.samplingStartTime = currentTime;
            hoverCal.lastProgressBeepTime = currentTime;
            hoverCal.sampleCount = 0;
            hoverCal.throttleSum = 0.0f;
            beeper(BEEPER_RX_SET);  // Beep to indicate sampling started
        }
        return;
    }

    if (hoverCal.status == HOVER_CAL_STATUS_SAMPLING) {
        // Collect throttle sample
        hoverCal.throttleSum += rcCommand[THROTTLE];
        hoverCal.sampleCount++;

        // Progress beep
        if (cmpTimeMs(currentTime, hoverCal.lastProgressBeepTime) >= HOVER_CAL_PROGRESS_BEEP_MS) {
            beeper(BEEPER_RX_SET);
            hoverCal.lastProgressBeepTime = currentTime;
        }

        // Check if sampling complete
        if (cmpTimeMs(currentTime, hoverCal.samplingStartTime) >= HOVER_CAL_SAMPLE_TIME_MS) {
            // Calculate average throttle
            if (hoverCal.sampleCount > 0) {
                uint16_t avgThrottle = lrintf(hoverCal.throttleSum / hoverCal.sampleCount);

                // Validate result is within sane bounds
                if (avgThrottle >= HOVER_CAL_THROTTLE_MIN && avgThrottle <= HOVER_CAL_THROTTLE_MAX) {
                    // Success! Store result
                    hoverCal.result = avgThrottle;
                    autopilotConfigMutable()->hoverThrottle = avgThrottle;
                    hoverCal.status = HOVER_CAL_STATUS_COMPLETE;
                    beeper(BEEPER_ACC_CALIBRATION);  // 2 short beeps for success
                } else {
                    // Result out of bounds - fail
                    hoverCalibrationFail(HOVER_CAL_FAIL_RESULT_RANGE);
                }
            } else {
                // No samples collected - shouldn't happen but handle it
                hoverCalibrationFail(HOVER_CAL_FAIL_RESULT_RANGE);
            }
        }
    }
}

bool isHoverCalibrationActive(void)
{
    return hoverCal.status == HOVER_CAL_STATUS_WAITING_STABLE ||
           hoverCal.status == HOVER_CAL_STATUS_SAMPLING;
}

hoverCalibrationStatus_e getHoverCalibrationStatus(void)
{
    return hoverCal.status;
}

hoverCalibrationFailReason_e getHoverCalibrationFailReason(void)
{
    return hoverCal.failReason;
}

uint8_t getHoverCalibrationProgress(void)
{
    if (hoverCal.status != HOVER_CAL_STATUS_SAMPLING) {
        return 0;
    }

    const timeMs_t elapsed = cmpTimeMs(millis(), hoverCal.samplingStartTime);
    if (elapsed >= HOVER_CAL_SAMPLE_TIME_MS) {
        return 100;
    }

    return (uint8_t)((elapsed * 100) / HOVER_CAL_SAMPLE_TIME_MS);
}

uint16_t getHoverCalibrationResult(void)
{
    return hoverCal.result;
}

#endif // USE_HOVER_CALIBRATION
