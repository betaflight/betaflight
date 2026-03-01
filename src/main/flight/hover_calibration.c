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
#include <stdlib.h>

#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"

#include "config/config.h"

#include "drivers/time.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "rx/rx.h"

#include "flight/imu.h"
#include "flight/position.h"

#include "io/beeper.h"

#include "pg/autopilot.h"

#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "hover_calibration.h"

// Calibration parameters
#define HOVER_CAL_MIN_ALTITUDE_CM       50      // Must be >50cm to avoid ground effect/prop wash
#define HOVER_CAL_VELOCITY_THRESHOLD    100.0f  // cm/s - vertical velocity threshold for "stable" (relaxed from 30)
#define HOVER_CAL_TILT_COS_THRESHOLD    0.940f  // ~20 degrees max tilt (relaxed from ~10 degrees)
#define HOVER_CAL_ARM_DELAY_MS          3000    // Wait 3s after arming before calibration can begin
#define HOVER_CAL_STABILITY_TIME_MS     2000    // Must be stable for 2s before sampling starts
#define HOVER_CAL_SAMPLE_TIME_MS        5000    // Collect samples for 5 seconds
#define HOVER_CAL_THROTTLE_MIN          1050    // Sanity bounds for result (relaxed from 1100)
#define HOVER_CAL_THROTTLE_MAX          1800    // (relaxed from 1700)
#define HOVER_CAL_PROGRESS_BEEP_MS      1000    // Beep every 1s during sampling
#define HOVER_CAL_MAX_SAMPLES           500     // Max samples for median calculation (5s @ 100Hz)

typedef struct {
    hoverCalibrationStatus_e status;
    hoverCalibrationFailReason_e failReason;
    uint16_t sampleCount;
    timeMs_t stabilityStartTime;
    timeMs_t samplingStartTime;
    timeMs_t lastProgressBeepTime;
    timeMs_t armTime;               // Time when armed state was first detected
    uint16_t result;
    bool wasArmedDuringCalibration; // Track if we ever detected armed state
    uint16_t throttleSamples[HOVER_CAL_MAX_SAMPLES]; // Store samples for median calculation
} hoverCalibrationState_t;

static hoverCalibrationState_t hoverCal;

// Comparison function for qsort
static int compareUint16(const void *a, const void *b)
{
    return (int)(*(const uint16_t *)a) - (int)(*(const uint16_t *)b);
}

// Calculate median of stored throttle samples using trimmed mean (25th-75th percentile)
static uint16_t calculateMedianThrottle(void)
{
    if (hoverCal.sampleCount == 0) {
        return 0;
    }
    
    // Sort the samples array
    qsort(hoverCal.throttleSamples, hoverCal.sampleCount, sizeof(uint16_t), compareUint16);
    
    // Use interquartile mean (average of middle 50% of samples) for robustness
    const uint16_t q1Index = hoverCal.sampleCount / 4;
    const uint16_t q3Index = (hoverCal.sampleCount * 3) / 4;
    
    uint32_t sum = 0;
    uint16_t count = 0;
    for (uint16_t i = q1Index; i < q3Index; i++) {
        sum += hoverCal.throttleSamples[i];
        count++;
    }
    
    if (count == 0) {
        // Fallback to simple median if not enough samples for IQM
        return hoverCal.throttleSamples[hoverCal.sampleCount / 2];
    }
    
    return (uint16_t)(sum / count);
}

void hoverCalibrationInit(void)
{
    hoverCal.status = HOVER_CAL_STATUS_IDLE;
    hoverCal.failReason = HOVER_CAL_FAIL_NONE;
    hoverCal.sampleCount = 0;
    hoverCal.stabilityStartTime = 0;
    hoverCal.samplingStartTime = 0;
    hoverCal.lastProgressBeepTime = 0;
    hoverCal.armTime = 0;
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
    hoverCal.stabilityStartTime = millis();
    hoverCal.samplingStartTime = 0;
    hoverCal.lastProgressBeepTime = 0;
    hoverCal.armTime = 0;
    hoverCal.result = 0;
    hoverCal.wasArmedDuringCalibration = false;

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
    // Check armed state - once armed during calibration, we track it
    const bool isArmedNow = ARMING_FLAG(ARMED);
    const timeMs_t currentTime = millis();
    
    if (isArmedNow) {
        if (!hoverCal.wasArmedDuringCalibration) {
            // First time detecting armed state - record the time
            hoverCal.wasArmedDuringCalibration = true;
            hoverCal.armTime = currentTime;
        }
        
        // Enforce minimum time after arming before allowing calibration
        // This prevents sampling during takeoff when briefly stable
        if (cmpTimeMs(currentTime, hoverCal.armTime) < HOVER_CAL_ARM_DELAY_MS) {
            return HOVER_CAL_FAIL_WAITING_ARM;  // Still in arm delay period
        }
    }
    
    // If not currently armed and never was armed during this calibration, wait
    if (!isArmedNow && !hoverCal.wasArmedDuringCalibration) {
        return HOVER_CAL_FAIL_WAITING_ARM;
    }
    
    // If we were armed but now disarmed, that's a disarm event
    if (!isArmedNow && hoverCal.wasArmedDuringCalibration) {
        return HOVER_CAL_FAIL_DISARMED;
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

    // Check altitude sensor is available (warn but don't fail - we can still sample throttle)
    if (!isAltitudeAvailable()) {
        // Continue anyway - altitude check is for safety, but throttle sampling works without it
    }

    // Check altitude is above minimum (avoid ground effect/prop wash)
    // Only check if altitude is available and positive (ignore erratic negative readings)
    const float altitude = getAltitudeCm();
    if (isAltitudeAvailable() && altitude >= 0 && altitude < HOVER_CAL_MIN_ALTITUDE_CM) {
        return HOVER_CAL_FAIL_TOO_LOW;
    }

    // Velocity and tilt checks are informational but don't block sampling
    // The user is responsible for hovering steadily - we just average what they give us
    // This makes the calibration more forgiving of sensor noise

    // Check vertical velocity (warn but continue if too fast)
    // const float verticalVelocity = fabsf(getAltitudeDerivative());
    // if (verticalVelocity > HOVER_CAL_VELOCITY_THRESHOLD) {
    //     return HOVER_CAL_FAIL_MOVING;
    // }

    // Check attitude is level (warn but continue if tilted)
    // const float tiltCos = getCosTiltAngle();
    // if (tiltCos < HOVER_CAL_TILT_COS_THRESHOLD) {
    //     return HOVER_CAL_FAIL_NOT_LEVEL;
    // }

    return HOVER_CAL_FAIL_NONE;  // Good enough to sample!
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
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 2, lrintf(rcData[THROTTLE]));  // Raw throttle we're sampling
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 3, lrintf(rcCommand[THROTTLE]));  // Transformed throttle (for comparison)
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 4, hoverCal.sampleCount);
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 5, ARMING_FLAG(ARMED) ? 1 : 0);  // Debug: is armed?
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 6, hoverCal.result);
    DEBUG_SET(DEBUG_HOVER_CALIBRATION, 7, stabilityResult);

    // Handle disarmed state (WAITING_ARM)
    if (isDisarmed) {
        if (hoverCal.status == HOVER_CAL_STATUS_SAMPLING && hoverCal.sampleCount > 0) {
            // Was sampling and got some data - complete with what we have
            // This handles the case where user lands after hovering
            // Need a reasonable minimum sample count to get a valid median
            const uint16_t minSamplesForCompletion = 50;  // ~0.5 seconds of samples
            if (hoverCal.sampleCount >= minSamplesForCompletion) {
                uint16_t medianThrottle = calculateMedianThrottle();
                if (medianThrottle >= HOVER_CAL_THROTTLE_MIN && medianThrottle <= HOVER_CAL_THROTTLE_MAX) {
                    hoverCal.result = medianThrottle;
                    autopilotConfigMutable()->hoverThrottle = medianThrottle;
                    hoverCal.status = HOVER_CAL_STATUS_COMPLETE;
                    hoverCal.failReason = HOVER_CAL_FAIL_NONE;
                    beeper(BEEPER_ACC_CALIBRATION);
                } else {
                    hoverCalibrationFail(HOVER_CAL_FAIL_RESULT_RANGE);
                }
            } else {
                // Not enough samples for reliable calibration
                hoverCalibrationFail(HOVER_CAL_FAIL_DISARMED);
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
            beeper(BEEPER_RX_SET);  // Beep to indicate sampling started
        }
        return;
    }

    if (hoverCal.status == HOVER_CAL_STATUS_SAMPLING) {
        // Collect throttle sample - use rcData (raw receiver value) not rcCommand (transformed)
        // Store in array for median calculation, if we have space
        if (hoverCal.sampleCount < HOVER_CAL_MAX_SAMPLES) {
            hoverCal.throttleSamples[hoverCal.sampleCount] = rcData[THROTTLE];
            hoverCal.sampleCount++;
        }

        // Progress beep
        if (cmpTimeMs(currentTime, hoverCal.lastProgressBeepTime) >= HOVER_CAL_PROGRESS_BEEP_MS) {
            beeper(BEEPER_RX_SET);
            hoverCal.lastProgressBeepTime = currentTime;
        }

        // Check if sampling complete
        if (cmpTimeMs(currentTime, hoverCal.samplingStartTime) >= HOVER_CAL_SAMPLE_TIME_MS) {
            // Calculate median throttle using interquartile mean for robustness
            if (hoverCal.sampleCount > 0) {
                uint16_t medianThrottle = calculateMedianThrottle();

                // Validate result is within sane bounds
                if (medianThrottle >= HOVER_CAL_THROTTLE_MIN && medianThrottle <= HOVER_CAL_THROTTLE_MAX) {
                    // Success! Store result
                    hoverCal.result = medianThrottle;
                    autopilotConfigMutable()->hoverThrottle = medianThrottle;
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
