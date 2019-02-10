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
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "drivers/time.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/pid.h"
#include "pg/rx.h"
#include "rx/rx.h"


#include "sensors/battery.h"

typedef float (applyRatesFn)(const int axis, float rcCommandf, const float rcCommandfAbs);

static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3];
static float throttlePIDAttenuation;
static bool reverseMotors = false;
static applyRatesFn *applyRates;
uint16_t currentRxRefreshRate;

FAST_RAM_ZERO_INIT uint8_t interpolationChannels;

enum {
    ROLL_FLAG = 1 << ROLL,
    PITCH_FLAG = 1 << PITCH,
    YAW_FLAG = 1 << YAW,
    THROTTLE_FLAG = 1 << THROTTLE,
};

#ifdef USE_RC_SMOOTHING_FILTER
#define RC_SMOOTHING_IDENTITY_FREQUENCY         80    // Used in the formula to convert a BIQUAD cutoff frequency to PT1
#define RC_SMOOTHING_FILTER_STARTUP_DELAY_MS    5000  // Time to wait after power to let the PID loop stabilize before starting average frame rate calculation
#define RC_SMOOTHING_FILTER_TRAINING_SAMPLES    50    // Number of rx frame rate samples to average during initial training
#define RC_SMOOTHING_FILTER_RETRAINING_SAMPLES  20    // Number of rx frame rate samples to average during frame rate changes
#define RC_SMOOTHING_FILTER_TRAINING_DELAY_MS   1000  // Additional time to wait after receiving first valid rx frame before initial training starts
#define RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS 2000  // Guard time to wait after retraining to prevent retraining again too quickly
#define RC_SMOOTHING_RX_RATE_CHANGE_PERCENT     20    // Look for samples varying this much from the current detected frame rate to initiate retraining
#define RC_SMOOTHING_RX_RATE_MIN_US             1000  // 1ms
#define RC_SMOOTHING_RX_RATE_MAX_US             50000 // 50ms or 20hz

static FAST_RAM_ZERO_INIT rcSmoothingFilter_t rcSmoothingData;
#endif // USE_RC_SMOOTHING_FILTER

float getSetpointRate(int axis)
{
    return setpointRate[axis];
}

float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}

float getRcDeflectionAbs(int axis)
{
    return rcDeflectionAbs[axis];
}

float getThrottlePIDAttenuation(void)
{
    return throttlePIDAttenuation;
}

#define THROTTLE_LOOKUP_LENGTH 12
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE

static int16_t rcLookupThrottle(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
    return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;
}

#define SETPOINT_RATE_LIMIT 1998
STATIC_ASSERT(CONTROL_RATE_CONFIG_RATE_LIMIT_MAX <= SETPOINT_RATE_LIMIT, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX_too_large);

#define RC_RATE_INCREMENTAL 14.54f

float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    if (currentControlRateProfile->rcExpo[axis]) {
        const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    }

    float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
    if (rcRate > 2.0f) {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    }
    float angleRate = 200.0f * rcRate * rcCommandf;
    if (currentControlRateProfile->rates[axis]) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }

    return angleRate;
}

float applyRaceFlightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    // -1.0 to 1.0 ranged and curved
    rcCommandf = ((1.0f + 0.01f * currentControlRateProfile->rcExpo[axis] * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);
    // convert to -2000 to 2000 range using acro+ modifier
    float angleRate = 10.0f * currentControlRateProfile->rcRates[axis] * rcCommandf;
    angleRate = angleRate * (1 + rcCommandfAbs * (float)currentControlRateProfile->rates[axis] * 0.01f);

    return angleRate;
}

static void calculateSetpointRate(int axis)
{
    float angleRate;
    
#ifdef USE_GPS_RESCUE
    if ((axis == FD_YAW) && FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // If GPS Rescue is active then override the setpointRate used in the
        // pid controller with the value calculated from the desired heading logic.
        angleRate = gpsRescueGetYawRate();

        // Treat the stick input as centered to avoid any stick deflection base modifications (like acceleration limit)
        rcDeflection[axis] = 0;
        rcDeflectionAbs[axis] = 0;
    } else
#endif
    {
        // scale rcCommandf to range [-1.0, 1.0]
        float rcCommandf = rcCommand[axis] / 500.0f;
        rcDeflection[axis] = rcCommandf;
        const float rcCommandfAbs = fabsf(rcCommandf);
        rcDeflectionAbs[axis] = rcCommandfAbs;

        angleRate = applyRates(axis, rcCommandf, rcCommandfAbs);
    }
    // Rate limit from profile (deg/sec)
    setpointRate[axis] = constrainf(angleRate, -1.0f * currentControlRateProfile->rate_limit[axis], 1.0f * currentControlRateProfile->rate_limit[axis]);

    DEBUG_SET(DEBUG_ANGLERATE, axis, angleRate);
}

static void scaleRcCommandToFpvCamAngle(void)
{
    //recalculate sin/cos only when rxConfig()->fpvCamAngleDegrees changed
    static uint8_t lastFpvCamAngleDegrees = 0;
    static float cosFactor = 1.0;
    static float sinFactor = 0.0;

    if (lastFpvCamAngleDegrees != rxConfig()->fpvCamAngleDegrees) {
        lastFpvCamAngleDegrees = rxConfig()->fpvCamAngleDegrees;
        cosFactor = cos_approx(rxConfig()->fpvCamAngleDegrees * RAD);
        sinFactor = sin_approx(rxConfig()->fpvCamAngleDegrees * RAD);
    }

    float roll = setpointRate[ROLL];
    float yaw = setpointRate[YAW];
    setpointRate[ROLL] = constrainf(roll * cosFactor -  yaw * sinFactor, -SETPOINT_RATE_LIMIT * 1.0f, SETPOINT_RATE_LIMIT * 1.0f);
    setpointRate[YAW]  = constrainf(yaw  * cosFactor + roll * sinFactor, -SETPOINT_RATE_LIMIT * 1.0f, SETPOINT_RATE_LIMIT * 1.0f);
}

#define THROTTLE_BUFFER_MAX 20
#define THROTTLE_DELTA_MS 100

static void checkForThrottleErrorResetState(uint16_t rxRefreshRate)
{
    static int index;
    static int16_t rcCommandThrottlePrevious[THROTTLE_BUFFER_MAX];

    const int rxRefreshRateMs = rxRefreshRate / 1000;
    const int indexMax = constrain(THROTTLE_DELTA_MS / rxRefreshRateMs, 1, THROTTLE_BUFFER_MAX);
    const int16_t throttleVelocityThreshold = (featureIsEnabled(FEATURE_3D)) ? currentPidProfile->itermThrottleThreshold / 2 : currentPidProfile->itermThrottleThreshold;

    rcCommandThrottlePrevious[index++] = rcCommand[THROTTLE];
    if (index >= indexMax) {
        index = 0;
    }

    const int16_t rcCommandSpeed = rcCommand[THROTTLE] - rcCommandThrottlePrevious[index];

    if (currentPidProfile->antiGravityMode == ANTI_GRAVITY_STEP) {
        if (ABS(rcCommandSpeed) > throttleVelocityThreshold) {
            pidSetItermAccelerator(CONVERT_PARAMETER_TO_FLOAT(currentPidProfile->itermAcceleratorGain));
        } else {
            pidSetItermAccelerator(1.0f);
        }
    }
}

FAST_CODE uint8_t processRcInterpolation(void)
{
    static FAST_RAM_ZERO_INIT float rcCommandInterp[4];
    static FAST_RAM_ZERO_INIT float rcStepSize[4];
    static FAST_RAM_ZERO_INIT int16_t rcInterpolationStepCount;

    uint16_t rxRefreshRate;
    uint8_t updatedChannel = 0;

    if (rxConfig()->rcInterpolation) {
         // Set RC refresh rate for sampling and channels to filter
        switch (rxConfig()->rcInterpolation) {
        case RC_SMOOTHING_AUTO:
            rxRefreshRate = currentRxRefreshRate + 1000; // Add slight overhead to prevent ramps
            break;
        case RC_SMOOTHING_MANUAL:
            rxRefreshRate = 1000 * rxConfig()->rcInterpolationInterval;
            break;
        case RC_SMOOTHING_OFF:
        case RC_SMOOTHING_DEFAULT:
        default:
            rxRefreshRate = rxGetRefreshRate();
        }

        if (isRXDataNew && rxRefreshRate > 0) {
            rcInterpolationStepCount = rxRefreshRate / targetPidLooptime;

            for (int channel = 0; channel < PRIMARY_CHANNEL_COUNT; channel++) {
                if ((1 << channel) & interpolationChannels) {
                    rcStepSize[channel] = (rcCommand[channel] - rcCommandInterp[channel]) / (float)rcInterpolationStepCount;
                }
            }

           DEBUG_SET(DEBUG_RC_INTERPOLATION, 0, lrintf(rcCommand[0]));
           DEBUG_SET(DEBUG_RC_INTERPOLATION, 1, lrintf(currentRxRefreshRate / 1000));
        } else {
            rcInterpolationStepCount--;
        }

        // Interpolate steps of rcCommand
        if (rcInterpolationStepCount > 0) {
            for (updatedChannel = 0; updatedChannel < PRIMARY_CHANNEL_COUNT; updatedChannel++) {
                if ((1 << updatedChannel) & interpolationChannels) {
                    rcCommandInterp[updatedChannel] += rcStepSize[updatedChannel];
                    rcCommand[updatedChannel] = rcCommandInterp[updatedChannel];
                }
            }
        }
    } else {
        rcInterpolationStepCount = 0; // reset factor in case of level modes flip flopping
    }

    DEBUG_SET(DEBUG_RC_INTERPOLATION, 2, rcInterpolationStepCount);

    return updatedChannel;

}

#ifdef USE_RC_SMOOTHING_FILTER
// Determine a cutoff frequency based on filter type and the calculated
// average rx frame time
FAST_CODE_NOINLINE int calcRcSmoothingCutoff(int avgRxFrameTimeUs, bool pt1)
{
    if (avgRxFrameTimeUs > 0) {
        const float cutoffFactor = (100 - rxConfig()->rc_smoothing_auto_factor) / 100.0f;
        float cutoff = (1 / (avgRxFrameTimeUs * 1e-6f)) / 2;  // calculate the nyquist frequency
        cutoff = cutoff * cutoffFactor;

        if (pt1) {
            cutoff = sq(cutoff) / RC_SMOOTHING_IDENTITY_FREQUENCY; // convert to a cutoff for pt1 that has similar characteristics
        }
        return lrintf(cutoff);
    } else {
        return 0;
    }
}

// Preforms a reasonableness check on the rx frame time to avoid bad data
// skewing the average.
FAST_CODE bool rcSmoothingRxRateValid(int currentRxRefreshRate)
{
    return (currentRxRefreshRate >= RC_SMOOTHING_RX_RATE_MIN_US && currentRxRefreshRate <= RC_SMOOTHING_RX_RATE_MAX_US);
}

// Initialize or update the filters base on either the manually selected cutoff, or
// the auto-calculated cutoff frequency based on detected rx frame rate.
FAST_CODE_NOINLINE void rcSmoothingSetFilterCutoffs(rcSmoothingFilter_t *smoothingData)
{
    const float dT = targetPidLooptime * 1e-6f;
    uint16_t oldCutoff = smoothingData->inputCutoffFrequency;
    
    if (rxConfig()->rc_smoothing_input_cutoff == 0) {
        smoothingData->inputCutoffFrequency = calcRcSmoothingCutoff(smoothingData->averageFrameTimeUs, (rxConfig()->rc_smoothing_input_type == RC_SMOOTHING_INPUT_PT1));
    }

    // initialize or update the input filter
    if ((smoothingData->inputCutoffFrequency != oldCutoff) || !smoothingData->filterInitialized) {
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            if ((1 << i) & interpolationChannels) {  // only update channels specified by rc_interp_ch
                switch (rxConfig()->rc_smoothing_input_type) {
                
                    case RC_SMOOTHING_INPUT_PT1:
                        if (!smoothingData->filterInitialized) {
                            pt1FilterInit((pt1Filter_t*) &smoothingData->filter[i], pt1FilterGain(smoothingData->inputCutoffFrequency, dT));
                        } else {
                            pt1FilterUpdateCutoff((pt1Filter_t*) &smoothingData->filter[i], pt1FilterGain(smoothingData->inputCutoffFrequency, dT));
                        }
                        break;
                        
                    case RC_SMOOTHING_INPUT_BIQUAD:
                    default:
                        if (!smoothingData->filterInitialized) {
                            biquadFilterInitLPF((biquadFilter_t*) &smoothingData->filter[i], smoothingData->inputCutoffFrequency, targetPidLooptime);
                        } else {
                            biquadFilterUpdateLPF((biquadFilter_t*) &smoothingData->filter[i], smoothingData->inputCutoffFrequency, targetPidLooptime);
                        }
                        break;
                }
            }
        }
    }

    // update or initialize the derivative filter
    oldCutoff = smoothingData->derivativeCutoffFrequency;
    if ((rxConfig()->rc_smoothing_derivative_cutoff == 0) && (rxConfig()->rc_smoothing_derivative_type != RC_SMOOTHING_DERIVATIVE_OFF)) {
        smoothingData->derivativeCutoffFrequency = calcRcSmoothingCutoff(smoothingData->averageFrameTimeUs, (rxConfig()->rc_smoothing_derivative_type == RC_SMOOTHING_DERIVATIVE_PT1));
    }

    if (!smoothingData->filterInitialized) {
        pidInitSetpointDerivativeLpf(smoothingData->derivativeCutoffFrequency, rxConfig()->rc_smoothing_debug_axis, rxConfig()->rc_smoothing_derivative_type);
    } else if (smoothingData->derivativeCutoffFrequency != oldCutoff) {
        pidUpdateSetpointDerivativeLpf(smoothingData->derivativeCutoffFrequency);
    }
}

FAST_CODE_NOINLINE void rcSmoothingResetAccumulation(rcSmoothingFilter_t *smoothingData)
{
    smoothingData->training.sum = 0;
    smoothingData->training.count = 0;
    smoothingData->training.min = UINT16_MAX;
    smoothingData->training.max = 0;
}

// Accumulate the rx frame time samples. Once we've collected enough samples calculate the
// average and return true.
FAST_CODE bool rcSmoothingAccumulateSample(rcSmoothingFilter_t *smoothingData, int rxFrameTimeUs)
{
    smoothingData->training.sum += rxFrameTimeUs;
    smoothingData->training.count++;
    smoothingData->training.max = MAX(smoothingData->training.max, rxFrameTimeUs);
    smoothingData->training.min = MIN(smoothingData->training.min, rxFrameTimeUs);

    // if we've collected enough samples then calculate the average and reset the accumulation
    const int sampleLimit = (rcSmoothingData.filterInitialized) ? RC_SMOOTHING_FILTER_RETRAINING_SAMPLES : RC_SMOOTHING_FILTER_TRAINING_SAMPLES;
    if (smoothingData->training.count >= sampleLimit) {
        smoothingData->training.sum = smoothingData->training.sum - smoothingData->training.min - smoothingData->training.max; // Throw out high and low samples
        smoothingData->averageFrameTimeUs = lrintf(smoothingData->training.sum / (smoothingData->training.count - 2));
        rcSmoothingResetAccumulation(smoothingData);
        return true;
    }
    return false;
}

// Determine if we need to caclulate filter cutoffs. If not then we can avoid
// examining the rx frame times completely 
FAST_CODE_NOINLINE bool rcSmoothingAutoCalculate(void)
{
    bool ret = false;

    // if the input cutoff is 0 (auto) then we need to calculate cutoffs
    if (rxConfig()->rc_smoothing_input_cutoff == 0) {
        ret = true;
    }

    // if the derivative type isn't OFF and the cutoff is 0 then we need to calculate
    if (rxConfig()->rc_smoothing_derivative_type != RC_SMOOTHING_DERIVATIVE_OFF) {
        if (rxConfig()->rc_smoothing_derivative_cutoff == 0) {
            ret = true;
        }
    }
    return ret;
}

FAST_CODE uint8_t processRcSmoothingFilter(void)
{
    uint8_t updatedChannel = 0;
    static FAST_RAM_ZERO_INIT float lastRxData[4];
    static FAST_RAM_ZERO_INIT bool initialized;
    static FAST_RAM_ZERO_INIT timeMs_t validRxFrameTimeMs;
    static FAST_RAM_ZERO_INIT bool calculateCutoffs;

    // first call initialization
    if (!initialized) {
        initialized = true;
        rcSmoothingData.filterInitialized = false;
        rcSmoothingData.averageFrameTimeUs = 0;
        rcSmoothingResetAccumulation(&rcSmoothingData);
        
        rcSmoothingData.inputCutoffFrequency = rxConfig()->rc_smoothing_input_cutoff;
        
        if (rxConfig()->rc_smoothing_derivative_type != RC_SMOOTHING_DERIVATIVE_OFF) {
            rcSmoothingData.derivativeCutoffFrequency = rxConfig()->rc_smoothing_derivative_cutoff;
        }
        
        calculateCutoffs = rcSmoothingAutoCalculate();

        // if we don't need to calculate cutoffs dynamically then the filters can be initialized now
        if (!calculateCutoffs) {
            rcSmoothingSetFilterCutoffs(&rcSmoothingData);
            rcSmoothingData.filterInitialized = true;
        }
    }

    if (isRXDataNew) {

        // store the new raw channel values
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            if ((1 << i) & interpolationChannels) {
                lastRxData[i] = rcCommand[i];
            }
        }

        // for dynamically calculated filters we need to examine each rx frame interval
        if (calculateCutoffs) {
            const timeMs_t currentTimeMs = millis();
            int sampleState = 0;

            // If the filter cutoffs are set to auto and we have good rx data, then determine the average rx frame rate
            // and use that to calculate the filter cutoff frequencies
            if ((currentTimeMs > RC_SMOOTHING_FILTER_STARTUP_DELAY_MS) && (targetPidLooptime > 0)) { // skip during FC initialization
                if (rxIsReceivingSignal()  && rcSmoothingRxRateValid(currentRxRefreshRate)) {

                    // set the guard time expiration if it's not set
                    if (validRxFrameTimeMs == 0) {
                        validRxFrameTimeMs = currentTimeMs + (rcSmoothingData.filterInitialized ? RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS : RC_SMOOTHING_FILTER_TRAINING_DELAY_MS);
                    } else {
                        sampleState = 1;
                    }

                    // if the guard time has expired then process the rx frame time
                    if (currentTimeMs > validRxFrameTimeMs) {
                        sampleState = 2;
                        bool accumulateSample = true;

                        // During initial training process all samples.
                        // During retraining check samples to determine if they vary by more than the limit percentage.
                        if (rcSmoothingData.filterInitialized) {
                            const float percentChange = (ABS(currentRxRefreshRate - rcSmoothingData.averageFrameTimeUs) / (float)rcSmoothingData.averageFrameTimeUs) * 100;
                            if (percentChange < RC_SMOOTHING_RX_RATE_CHANGE_PERCENT) {
                                // We received a sample that wasn't more than the limit percent so reset the accumulation
                                // During retraining we need a contiguous block of samples that are all significantly different than the current average
                                rcSmoothingResetAccumulation(&rcSmoothingData);
                                accumulateSample = false;
                            }
                        }

                        // accumlate the sample into the average
                        if (accumulateSample) {
                            if (rcSmoothingAccumulateSample(&rcSmoothingData, currentRxRefreshRate)) {
                                // the required number of samples were collected so set the filter cutoffs
                                rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                                rcSmoothingData.filterInitialized = true;
                                validRxFrameTimeMs = 0;
                            }
                        }

                    }
                } else {
                    // we have either stopped receiving rx samples (failsafe?) or the sample time is unreasonable so reset the accumulation
                    rcSmoothingResetAccumulation(&rcSmoothingData);
                }
            }

            // rx frame rate training blackbox debugging
            if (debugMode == DEBUG_RC_SMOOTHING_RATE) {
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 0, currentRxRefreshRate);              // log each rx frame interval
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 1, rcSmoothingData.training.count);    // log the training step count
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 2, rcSmoothingData.averageFrameTimeUs);// the current calculated average
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 3, sampleState);                       // indicates whether guard time is active
            }
        }
    }

    if (rcSmoothingData.filterInitialized && (debugMode == DEBUG_RC_SMOOTHING)) {
        // after training has completed then log the raw rc channel and the calculated
        // average rx frame rate that was used to calculate the automatic filter cutoffs
        DEBUG_SET(DEBUG_RC_SMOOTHING, 0, lrintf(lastRxData[rxConfig()->rc_smoothing_debug_axis]));
        DEBUG_SET(DEBUG_RC_SMOOTHING, 3, rcSmoothingData.averageFrameTimeUs);
    }

    // each pid loop continue to apply the last received channel value to the filter
    for (updatedChannel = 0; updatedChannel < PRIMARY_CHANNEL_COUNT; updatedChannel++) {
        if ((1 << updatedChannel) & interpolationChannels) {  // only smooth selected channels base on the rc_interp_ch value
            if (rcSmoothingData.filterInitialized) {
                switch (rxConfig()->rc_smoothing_input_type) {
                    case RC_SMOOTHING_INPUT_PT1:
                        rcCommand[updatedChannel] = pt1FilterApply((pt1Filter_t*) &rcSmoothingData.filter[updatedChannel], lastRxData[updatedChannel]);
                        break;

                    case RC_SMOOTHING_INPUT_BIQUAD:
                    default:
                        rcCommand[updatedChannel] = biquadFilterApplyDF1((biquadFilter_t*) &rcSmoothingData.filter[updatedChannel], lastRxData[updatedChannel]);
                        break;
                }
            } else {
                // If filter isn't initialized yet then use the actual unsmoothed rx channel data
                rcCommand[updatedChannel] = lastRxData[updatedChannel];
            }
        }
    }

    return interpolationChannels;
}
#endif // USE_RC_SMOOTHING_FILTER

FAST_CODE void processRcCommand(void)
{
    uint8_t updatedChannel;

    if (isRXDataNew && pidAntiGravityEnabled()) {
        checkForThrottleErrorResetState(currentRxRefreshRate);
    }

    switch (rxConfig()->rc_smoothing_type) {
#ifdef USE_RC_SMOOTHING_FILTER
    case RC_SMOOTHING_TYPE_FILTER:
        updatedChannel = processRcSmoothingFilter();
        break;
#endif // USE_RC_SMOOTHING_FILTER
    case RC_SMOOTHING_TYPE_INTERPOLATION:
    default:
        updatedChannel = processRcInterpolation();
        break;
    }

    if (isRXDataNew || updatedChannel) {
        const uint8_t maxUpdatedAxis = isRXDataNew ? FD_YAW : MIN(updatedChannel, FD_YAW); // throttle channel doesn't require rate calculation
#if defined(SIMULATOR_BUILD)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunsafe-loop-optimizations"
#endif
        for (int axis = FD_ROLL; axis <= maxUpdatedAxis; axis++) {
#if defined(SIMULATOR_BUILD)
#pragma GCC diagnostic pop
#endif
            calculateSetpointRate(axis);
        }

        DEBUG_SET(DEBUG_RC_INTERPOLATION, 3, setpointRate[0]);

        // Scaling of AngleRate to camera angle (Mixing Roll and Yaw)
        if (rxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !FLIGHT_MODE(HEADFREE_MODE)) {
            scaleRcCommandToFpvCamAngle();
        }
    }

    if (isRXDataNew) {
        isRXDataNew = false;
    }
}

FAST_CODE FAST_CODE_NOINLINE void updateRcCommands(void)
{
    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    int32_t prop;
    if (rcData[THROTTLE] < currentControlRateProfile->tpa_breakpoint) {
        prop = 100;
        throttlePIDAttenuation = 1.0f;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
        } else {
            prop = 100 - currentControlRateProfile->dynThrPID;
        }
        throttlePIDAttenuation = prop / 100.0f;
    }

    for (int axis = 0; axis < 3; axis++) {
        // non coupled PID reduction scaler used in PID controller 1 and PID controller 2.

        int32_t tmp = MIN(ABS(rcData[axis] - rxConfig()->midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (tmp > rcControlsConfig()->deadband) {
                tmp -= rcControlsConfig()->deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp;
        } else {
            if (tmp > rcControlsConfig()->yaw_deadband) {
                tmp -= rcControlsConfig()->yaw_deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp * -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
        }
        if (rcData[axis] < rxConfig()->midrc) {
            rcCommand[axis] = -rcCommand[axis];
        }
    }

    int32_t tmp;
    if (featureIsEnabled(FEATURE_3D)) {
        tmp = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);
        tmp = (uint32_t)(tmp - PWM_RANGE_MIN);
    } else {
        tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
        tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);
    }

    if (getLowVoltageCutoff()->enabled) {
        tmp = tmp * getLowVoltageCutoff()->percentage / 100;
    }

    rcCommand[THROTTLE] = rcLookupThrottle(tmp);

    if (featureIsEnabled(FEATURE_3D) && !failsafeIsActive()) {
        if (!flight3DConfig()->switched_mode3d) {
            if (IS_RC_MODE_ACTIVE(BOX3D)) {
                fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
                rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MAX - rxConfig()->midrc);
            }
        } else {
            if (IS_RC_MODE_ACTIVE(BOX3D)) {
                reverseMotors = true;
                fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
                rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MIN - rxConfig()->midrc);
            } else {
                reverseMotors = false;
                fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
                rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MAX - rxConfig()->midrc);
            }
        }
    }
    if (FLIGHT_MODE(HEADFREE_MODE)) {
        static t_fp_vector_def  rcCommandBuff;

        rcCommandBuff.X = rcCommand[ROLL];
        rcCommandBuff.Y = rcCommand[PITCH];
        if ((!FLIGHT_MODE(ANGLE_MODE) && (!FLIGHT_MODE(HORIZON_MODE)) && (!FLIGHT_MODE(GPS_RESCUE_MODE)))) {
            rcCommandBuff.Z = rcCommand[YAW];
        } else {
            rcCommandBuff.Z = 0;
        }
        imuQuaternionHeadfreeTransformVectorEarthToBody(&rcCommandBuff);
        rcCommand[ROLL] = rcCommandBuff.X;
        rcCommand[PITCH] = rcCommandBuff.Y;
        if ((!FLIGHT_MODE(ANGLE_MODE)&&(!FLIGHT_MODE(HORIZON_MODE)) && (!FLIGHT_MODE(GPS_RESCUE_MODE)))) {
            rcCommand[YAW] = rcCommandBuff.Z;
        }
    }
}

void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    setpointRate[YAW] = 0;
}

bool isMotorsReversed(void)
{
    return reverseMotors;
}

void initRcProcessing(void)
{
    for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        const int16_t tmp = 10 * i - currentControlRateProfile->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - currentControlRateProfile->thrMid8;
        if (tmp < 0)
            y = currentControlRateProfile->thrMid8;
        lookupThrottleRC[i] = 10 * currentControlRateProfile->thrMid8 + tmp * (100 - currentControlRateProfile->thrExpo8 + (int32_t) currentControlRateProfile->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = PWM_RANGE_MIN + (PWM_RANGE_MAX - PWM_RANGE_MIN) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }

    switch (currentControlRateProfile->rates_type) {
    case RATES_TYPE_BETAFLIGHT:
    default:
        applyRates = applyBetaflightRates;

        break;
    case RATES_TYPE_RACEFLIGHT:
        applyRates = applyRaceFlightRates;

        break;
    }

    interpolationChannels = 0;
    switch (rxConfig()->rcInterpolationChannels) {
    case INTERPOLATION_CHANNELS_RPYT:
        interpolationChannels |= THROTTLE_FLAG;

        FALLTHROUGH;
    case INTERPOLATION_CHANNELS_RPY:
        interpolationChannels |= YAW_FLAG;

        FALLTHROUGH;
    case INTERPOLATION_CHANNELS_RP:
        interpolationChannels |= ROLL_FLAG | PITCH_FLAG;

        break;
    case INTERPOLATION_CHANNELS_RPT:
        interpolationChannels |= ROLL_FLAG | PITCH_FLAG;

        FALLTHROUGH;
    case INTERPOLATION_CHANNELS_T:
        interpolationChannels |= THROTTLE_FLAG;

        break;
    }
}

bool rcSmoothingIsEnabled(void)
{
    return !(
#if defined(USE_RC_SMOOTHING_FILTER)
        rxConfig()->rc_smoothing_type == RC_SMOOTHING_TYPE_INTERPOLATION &&
#endif
        rxConfig()->rcInterpolation == RC_SMOOTHING_OFF);
}

#ifdef USE_RC_SMOOTHING_FILTER
int rcSmoothingGetValue(int whichValue)
{
    switch (whichValue) {
        case RC_SMOOTHING_VALUE_INPUT_ACTIVE:
            return rcSmoothingData.inputCutoffFrequency;
        case RC_SMOOTHING_VALUE_DERIVATIVE_ACTIVE:
            return rcSmoothingData.derivativeCutoffFrequency;
        case RC_SMOOTHING_VALUE_AVERAGE_FRAME:
            return rcSmoothingData.averageFrameTimeUs;
        default:
            return 0;
    }
}

bool rcSmoothingInitializationComplete(void) {
    return (rxConfig()->rc_smoothing_type != RC_SMOOTHING_TYPE_FILTER) || rcSmoothingData.filterInitialized;
}
#endif // USE_RC_SMOOTHING_FILTER
