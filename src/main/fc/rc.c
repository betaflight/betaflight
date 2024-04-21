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

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/pid.h"
#include "flight/pid_init.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rc.h"


typedef float (applyRatesFn)(const int axis, float rcCommandf, const float rcCommandfAbs);
// note that rcCommand[] is an external float

static float rawSetpoint[XYZ_AXIS_COUNT];

static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3]; // deflection range -1 to 1
static float maxRcDeflectionAbs;
static bool reverseMotors = false;
static applyRatesFn *applyRates;

static uint16_t currentRxIntervalUs; // packet interval in microseconds
static float currentRxRateHz; // packet rate in hertz

static bool isRxDataNew = false;
static bool isRxIntervalValid = false;
static float rcCommandDivider = 500.0f;
static float rcCommandYawDivider = 500.0f;

enum {
    ROLL_FLAG = 1 << ROLL,
    PITCH_FLAG = 1 << PITCH,
    YAW_FLAG = 1 << YAW,
    THROTTLE_FLAG = 1 << THROTTLE,
};

#ifdef USE_FEEDFORWARD
static float feedforwardSmoothed[3];
static float feedforwardRaw[3];
typedef struct laggedMovingAverageCombined_s {
    laggedMovingAverage_t filter;
    float buf[4];
} laggedMovingAverageCombined_t;
laggedMovingAverageCombined_t  feedforwardDeltaAvg[XYZ_AXIS_COUNT];

float getFeedforward(int axis)
{
#ifdef USE_RC_SMOOTHING_FILTER
    return feedforwardSmoothed[axis];
#else
    return feedforwardRaw[axis];
#endif
}
#endif // USE_FEEDFORWARD

#ifdef USE_RC_SMOOTHING_FILTER
static FAST_DATA_ZERO_INIT rcSmoothingFilter_t rcSmoothingData;
static float rcDeflectionSmoothed[3];
#endif // USE_RC_SMOOTHING_FILTER

#define RX_INTERVAL_MIN_US     950 // 0.950ms to fit 1kHz without an issue
#define RX_INTERVAL_MAX_US   65500 // 65.5ms or 15.26hz

float getSetpointRate(int axis)
{
#ifdef USE_RC_SMOOTHING_FILTER
    return setpointRate[axis];
#else
    return rawSetpoint[axis];
#endif
}

static float maxRcRate[3];
float getMaxRcRate(int axis)
{
    return maxRcRate[axis];
}

float getRcDeflection(int axis)
{
#ifdef USE_RC_SMOOTHING_FILTER
    return rcDeflectionSmoothed[axis];
#else
    return rcDeflection[axis];
#endif
}

float getRcDeflectionRaw(int axis)
{
    return rcDeflection[axis];
}

float getRcDeflectionAbs(int axis)
{
    return rcDeflectionAbs[axis];
}

#define THROTTLE_LOOKUP_LENGTH 12
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE

static int16_t rcLookupThrottle(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
    return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;
}

#define SETPOINT_RATE_LIMIT_MIN -1998.0f
#define SETPOINT_RATE_LIMIT_MAX 1998.0f
STATIC_ASSERT(CONTROL_RATE_CONFIG_RATE_LIMIT_MAX <= (uint16_t)SETPOINT_RATE_LIMIT_MAX, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX_too_large);

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

float applyKissRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    const float rcCurvef = currentControlRateProfile->rcExpo[axis] / 100.0f;

    float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
    float kissRcCommandf = (power3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (currentControlRateProfile->rcRates[axis] / 1000.0f);
    float kissAngle = constrainf(((2000.0f * kissRpyUseRates) * kissRcCommandf), SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);

    return kissAngle;
}

float applyActualRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    expof = rcCommandfAbs * (power5(rcCommandf) * expof + rcCommandf * (1 - expof));

    const float centerSensitivity = currentControlRateProfile->rcRates[axis] * 10.0f;
    const float stickMovement = MAX(0, currentControlRateProfile->rates[axis] * 10.0f - centerSensitivity);
    const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;

    return angleRate;
}

float applyQuickRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    const uint16_t rcRate = currentControlRateProfile->rcRates[axis] * 2;
    const uint16_t maxDPS = MAX(currentControlRateProfile->rates[axis] * 10, rcRate);
    const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    const float superFactorConfig = ((float)maxDPS / rcRate - 1) / ((float)maxDPS / rcRate);

    float curve;
    float superFactor;
    float angleRate;

    if (currentControlRateProfile->quickRatesRcExpo) {
        curve = power3(rcCommandf) * expof + rcCommandf * (1 - expof);
        superFactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superFactorConfig), 0.01f, 1.00f));
        angleRate = constrainf(curve * rcRate * superFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
    } else {
        curve = power3(rcCommandfAbs) * expof + rcCommandfAbs * (1 - expof);
        superFactor = 1.0f / (constrainf(1.0f - (curve * superFactorConfig), 0.01f, 1.00f));
        angleRate = constrainf(rcCommandf * rcRate * superFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
    }

    return angleRate;
}

static void scaleRawSetpointToFpvCamAngle(void)
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

    float roll = rawSetpoint[ROLL];
    float yaw = rawSetpoint[YAW];
    rawSetpoint[ROLL] = constrainf(roll * cosFactor -  yaw * sinFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
    rawSetpoint[YAW]  = constrainf(yaw  * cosFactor + roll * sinFactor, SETPOINT_RATE_LIMIT_MIN, SETPOINT_RATE_LIMIT_MAX);
}

void updateRcRefreshRate(timeUs_t currentTimeUs)
{
    static timeUs_t lastRxTimeUs;

    timeDelta_t frameAgeUs;
    timeDelta_t frameDeltaUs = rxGetFrameDelta(&frameAgeUs);

    if (!frameDeltaUs || cmpTimeUs(currentTimeUs, lastRxTimeUs) <= frameAgeUs) {
        frameDeltaUs = cmpTimeUs(currentTimeUs, lastRxTimeUs);
    }

    DEBUG_SET(DEBUG_RX_TIMING, 0, MIN(frameDeltaUs / 10, INT16_MAX));
    DEBUG_SET(DEBUG_RX_TIMING, 1, MIN(frameAgeUs / 10, INT16_MAX));

    lastRxTimeUs = currentTimeUs;
    currentRxIntervalUs = constrain(frameDeltaUs, RX_INTERVAL_MIN_US, RX_INTERVAL_MAX_US);
    isRxIntervalValid = frameDeltaUs == currentRxIntervalUs;

    currentRxRateHz = 1e6f / currentRxIntervalUs; // cannot be zero due to preceding constraint
    DEBUG_SET(DEBUG_RX_TIMING, 2, isRxIntervalValid);
    DEBUG_SET(DEBUG_RX_TIMING, 3, MIN(currentRxIntervalUs / 10, INT16_MAX));
}

uint16_t getCurrentRxIntervalUs(void)
{
    return currentRxIntervalUs;
}

#ifdef USE_RC_SMOOTHING_FILTER

// Initialize or update the filters base on either the manually selected cutoff, or
// the auto-calculated cutoff frequency based on detected rx frame rate.
FAST_CODE_NOINLINE void rcSmoothingSetFilterCutoffs(rcSmoothingFilter_t *smoothingData)
{
    // in auto mode, calculate the RC smoothing cutoff from the smoothed Rx link frequency
    const uint16_t oldSetpointCutoff = smoothingData->setpointCutoffFrequency;
    const uint16_t oldFeedforwardCutoff = smoothingData->feedforwardCutoffFrequency;
    const uint16_t minCutoffHz = 15; // don't let any RC smoothing filter cutoff go below 15Hz
    if (smoothingData->setpointCutoffSetting == 0) {
        smoothingData->setpointCutoffFrequency = MAX(minCutoffHz, (uint16_t)(smoothingData->smoothedRxRateHz * smoothingData->autoSmoothnessFactorSetpoint));
    }
    if (smoothingData->throttleCutoffSetting == 0) {
        smoothingData->throttleCutoffFrequency = MAX(minCutoffHz, (uint16_t)(smoothingData->smoothedRxRateHz * smoothingData->autoSmoothnessFactorThrottle));
    }

    if (smoothingData->feedforwardCutoffSetting == 0) {
        smoothingData->feedforwardCutoffFrequency = MAX(minCutoffHz, (uint16_t)(smoothingData->smoothedRxRateHz * smoothingData->autoSmoothnessFactorFeedforward));
    }

    const float dT = targetPidLooptime * 1e-6f;
    if ((smoothingData->setpointCutoffFrequency != oldSetpointCutoff) || !smoothingData->filterInitialized) {
        // note that cutoff frequencies are integers, filter cutoffs won't re-calculate until there is > 1hz variation from previous cutoff
        // initialize or update the setpoint cutoff based filters
        const float setpointCutoffFrequency = smoothingData->setpointCutoffFrequency;
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            if (i < THROTTLE) {
                if (!smoothingData->filterInitialized) {
                    pt3FilterInit(&smoothingData->filterSetpoint[i], pt3FilterGain(setpointCutoffFrequency, dT));
                } else {
                    pt3FilterUpdateCutoff(&smoothingData->filterSetpoint[i], pt3FilterGain(setpointCutoffFrequency, dT));
                }
            } else {
                const float throttleCutoffFrequency = smoothingData->throttleCutoffFrequency;
                if (!smoothingData->filterInitialized) {
                    pt3FilterInit(&smoothingData->filterSetpoint[i], pt3FilterGain(throttleCutoffFrequency, dT));
                } else {
                    pt3FilterUpdateCutoff(&smoothingData->filterSetpoint[i], pt3FilterGain(throttleCutoffFrequency, dT));
                }
            }
        }
        // initialize or update the RC Deflection filter
        for (int i = FD_ROLL; i < FD_YAW; i++) {
            if (!smoothingData->filterInitialized) {
                pt3FilterInit(&smoothingData->filterRcDeflection[i], pt3FilterGain(setpointCutoffFrequency, dT));
            } else {
                pt3FilterUpdateCutoff(&smoothingData->filterRcDeflection[i], pt3FilterGain(setpointCutoffFrequency, dT));
            }
        }
    }
    // initialize or update the Feedforward filter
    if ((smoothingData->feedforwardCutoffFrequency != oldFeedforwardCutoff) || !smoothingData->filterInitialized) {
       for (int i = FD_ROLL; i <= FD_YAW; i++) {
            const float feedforwardCutoffFrequency = smoothingData->feedforwardCutoffFrequency;
            if (!smoothingData->filterInitialized) {
                pt3FilterInit(&smoothingData->filterFeedforward[i], pt3FilterGain(feedforwardCutoffFrequency, dT));
            } else {
                pt3FilterUpdateCutoff(&smoothingData->filterFeedforward[i], pt3FilterGain(feedforwardCutoffFrequency, dT));
            }
        }
    }

    DEBUG_SET(DEBUG_RC_SMOOTHING, 1, smoothingData->setpointCutoffFrequency);
    DEBUG_SET(DEBUG_RC_SMOOTHING, 2, smoothingData->feedforwardCutoffFrequency);
}

// Determine if we need to caclulate filter cutoffs. If not then we can avoid
// examining the rx frame times completely
FAST_CODE_NOINLINE bool rcSmoothingAutoCalculate(void)
{
    // if any rc smoothing cutoff is 0 (auto) then we need to calculate cutoffs
    if ((rcSmoothingData.setpointCutoffSetting == 0) || (rcSmoothingData.feedforwardCutoffSetting == 0) || (rcSmoothingData.throttleCutoffSetting == 0)) {
        return true;
    }
    return false;
}

static FAST_CODE void processRcSmoothingFilter(void)
{
    static FAST_DATA_ZERO_INIT float rxDataToSmooth[4];
    static FAST_DATA_ZERO_INIT bool initialized;
    static FAST_DATA_ZERO_INIT bool calculateCutoffs;

    // first call initialization
    if (!initialized) {
        initialized = true;
        rcSmoothingData.filterInitialized = false;
        rcSmoothingData.smoothedRxRateHz = 0.0f;
        rcSmoothingData.sampleCount = 0;
        rcSmoothingData.debugAxis = rxConfig()->rc_smoothing_debug_axis;

        rcSmoothingData.autoSmoothnessFactorSetpoint = 1.5f / (1.0f + (rxConfig()->rc_smoothing_auto_factor_rpy / 10.0f));
        rcSmoothingData.autoSmoothnessFactorFeedforward = 1.5f / (1.0f + (rxConfig()->rc_smoothing_auto_factor_rpy / 10.0f));
        rcSmoothingData.autoSmoothnessFactorThrottle = 1.5f / (1.0f + (rxConfig()->rc_smoothing_auto_factor_throttle / 10.0f));

        rcSmoothingData.setpointCutoffSetting = rxConfig()->rc_smoothing_setpoint_cutoff;
        rcSmoothingData.throttleCutoffSetting = rxConfig()->rc_smoothing_throttle_cutoff;
        rcSmoothingData.feedforwardCutoffSetting = rxConfig()->rc_smoothing_feedforward_cutoff;

        rcSmoothingData.setpointCutoffFrequency = rcSmoothingData.setpointCutoffSetting;
        rcSmoothingData.feedforwardCutoffFrequency = rcSmoothingData.feedforwardCutoffSetting;
        rcSmoothingData.throttleCutoffFrequency = rcSmoothingData.throttleCutoffSetting;

        if (rxConfig()->rc_smoothing_mode) {
            calculateCutoffs = rcSmoothingAutoCalculate();
            // if we don't need to calculate cutoffs dynamically then the filters can be initialized now
            if (!calculateCutoffs) {
                rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                rcSmoothingData.filterInitialized = true;
            }
        }
    }

    if (isRxDataNew) {
        if (calculateCutoffs) {
            // for auto calculated filters, calculate the link interval and update the RC smoothing filters at regular intervals
            // this is more efficient than monitoring for significant changes and making comparisons to decide whether to update the filter
            const timeMs_t currentTimeMs = millis();
            int sampleState = 0;
            const bool ready = (currentTimeMs > 1000) && (targetPidLooptime > 0);
            if (ready) { // skip during FC initialization
                // Wait 1000ms after power to let the PID loop stabilize before starting average frame rate calculation
                if (rxIsReceivingSignal() && isRxIntervalValid) {

                    static uint16_t previousRxIntervalUs;
                    if (abs(currentRxIntervalUs - previousRxIntervalUs) < (previousRxIntervalUs - (previousRxIntervalUs / 8))) {
                        // exclude large steps, eg after dropouts or telemetry
                        // by using interval here, we catch a dropout/telemetry where the inteval increases by 100%, but accept
                        // the return to normal value, which is only 50% different from the 100% interval of a single drop, and 66% of a return after a double drop.
                        static float prevRxRateHz;
                        // smooth the current Rx link frequency estimates
                        const float kF = 0.1f; // first order lowpass smoothing filter coefficient
                        const float smoothedRxRateHz = prevRxRateHz + kF * (currentRxRateHz - prevRxRateHz);
                        prevRxRateHz = smoothedRxRateHz;
      
                        // recalculate cutoffs every 3 acceptable samples
                        if (rcSmoothingData.sampleCount) {
                            rcSmoothingData.sampleCount --;
                            sampleState = 1;
                        } else {
                            rcSmoothingData.smoothedRxRateHz = smoothedRxRateHz;
                            rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                            rcSmoothingData.filterInitialized = true;
                            rcSmoothingData.sampleCount = 3;
                            sampleState = 2;
                        }
                    }
                    previousRxIntervalUs = currentRxIntervalUs;
                } else {
                    // either we stopped receiving rx samples (failsafe?) or the sample interval is unreasonable
                    // require a full re-evaluation period after signal is restored
                    rcSmoothingData.sampleCount = 0;
                    sampleState = 4;
                }
            }
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 0, currentRxIntervalUs / 10);
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 1, rcSmoothingData.sampleCount);
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 2, rcSmoothingData.smoothedRxRateHz); // value used by filters
            DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 3, sampleState); // guard time = 1, guard time expired = 2
        }
        // Get new values to be smoothed
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            rxDataToSmooth[i] = i == THROTTLE ? rcCommand[i] : rawSetpoint[i];
            if (i < THROTTLE) {
                DEBUG_SET(DEBUG_RC_INTERPOLATION, i, lrintf(rxDataToSmooth[i]));
            } else {
                DEBUG_SET(DEBUG_RC_INTERPOLATION, i, ((lrintf(rxDataToSmooth[i])) - 1000));
            }
        }
    }

    DEBUG_SET(DEBUG_RC_SMOOTHING, 0, rcSmoothingData.smoothedRxRateHz);
    DEBUG_SET(DEBUG_RC_SMOOTHING, 3, rcSmoothingData.sampleCount);

    // each pid loop, apply the last received channel value to the filter, if initialised - thanks @klutvott
    for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
        float *dst = i == THROTTLE ? &rcCommand[i] : &setpointRate[i];
        if (rcSmoothingData.filterInitialized) {
            *dst = pt3FilterApply(&rcSmoothingData.filterSetpoint[i], rxDataToSmooth[i]);
        } else {
            // If filter isn't initialized yet, as in smoothing off, use the actual unsmoothed rx channel data
            *dst = rxDataToSmooth[i];
        }
    }

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        // Feedforward smoothing
        feedforwardSmoothed[axis] = pt3FilterApply(&rcSmoothingData.filterFeedforward[axis], feedforwardRaw[axis]);
        // Horizon mode smoothing of rcDeflection on pitch and roll to provide a smooth angle element
        const bool smoothRcDeflection = FLIGHT_MODE(HORIZON_MODE) && rcSmoothingData.filterInitialized;
        if (smoothRcDeflection && axis < FD_YAW) {
            rcDeflectionSmoothed[axis] = pt3FilterApply(&rcSmoothingData.filterRcDeflection[axis], rcDeflection[axis]);
        } else {
            rcDeflectionSmoothed[axis] = rcDeflection[axis];
        }
    }
}
#endif // USE_RC_SMOOTHING_FILTER

NOINLINE void initAveraging(uint16_t feedforwardAveraging)
{
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        laggedMovingAverageInit(&feedforwardDeltaAvg[i].filter, feedforwardAveraging + 1, (float *)&feedforwardDeltaAvg[i].buf[0]);
    }
}

FAST_CODE_NOINLINE void calculateFeedforward(const pidRuntime_t *pid, int axis)
{
    const float rxInterval = currentRxIntervalUs * 1e-6f; // seconds
    float rxRate = currentRxRateHz;
    static float prevRxInterval;

    static float prevRcCommand[3];
    static float prevRcCommandDeltaAbs[3];          // for duplicate interpolation
    static float prevSetpoint[3];                   // equals raw unless interpolated 
    static float prevSetpointSpeed[3];
    static float prevAcceleration[3];               // for duplicate interpolation
    static bool prevDuplicatePacket[3];             // to identify multiple identical packets
    static uint16_t feedforwardAveraging = 0;

    if (feedforwardAveraging != pid->feedforwardAveraging) {
        feedforwardAveraging = pid->feedforwardAveraging;
        initAveraging(feedforwardAveraging);
    }

    const float rcCommandDeltaAbs = fabsf(rcCommand[axis] - prevRcCommand[axis]);
    prevRcCommand[axis] = rcCommand[axis];

    const float setpoint = rawSetpoint[axis];
    float setpointSpeed = (setpoint - prevSetpoint[axis]);
    prevSetpoint[axis] = setpoint;

    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FEEDFORWARD, 1, lrintf(setpointSpeed * 0.1f));   // raw setpoint delta
    }

    float setpointAcceleration = 0.0f;
    float feedforward = 0.0f;
    float jitterAttenuator = 1.0f;
    float zeroTheAcceleration = 1.0f;

    // calculate jitterAttenuation factor
    if (pid->feedforwardJitterFactor) {
        if (rcCommandDeltaAbs < pid->feedforwardJitterFactor) {
            jitterAttenuator = MAX(1.0f - (rcCommandDeltaAbs + prevRcCommandDeltaAbs[axis]) * pid->feedforwardJitterFactorInv, 0.0f);
            // note that feedforwardJitterFactorInv includes a divide by 2 to average the two previous rcCommandDeltaAbs values
            jitterAttenuator = 1.0f - jitterAttenuator * jitterAttenuator;
        }
    }
    prevRcCommandDeltaAbs[axis] = rcCommandDeltaAbs;

    // interpolate if necessary to remove steps in setpointSpeed
    if (rcCommandDeltaAbs) {
        // movement!
        if (prevDuplicatePacket[axis] == true) {
            rxRate = 1.0f / (rxInterval + prevRxInterval);
            zeroTheAcceleration = 0.0f;
            // don't add acceleration, empirically seems better on FrSky
        }
        setpointSpeed *= rxRate;
        prevDuplicatePacket[axis] = false;
    } else {
        // no movement!
        if (prevDuplicatePacket[axis] == false) {
            // first duplicate after movement
            setpointSpeed = prevSetpointSpeed[axis];
            if (fabsf(setpoint) < 0.95f * maxRcRate[axis]) {
                setpointSpeed += prevAcceleration[axis];
            }
            zeroTheAcceleration = 0.0f; // force acceleration to zero
        } else {
            // second and subsequent duplicates after movement should be zeroed
            setpointSpeed = 0.0f;
            prevSetpointSpeed[axis] = 0.0f;
            zeroTheAcceleration = 0.0f; // force acceleration to zero
        }
        prevDuplicatePacket[axis] = true;
    }
    prevRxInterval = rxInterval;

    // smooth the setpointSpeed value
    setpointSpeed = prevSetpointSpeed[axis] + pid->feedforwardSmoothFactor * (setpointSpeed - prevSetpointSpeed[axis]);

    // calculate acceleration from smoothed setpointSpeed and attenuate
    setpointAcceleration = (setpointSpeed - prevSetpointSpeed[axis]) * rxRate * 0.01f;
    prevSetpointSpeed[axis] = setpointSpeed;

    // smooth the acceleration element (effectively a second order filter since incoming setpoint was smoothed)
    // apply jitter reduction to acceleration here, effectively twice on acceleration
    setpointAcceleration = prevAcceleration[axis] + pid->feedforwardSmoothFactor * (setpointAcceleration - prevAcceleration[axis]);
    prevAcceleration[axis] = setpointAcceleration * zeroTheAcceleration;
    setpointAcceleration = setpointAcceleration * pid->feedforwardBoostFactor * jitterAttenuator * zeroTheAcceleration;

    feedforward = setpointSpeed + setpointAcceleration;

    // apply jitter attenuation to classic feedforward elements only (twice on acceleaertion)
    feedforward *= jitterAttenuator;

    // apply feedforward transition to classic feedforward elements only
    const bool useTransition = (pid->feedforwardTransition != 0.0f) && (rcDeflectionAbs[axis] < pid->feedforwardTransition);
    if (useTransition) {
        feedforward *= rcDeflectionAbs[axis] * pid->feedforwardTransitionInv;
    }

    // apply max rate limiting, forcing to zero when sticks approach max, but not on yaw
    if (axis < FD_YAW) {
        if (pid->feedforwardMaxRateLimit) {
            if (feedforward * setpoint > 0.0f) { // in same direction
                const float limit = (maxRcRate[axis] - fabsf(setpoint)) * pid->feedforwardMaxRateLimit;
                feedforward = (limit > 0.0f) ? constrainf(feedforward, -limit, limit) : 0.0f;
            }
        }
    } else {
        // add high-pass filtered setpoint to feedforward for yaw
        // this is not a derivative, being proportional to setpoint, so does not require jitter reduction
        // it is not interpolated, so dropouts will cause flat spots, but no derivative drops to zero
        // this provides a sustained FF on yaw that mimics the normal yaw motor drive requirements
        static float prevSetpointYaw = 0.0f;
        // const of 7.0 to get decay roughly right
        const float setpointLpfYaw = prevSetpointYaw + 7.0f * rxInterval * (setpoint - prevSetpointYaw);
        prevSetpointYaw = setpointLpfYaw;
        // const of 20 to scale yaw setpoint factor to a decent number
        feedforward += 20.0f * (setpoint - setpointLpfYaw);
    }

    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FEEDFORWARD, 2, lrintf(feedforward * 0.1f));
        // feedforward including acceleration but before averaging
    }

    // apply averaging to final values, for additional smoothing if needed
    if (feedforwardAveraging) {
        feedforward = laggedMovingAverageUpdate(&feedforwardDeltaAvg[axis].filter, feedforward);
    }

    feedforwardRaw[axis] = feedforward;

    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FEEDFORWARD, 0, lrintf(setpoint * 10.0f)); // setpoint value used for FF
        DEBUG_SET(DEBUG_FEEDFORWARD, 3, lrintf(feedforwardRaw[axis] * 0.1f)); // final feedforward value
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 0, lrintf(jitterAttenuator * 100.0f)); // jitter attenuation factor in percent
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 1, lrintf(maxRcRate[axis])); // max RC rate
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 2, lrintf(setpoint)); // setpoint used for FF
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 3, lrintf(feedforwardRaw[axis])); // un-smoothed final feedforward
    }
}

FAST_CODE void processRcCommand(void)
{
    if (isRxDataNew) {
        maxRcDeflectionAbs = 0.0f;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {

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
                float rcCommandf;
                if (axis == FD_YAW) {
                    rcCommandf = rcCommand[axis] / rcCommandYawDivider;
                } else {
                    rcCommandf = rcCommand[axis] / rcCommandDivider;
                }

                rcDeflection[axis] = rcCommandf;
                const float rcCommandfAbs = fabsf(rcCommandf);
                rcDeflectionAbs[axis] = rcCommandfAbs;
                maxRcDeflectionAbs = fmaxf(maxRcDeflectionAbs, rcCommandfAbs);

                angleRate = applyRates(axis, rcCommandf, rcCommandfAbs);
            }


            rawSetpoint[axis] = constrainf(angleRate, -1.0f * currentControlRateProfile->rate_limit[axis], 1.0f * currentControlRateProfile->rate_limit[axis]);
            DEBUG_SET(DEBUG_ANGLERATE, axis, angleRate);

#ifdef USE_FEEDFORWARD
        calculateFeedforward(&pidRuntime, axis);
#endif // USE_FEEDFORWARD

        calcEzLandingLimit(maxRcDeflectionAbs);

        }
        // adjust unfiltered setpoint steps to camera angle (mixing Roll and Yaw)
        if (rxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !FLIGHT_MODE(HEADFREE_MODE)) {
            scaleRawSetpointToFpvCamAngle();
        }
    }

#ifdef USE_RC_SMOOTHING_FILTER
    processRcSmoothingFilter();
#endif

    isRxDataNew = false;
}

FAST_CODE_NOINLINE void updateRcCommands(void)
{
    isRxDataNew = true;

    for (int axis = 0; axis < 3; axis++) {
        // non coupled PID reduction scaler used in PID controller 1 and PID controller 2.

        float tmp = MIN(fabsf(rcData[axis] - rxConfig()->midrc), 500.0f);
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
    rcCommandDivider = 500.0f - rcControlsConfig()->deadband;
    rcCommandYawDivider = 500.0f - rcControlsConfig()->yaw_deadband;

    for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        const int16_t tmp = 10 * i - currentControlRateProfile->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - currentControlRateProfile->thrMid8;
        if (tmp < 0)
            y = currentControlRateProfile->thrMid8;
        lookupThrottleRC[i] = 10 * currentControlRateProfile->thrMid8 + tmp * (100 - currentControlRateProfile->thrExpo8 + (int32_t) currentControlRateProfile->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = PWM_RANGE_MIN + PWM_RANGE * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }

    switch (currentControlRateProfile->rates_type) {
    case RATES_TYPE_BETAFLIGHT:
    default:
        applyRates = applyBetaflightRates;
        break;
    case RATES_TYPE_RACEFLIGHT:
        applyRates = applyRaceFlightRates;
        break;
    case RATES_TYPE_KISS:
        applyRates = applyKissRates;
        break;
    case RATES_TYPE_ACTUAL:
        applyRates = applyActualRates;
        break;
    case RATES_TYPE_QUICK:
        applyRates = applyQuickRates;
        break;
    }

    for (int i = 0; i < 3; i++) {
        maxRcRate[i] = applyRates(i, 1.0f, 1.0f);
#ifdef USE_FEEDFORWARD
        feedforwardSmoothed[i] = 0.0f;
        feedforwardRaw[i] = 0.0f;
#endif // USE_FEEDFORWARD
    }

#ifdef USE_YAW_SPIN_RECOVERY
    const int maxYawRate = (int)maxRcRate[FD_YAW];
    initYawSpinRecovery(maxYawRate);
#endif
}

// send rc smoothing details to blackbox
#ifdef USE_RC_SMOOTHING_FILTER
rcSmoothingFilter_t *getRcSmoothingData(void)
{
    return &rcSmoothingData;
}

bool rcSmoothingInitializationComplete(void)
{
    return rcSmoothingData.filterInitialized;
}
#endif // USE_RC_SMOOTHING_FILTER
