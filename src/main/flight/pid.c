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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config_reset.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/fc_core.h"
#include "fc/fc_rc.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/mixer.h"

#include "io/gps.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"


FAST_RAM_ZERO_INIT uint32_t targetPidLooptime;
FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT bool pidStabilisationEnabled;

static FAST_RAM_ZERO_INIT bool inCrashRecoveryMode = false;

static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#ifdef STM32F10X
#define PID_PROCESS_DENOM_DEFAULT       1
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500)  || defined(USE_GYRO_SPI_ICM20689)
#define PID_PROCESS_DENOM_DEFAULT       4
#else
#define PID_PROCESS_DENOM_DEFAULT       2
#endif

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 25,  // throttle level % needed to accumulate deactivation time
    .runaway_takeoff_deactivate_delay = 500     // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);
#endif

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, MAX_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 4);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  { 46, 45, 25 },
            [PID_PITCH] = { 50, 50, 27 },
            [PID_YAW] =   { 65, 45, 0 },
            [PID_ALT] =   { 50, 0, 0 },
            [PID_POS] =   { 15, 0, 0 },     // POSHOLD_P * 100, POSHOLD_I * 100,
            [PID_POSR] =  { 34, 14, 53 },   // POSHOLD_RATE_P * 10, POSHOLD_RATE_I * 100, POSHOLD_RATE_D * 1000,
            [PID_NAVR] =  { 25, 33, 83 },   // NAV_P * 10, NAV_I * 100, NAV_D * 1000
            [PID_LEVEL] = { 50, 50, 75 },
            [PID_MAG] =   { 40, 0, 0 },
            [PID_VEL] =   { 55, 55, 75 }
        },

        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 0,
        .dterm_lowpass_hz = 100,    // dual PT1 filtering ON by default
        .dterm_lowpass2_hz = 200,   // second Dterm LPF ON by default
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 160,
        .dterm_filter_type = FILTER_PT1,
        .itermWindupPointPercent = 40,
        .vbatPidCompensation = 0,
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .levelAngleLimit = 55,
        .setpointRelaxRatio = 0,
        .dtermSetpointWeight = 60,
        .yawRateAccelLimit = 100,
        .rateAccelLimit = 0,
        .itermThrottleThreshold = 350,
        .itermAcceleratorGain = 5000,
        .crash_time = 500,          // ms
        .crash_delay = 0,           // ms
        .crash_recovery_angle = 10, // degrees
        .crash_recovery_rate = 100, // degrees/second
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .crash_limit_yaw = 200,
        .itermLimit = 150,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = true,
        .smart_feedforward = false,
        .iterm_relax = ITERM_RELAX_OFF,
        .iterm_relax_cutoff = 11,
        .iterm_relax_type = ITERM_RELAX_GYRO,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < MAX_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    dT = targetPidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
}

static FAST_RAM float itermAccelerator = 1.0f;

void pidSetItermAccelerator(float newItermAccelerator)
{
    itermAccelerator = newItermAccelerator;
}

float pidItermAccelerator(void)
{
    return itermAccelerator;
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} dtermLowpass_t;

static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermNotchApplyFn;
static FAST_RAM_ZERO_INIT biquadFilter_t dtermNotch[2];
static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpassApplyFn;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass[2];
static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpass2ApplyFn;
static FAST_RAM_ZERO_INIT pt1Filter_t dtermLowpass2[2];
static FAST_RAM_ZERO_INIT filterApplyFnPtr ptermYawLowpassApplyFn;
static FAST_RAM_ZERO_INIT pt1Filter_t ptermYawLowpass;
#if defined(USE_ITERM_RELAX)
static FAST_RAM_ZERO_INIT pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t itermRelax;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxCutoff;
#endif

#ifdef USE_RC_SMOOTHING_FILTER
static FAST_RAM_ZERO_INIT pt1Filter_t setpointDerivativePt1[2];
static FAST_RAM_ZERO_INIT biquadFilter_t setpointDerivativeBiquad[2];
static FAST_RAM_ZERO_INIT bool setpointDerivativeLpfInitialized;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingDebugAxis;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingFilterType;
#endif // USE_RC_SMOOTHING_FILTER

void pidInitFilters(const pidProfile_t *pidProfile)
{
    BUILD_BUG_ON(FD_YAW != 2); // only setting up Dterm filters on roll and pitch axes, so ensure yaw axis is 2

    if (targetPidLooptime == 0) {
        // no looptime set, so set all the filters to null
        dtermNotchApplyFn = nullFilterApply;
        dtermLowpassApplyFn = nullFilterApply;
        ptermYawLowpassApplyFn = nullFilterApply;
        return;
    }

    const uint32_t pidFrequencyNyquist = pidFrequency / 2; // No rounding needed

    uint16_t dTermNotchHz;
    if (pidProfile->dterm_notch_hz <= pidFrequencyNyquist) {
        dTermNotchHz = pidProfile->dterm_notch_hz;
    } else {
        if (pidProfile->dterm_notch_cutoff < pidFrequencyNyquist) {
            dTermNotchHz = pidFrequencyNyquist;
        } else {
            dTermNotchHz = 0;
        }
    }

    if (dTermNotchHz != 0 && pidProfile->dterm_notch_cutoff != 0) {
        dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
            biquadFilterInit(&dtermNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH);
        }
    } else {
        dtermNotchApplyFn = nullFilterApply;
    }

    //2nd Dterm Lowpass Filter
    if (pidProfile->dterm_lowpass2_hz == 0 || pidProfile->dterm_lowpass2_hz > pidFrequencyNyquist) {
    	dtermLowpass2ApplyFn = nullFilterApply;
    } else {
        dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
        for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
            pt1FilterInit(&dtermLowpass2[axis], pt1FilterGain(pidProfile->dterm_lowpass2_hz, dT));
        }
    }

    if (pidProfile->dterm_lowpass_hz == 0 || pidProfile->dterm_lowpass_hz > pidFrequencyNyquist) {
        dtermLowpassApplyFn = nullFilterApply;
    } else {
        switch (pidProfile->dterm_filter_type) {
        default:
            dtermLowpassApplyFn = nullFilterApply;
            break;
        case FILTER_PT1:
            dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
                pt1FilterInit(&dtermLowpass[axis].pt1Filter, pt1FilterGain(pidProfile->dterm_lowpass_hz, dT));
            }
            break;
        case FILTER_BIQUAD:
            dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
                biquadFilterInitLPF(&dtermLowpass[axis].biquadFilter, pidProfile->dterm_lowpass_hz, targetPidLooptime);
            }
            break;
        }
    }

    if (pidProfile->yaw_lowpass_hz == 0 || pidProfile->yaw_lowpass_hz > pidFrequencyNyquist) {
        ptermYawLowpassApplyFn = nullFilterApply;
    } else {
        ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
        pt1FilterInit(&ptermYawLowpass, pt1FilterGain(pidProfile->yaw_lowpass_hz, dT));
    }

#if defined(USE_THROTTLE_BOOST)
    pt1FilterInit(&throttleLpf, pt1FilterGain(pidProfile->throttle_boost_cutoff, dT));
#endif
#if defined(USE_ITERM_RELAX)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&windupLpf[i], pt1FilterGain(itermRelaxCutoff, dT));
        }
    }
#endif
}

#ifdef USE_RC_SMOOTHING_FILTER
void pidInitSetpointDerivativeLpf(uint16_t filterCutoff, uint8_t debugAxis, uint8_t filterType)
{
    rcSmoothingDebugAxis = debugAxis;
    rcSmoothingFilterType = filterType;
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF)) {
        setpointDerivativeLpfInitialized = true;
        for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
            switch (rcSmoothingFilterType) {
                case RC_SMOOTHING_DERIVATIVE_PT1:
                    pt1FilterInit(&setpointDerivativePt1[axis], pt1FilterGain(filterCutoff, dT));
                    break;
                case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                    biquadFilterInitLPF(&setpointDerivativeBiquad[axis], filterCutoff, targetPidLooptime);
                    break;
            }
        }
    }
}

void pidUpdateSetpointDerivativeLpf(uint16_t filterCutoff)
{
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF)) {
        for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
            switch (rcSmoothingFilterType) {
                case RC_SMOOTHING_DERIVATIVE_PT1:
                    pt1FilterUpdateCutoff(&setpointDerivativePt1[axis], pt1FilterGain(filterCutoff, dT));
                    break;
                case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                    biquadFilterUpdateLPF(&setpointDerivativeBiquad[axis], filterCutoff, targetPidLooptime);
                    break;
            }
        }
    }
}
#endif // USE_RC_SMOOTHING_FILTER

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
} pidCoefficient_t;

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[3];
static FAST_RAM_ZERO_INIT float maxVelocity[3];
static FAST_RAM_ZERO_INIT float relaxFactor;
static FAST_RAM_ZERO_INIT float dtermSetpointWeight;
static FAST_RAM_ZERO_INIT float levelGain, horizonGain, horizonTransition, horizonCutoffDegrees, horizonFactorRatio;
static FAST_RAM_ZERO_INIT float ITermWindupPointInv;
static FAST_RAM_ZERO_INIT uint8_t horizonTiltExpertMode;
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeLimitUs;
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeDelayUs;
static FAST_RAM_ZERO_INIT int32_t crashRecoveryAngleDeciDegrees;
static FAST_RAM_ZERO_INIT float crashRecoveryRate;
static FAST_RAM_ZERO_INIT float crashDtermThreshold;
static FAST_RAM_ZERO_INIT float crashGyroThreshold;
static FAST_RAM_ZERO_INIT float crashSetpointThreshold;
static FAST_RAM_ZERO_INIT float crashLimitYaw;
static FAST_RAM_ZERO_INIT float itermLimit;
#if defined(USE_THROTTLE_BOOST)
FAST_RAM_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
#endif
static FAST_RAM_ZERO_INIT bool itermRotation;

#if defined(USE_SMART_FEEDFORWARD)
static FAST_RAM_ZERO_INIT bool smartFeedforward;
#endif
#if defined(USE_ABSOLUTE_CONTROL)
static FAST_RAM_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acGain;
static FAST_RAM_ZERO_INIT float acLimit;
static FAST_RAM_ZERO_INIT float acErrorLimit;
#endif

void pidResetITerm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}

#ifdef USE_ACRO_TRAINER
static FAST_RAM_ZERO_INIT float acroTrainerAngleLimit;
static FAST_RAM_ZERO_INIT float acroTrainerLookaheadTime;
static FAST_RAM_ZERO_INIT uint8_t acroTrainerDebugAxis;
static FAST_RAM_ZERO_INIT bool acroTrainerActive;
static FAST_RAM_ZERO_INIT int acroTrainerAxisState[2];  // only need roll and pitch
static FAST_RAM_ZERO_INIT float acroTrainerGain;
#endif // USE_ACRO_TRAINER

void pidInitConfig(const pidProfile_t *pidProfile)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
    }

    dtermSetpointWeight = pidProfile->dtermSetpointWeight / 100.0f;
    if (pidProfile->setpointRelaxRatio == 0) {
        relaxFactor = 0;
    } else {
        relaxFactor = 100.0f / pidProfile->setpointRelaxRatio;
    }
    levelGain = pidProfile->pid[PID_LEVEL].P / 10.0f;
    horizonGain = pidProfile->pid[PID_LEVEL].I / 10.0f;
    horizonTransition = (float)pidProfile->pid[PID_LEVEL].D;
    horizonTiltExpertMode = pidProfile->horizon_tilt_expert_mode;
    horizonCutoffDegrees = (175 - pidProfile->horizon_tilt_effect) * 1.8f;
    horizonFactorRatio = (100 - pidProfile->horizon_tilt_effect) * 0.01f;
    maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * dT;
    maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * dT;
    const float ITermWindupPoint = (float)pidProfile->itermWindupPointPercent / 100.0f;
    ITermWindupPointInv = 1.0f / (1.0f - ITermWindupPoint);
    crashTimeLimitUs = pidProfile->crash_time * 1000;
    crashTimeDelayUs = pidProfile->crash_delay * 1000;
    crashRecoveryAngleDeciDegrees = pidProfile->crash_recovery_angle * 10;
    crashRecoveryRate = pidProfile->crash_recovery_rate;
    crashGyroThreshold = pidProfile->crash_gthreshold;
    crashDtermThreshold = pidProfile->crash_dthreshold;
    crashSetpointThreshold = pidProfile->crash_setpoint_threshold;
    crashLimitYaw = pidProfile->crash_limit_yaw;
    itermLimit = pidProfile->itermLimit;
#if defined(USE_THROTTLE_BOOST)
    throttleBoost = pidProfile->throttle_boost * 0.1f;
#endif
    itermRotation = pidProfile->iterm_rotation;
#if defined(USE_SMART_FEEDFORWARD)
    smartFeedforward = pidProfile->smart_feedforward;
#endif
#if defined(USE_ITERM_RELAX)
    itermRelax = pidProfile->iterm_relax;
    itermRelaxType = pidProfile->iterm_relax_type;
    itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
#endif

#ifdef USE_ACRO_TRAINER
    acroTrainerAngleLimit = pidProfile->acro_trainer_angle_limit;
    acroTrainerLookaheadTime = (float)pidProfile->acro_trainer_lookahead_ms / 1000.0f;
    acroTrainerDebugAxis = pidProfile->acro_trainer_debug_axis;
    acroTrainerGain = (float)pidProfile->acro_trainer_gain / 10.0f;
#endif // USE_ACRO_TRAINER

#if defined(USE_ABSOLUTE_CONTROL)
    acGain = (float)pidProfile->abs_control_gain;
    acLimit = (float)pidProfile->abs_control_limit;
    acErrorLimit = (float)pidProfile->abs_control_error_limit;
#endif
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime * pidConfig()->pid_process_denom); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    acroTrainerAxisState[FD_ROLL] = 0;
    acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if ((dstPidProfileIndex < MAX_PROFILE_COUNT-1 && srcPidProfileIndex < MAX_PROFILE_COUNT-1)
        && dstPidProfileIndex != srcPidProfileIndex
    ) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
static float calcHorizonLevelStrength(void)
{
    // start with 1.0 at center stick, 0.0 at max stick deflection:
    float horizonLevelStrength = 1.0f - MAX(getRcDeflectionAbs(FD_ROLL), getRcDeflectionAbs(FD_PITCH));

    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;

    // horizonTiltExpertMode:  0 = leveling always active when sticks centered,
    //                         1 = leveling can be totally off when inverted
    if (horizonTiltExpertMode) {
        if (horizonTransition > 0 && horizonCutoffDegrees > 0) {
                    // if d_level > 0 and horizonTiltEffect < 175
            // horizonCutoffDegrees: 0 to 125 => 270 to 90 (represents where leveling goes to zero)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations; 0.0 at horizonCutoffDegrees value:
            const float inclinationLevelRatio = constrainf((horizonCutoffDegrees-currentInclination) / horizonCutoffDegrees, 0, 1);
            // apply configured horizon sensitivity:
                // when stick is near center (horizonLevelStrength ~= 1.0)
                //  H_sensitivity value has little effect,
                // when stick is deflected (horizonLevelStrength near 0.0)
                //  H_sensitivity value has more effect:
            horizonLevelStrength = (horizonLevelStrength - 1) * 100 / horizonTransition + 1;
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength *= inclinationLevelRatio;
        } else  { // d_level=0 or horizon_tilt_effect>=175 means no leveling
          horizonLevelStrength = 0;
        }
    } else { // horizon_tilt_expert_mode = 0 (leveling always active when sticks centered)
        float sensitFact;
        if (horizonFactorRatio < 1.01f) { // if horizonTiltEffect > 0
            // horizonFactorRatio: 1.0 to 0.0 (larger means more leveling)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations, goes to 1.0 at inclination==level:
            const float inclinationLevelRatio = (180-currentInclination)/180 * (1.0f-horizonFactorRatio) + horizonFactorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = horizonTransition * inclinationLevelRatio;
        } else { // horizonTiltEffect=0 for "old" functionality
            sensitFact = horizonTransition;
        }

        if (sensitFact <= 0) {           // zero means no leveling
            horizonLevelStrength = 0;
        } else {
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  sensitFact value has more effect:
            horizonLevelStrength = ((horizonLevelStrength - 1) * (100 / sensitFact)) + 1;
        }
    }
    return constrainf(horizonLevelStrength, 0, 1);
}

static float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint) {
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
    float angle = pidProfile->levelAngleLimit * getRcDeflection(axis);
#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100; // ANGLE IS IN CENTIDEGREES
#endif
    angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);
    const float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // ANGLE mode - control is angle based
        currentPidSetpoint = errorAngle * levelGain;
    } else {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        const float horizonLevelStrength = calcHorizonLevelStrength();
        currentPidSetpoint = currentPidSetpoint + (errorAngle * horizonGain * horizonLevelStrength);
    }
    return currentPidSetpoint;
}

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[3];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (ABS(currentVelocity) > maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity[axis] : previousSetpoint[axis] - maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static timeUs_t crashDetectedAtUs;

static void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate)
{
    if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeDelayUs) {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP) {
            BEEP_ON;
        }
        if (axis == FD_YAW) {
            *errorRate = constrainf(*errorRate, -crashLimitYaw, crashLimitYaw);
        } else {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            if (sensors(SENSOR_ACC)) {
                // errorAngle is deviation from horizontal
                const float errorAngle =  -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                *currentPidSetpoint = errorAngle * levelGain;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
        // reset ITerm, since accumulated error before crash is now meaningless
        // and ITerm windup during crash recovery can be extreme, especially on yaw axis
        pidData[axis].I = 0.0f;
        if (cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeLimitUs
            || (getMotorMixRange() < 1.0f
                   && ABS(gyro.gyroADCf[FD_ROLL]) < crashRecoveryRate
                   && ABS(gyro.gyroADCf[FD_PITCH]) < crashRecoveryRate
                   && ABS(gyro.gyroADCf[FD_YAW]) < crashRecoveryRate)) {
            if (sensors(SENSOR_ACC)) {
                // check aircraft nearly level
                if (ABS(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < crashRecoveryAngleDeciDegrees
                   && ABS(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < crashRecoveryAngleDeciDegrees) {
                    inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            } else {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
}

static void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected()) {
        if (ARMING_FLAG(ARMED)) {
            if (getMotorMixRange() >= 1.0f && !inCrashRecoveryMode
                && ABS(delta) > crashDtermThreshold
                && ABS(errorRate) > crashGyroThreshold
                && ABS(getSetpointRate(axis)) < crashSetpointThreshold) {
                inCrashRecoveryMode = true;
                crashDetectedAtUs = currentTimeUs;
            }
            if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) < crashTimeDelayUs && (ABS(errorRate) < crashGyroThreshold
                || ABS(getSetpointRate(axis)) > crashSetpointThreshold)) {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else if (inCrashRecoveryMode) {
            inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT]) 
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

static void rotateITermAndAxisError() 
{
    if (itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || acGain > 0
#endif
        ) {
        const float gyroToAngle = dT * RAD;
        float rotationRads[3];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (acGain > 0) {
            rotateVector(axisError, rotationRads);
        }
#endif
        if (itermRotation) {
            float v[3];
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            rotateVector(v, rotationRads );
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

#ifdef USE_ACRO_TRAINER

int acroTrainerSign(float x)
{
    return x > 0 ? 1 : -1;
}

// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
static FAST_CODE_NOINLINE float applyAcroTrainer(int axis, const rollAndPitchTrims_t *angleTrim, float setPoint)
{
    float ret = setPoint;

    if (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
        bool resetIterm = false;
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((acroTrainerAxisState[axis] != 0) && (acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > acroTrainerAngleLimit) && (acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                acroTrainerAxisState[axis] = angleSign;
                resetIterm = true;
            }
        }

        if (acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((acroTrainerAngleLimit * angleSign) - currentAngle) * acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {
        
        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * acroTrainerGain;
                resetIterm = true;
            }
        }

        if (resetIterm) {
            pidData[axis].I = 0;
        }
 
        if (axis == acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, timeUs_t currentTimeUs)
{
    static float previousGyroRateDterm[2];
    static float previousPidSetpoint[2];

    const float tpaFactor = getThrottlePIDAttenuation();
    const float motorMixRange = getMotorMixRange();

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinActive = gyroYawSpinDetected();
#endif

    // Dynamic i component,
    // gradually scale back integration when above windup point
    const float dynCi = MIN((1.0f - motorMixRange) * ITermWindupPointInv, 1.0f) * dT * itermAccelerator;

    // Dynamic d component, enable 2-DOF PID controller only for rate mode
    const float dynCd = flightModeFlags ? 0.0f : dtermSetpointWeight;

    // Precalculate gyro deta for D-term here, this allows loop unrolling
    float gyroRateDterm[2];
    for (int axis = FD_ROLL; axis < FD_YAW; ++axis) {
        gyroRateDterm[axis] = dtermNotchApplyFn((filter_t *) &dtermNotch[axis], gyro.gyroADCf[axis]);
        gyroRateDterm[axis] = dtermLowpassApplyFn((filter_t *) &dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = dtermLowpass2ApplyFn((filter_t *) &dtermLowpass2[axis], gyroRateDterm[axis]);
    }

    rotateITermAndAxisError();

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        float currentPidSetpoint = getSetpointRate(axis);
        if (maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }
        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) && axis != YAW) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }

#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && acroTrainerActive && !inCrashRecoveryMode) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
#endif // USE_ACRO_TRAINER

        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && yawSpinActive) {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec

#ifdef USE_ABSOLUTE_CONTROL
        float acCorrection = 0;
        float acErrorRate;
#endif        
        float itermErrorRate = 0.0f;

#if defined(USE_ITERM_RELAX)
        if (itermRelax && (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY )) {
            const float setpointLpf = pt1FilterApply(&windupLpf[axis], currentPidSetpoint);
            const float setpointHpf = fabsf(currentPidSetpoint - setpointLpf);
            const float itermRelaxFactor = 1 - setpointHpf / 30.0f;
            if (itermRelaxType == ITERM_RELAX_SETPOINT && setpointHpf < 30) {
                itermErrorRate = itermRelaxFactor * (currentPidSetpoint - gyroRate);
            } else if (itermRelaxType == ITERM_RELAX_GYRO ) {
                itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            }
            
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(itermErrorRate));
            }

#if defined(USE_ABSOLUTE_CONTROL)
            const float gmaxac = setpointLpf + 2 * setpointHpf;
            const float gminac = setpointLpf - 2 * setpointHpf;
            if (gyroRate >= gminac && gyroRate <= gmaxac) {
                float acErrorRate1 = gmaxac - gyroRate;
                float acErrorRate2 = gminac - gyroRate;
                if (acErrorRate1 * axisError[axis] < 0) {
                    acErrorRate = acErrorRate1;
                } else {
                    acErrorRate = acErrorRate2;
                }
                if (fabsf(acErrorRate * dT) > fabsf(axisError[axis]) ) {
                    acErrorRate = -axisError[axis] / dT;
                }
            } else {
                acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
            }
#endif // USE_ABSOLUTE_CONTROL             
        } else
#endif // USE_ITERM_RELAX
        {
            itermErrorRate = currentPidSetpoint - gyroRate;
#if defined(USE_ABSOLUTE_CONTROL)
            acErrorRate = itermErrorRate;
#endif // USE_ABSOLUTE_CONTROL
        }
        
#if defined(USE_ABSOLUTE_CONTROL)
        if (acGain > 0 && isAirmodeActivated()) {
            axisError[axis] = constrainf(axisError[axis] + acErrorRate * dT, -acErrorLimit, acErrorLimit);
            acCorrection = constrainf(axisError[axis] * acGain, -acLimit, acLimit);
            currentPidSetpoint += acCorrection;
            itermErrorRate += acCorrection;
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(axisError[axis] * 10));
            }
        }
#endif

        float errorRate = currentPidSetpoint - gyroRate; // r - y
        handleCrashRecovery(
            pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, gyroRate,
            &currentPidSetpoint, &errorRate);

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (dtermSetpointWeight) can be tuned (amount derivative on measurement or error).

        // -----calculate P component and add Dynamic Part based on stick input
        pidData[axis].P = pidCoefficient[axis].Kp * errorRate * tpaFactor;
        if (axis == FD_YAW) {
            pidData[axis].P = ptermYawLowpassApplyFn((filter_t *) &ptermYawLowpass, pidData[axis].P);
        }

        // -----calculate I component

        const float ITerm = pidData[axis].I;
        const float ITermNew = constrainf(ITerm + pidCoefficient[axis].Ki * itermErrorRate * dynCi, -itermLimit, itermLimit);
        const bool outputSaturated = mixerIsOutputSaturated(axis, errorRate);
        if (outputSaturated == false || ABS(ITermNew) < ABS(ITerm)) {
            // Only increase ITerm if output is not saturated
            pidData[axis].I = ITermNew;
        }

        // -----calculate D component
        if (axis != FD_YAW) {
            // no transition if relaxFactor == 0
            float transition = relaxFactor > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * relaxFactor) : 1;

            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float delta =
                - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidFrequency;

            detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, delta, errorRate);

            pidData[axis].D = pidCoefficient[axis].Kd * delta * tpaFactor;

            float pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];

#ifdef USE_RC_SMOOTHING_FILTER
            if (axis == rcSmoothingDebugAxis) {
                DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(pidSetpointDelta * 100.0f));
            }
            if ((dynCd != 0) && setpointDerivativeLpfInitialized) {
                switch (rcSmoothingFilterType) {
                    case RC_SMOOTHING_DERIVATIVE_PT1:
                        pidSetpointDelta = pt1FilterApply(&setpointDerivativePt1[axis], pidSetpointDelta);
                        break;
                    case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                        pidSetpointDelta = biquadFilterApplyDF1(&setpointDerivativeBiquad[axis], pidSetpointDelta);
                        break;
                }
                if (axis == rcSmoothingDebugAxis) {
                    DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(pidSetpointDelta * 100.0f));
                }
            }
#endif // USE_RC_SMOOTHING_FILTER

            const float pidFeedForward =
                pidCoefficient[axis].Kd * dynCd * transition * pidSetpointDelta * tpaFactor * pidFrequency;
#if defined(USE_SMART_FEEDFORWARD)
            bool addFeedforward = true;
            if (smartFeedforward) {
                if (pidData[axis].P * pidFeedForward > 0) {
                    if (ABS(pidFeedForward) > ABS(pidData[axis].P)) {
                        pidData[axis].P = 0;
                    } else {
                        addFeedforward = false;
                    }
                }
            }
            if (addFeedforward)
#endif
            {
                pidData[axis].D += pidFeedForward;
            }
            previousGyroRateDterm[axis] = gyroRateDterm[axis];
            previousPidSetpoint[axis] = currentPidSetpoint;

#ifdef USE_YAW_SPIN_RECOVERY
            if (yawSpinActive)  {
                // zero PIDs on pitch and roll leaving yaw P to correct spin 
                pidData[axis].P = 0;
                pidData[axis].I = 0;
                pidData[axis].D = 0;
            }
#endif // USE_YAW_SPIN_RECOVERY
        }
    }

    // calculating the PID sum
    pidData[FD_ROLL].Sum = pidData[FD_ROLL].P + pidData[FD_ROLL].I + pidData[FD_ROLL].D;
    pidData[FD_PITCH].Sum = pidData[FD_PITCH].P + pidData[FD_PITCH].I + pidData[FD_PITCH].D;

#ifdef USE_YAW_SPIN_RECOVERY
    if (yawSpinActive) {
    // yaw P alone to correct spin 
        pidData[FD_YAW].I = 0;
    }
#endif // USE_YAW_SPIN_RECOVERY

    // YAW has no D
    pidData[FD_YAW].Sum = pidData[FD_YAW].P + pidData[FD_YAW].I;

    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if (!pidStabilisationEnabled || gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;

            pidData[axis].Sum = 0;
        }
    }
}

bool crashRecoveryModeActive(void)
{
    return inCrashRecoveryMode;
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER
