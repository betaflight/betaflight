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
#include "common/filter.h"

#include "drivers/dshot_command.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "pg/motor.h"

#include "rx/rx.h"

#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "pid_init.h"

#ifdef USE_D_MAX
#define D_MAX_RANGE_HZ 85    // PT2 lowpass input cutoff to peak D around propwash frequencies
#define D_MAX_LOWPASS_HZ 35  // PT2 lowpass cutoff to smooth the boost effect
#define D_MAX_GAIN_FACTOR 0.00008f
#define D_MAX_SETPOINT_GAIN_FACTOR 0.00008f
#endif

#define ATTITUDE_CUTOFF_HZ 50

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    pidRuntime.dT = targetPidLooptime * 1e-6f;
    pidRuntime.pidFrequency = 1.0f / pidRuntime.dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(targetPidLooptime);
#endif
}

#ifdef USE_WING
static void tpaSpeedBasicInit(const pidProfile_t *pidProfile)
{
    // basic model assumes prop pitch speed is inf
    const float gravityFactor = pidProfile->tpa_speed_basic_gravity / 100.0f;
    const float delaySec = pidProfile->tpa_speed_basic_delay / 1000.0f;

    pidRuntime.tpaSpeed.twr = 1.0f / (gravityFactor * gravityFactor);
    const float massDragRatio = (2.0f / logf(3.0f)) * (2.0f / logf(3.0f)) * pidRuntime.tpaSpeed.twr * G_ACCELERATION * delaySec * delaySec;
    pidRuntime.tpaSpeed.dragMassRatio = 1.0f / massDragRatio;
    pidRuntime.tpaSpeed.maxSpeed = sqrtf(massDragRatio * pidRuntime.tpaSpeed.twr * G_ACCELERATION + G_ACCELERATION);
    pidRuntime.tpaSpeed.inversePropMaxSpeed = 0.0f;
}

static void tpaSpeedAdvancedInit(const pidProfile_t *pidProfile)
{
    // Advanced model uses prop pitch speed, and is quite limited when craft speed is far above prop pitch speed.
    pidRuntime.tpaSpeed.twr = (float)pidProfile->tpa_speed_adv_thrust / (float)pidProfile->tpa_speed_adv_mass;
    const float mass = pidProfile->tpa_speed_adv_mass / 1000.0f;
    const float dragK = pidProfile->tpa_speed_adv_drag_k / 10000.0f;
    const float propPitch = pidProfile->tpa_speed_adv_prop_pitch / 100.0f;
    pidRuntime.tpaSpeed.dragMassRatio = dragK / mass;
    const float propMaxSpeed = (2.54f / 100.0f / 60.0f) * propPitch * motorConfig()->kv * pidRuntime.tpaSpeed.maxVoltage;
    if (propMaxSpeed <= 0.0f) { // assuming propMaxSpeed is inf
        pidRuntime.tpaSpeed.inversePropMaxSpeed = 0.0f;
    } else {
        pidRuntime.tpaSpeed.inversePropMaxSpeed = 1.0f / propMaxSpeed;
    }

    const float maxFallSpeed = sqrtf(mass * G_ACCELERATION / dragK);

    const float a = dragK;
    const float b = mass * pidRuntime.tpaSpeed.twr * G_ACCELERATION * pidRuntime.tpaSpeed.inversePropMaxSpeed;
    const float c = -mass * (pidRuntime.tpaSpeed.twr + 1) * G_ACCELERATION;

    const float maxDiveSpeed = (-b + sqrtf(b*b - 4.0f * a * c)) / (2.0f * a);

    pidRuntime.tpaSpeed.maxSpeed = MAX(maxFallSpeed, maxDiveSpeed);
    UNUSED(pidProfile);
}

static void tpaSpeedInit(const pidProfile_t *pidProfile)
{
    pidRuntime.tpaSpeed.speed = 0.0f;
    pidRuntime.tpaSpeed.maxVoltage = pidProfile->tpa_speed_max_voltage / 100.0f;
    pidRuntime.tpaSpeed.pitchOffset = pidProfile->tpa_speed_pitch_offset * M_PIf / 10.0f / 180.0f;

    switch (pidProfile->tpa_speed_type) {
    case TPA_SPEED_BASIC:
        tpaSpeedBasicInit(pidProfile);
        break;
    case TPA_SPEED_ADVANCED:
        tpaSpeedAdvancedInit(pidProfile);
        break;
    default:
        break;
    }
}
#endif // USE_WING

void pidInitFilters(const pidProfile_t *pidProfile)
{
    STATIC_ASSERT(FD_YAW == 2, FD_YAW_incorrect); // ensure yaw axis is 2

    if (targetPidLooptime == 0) {
        // no looptime set, so set all the filters to null
        pidRuntime.dtermNotchApplyFn = nullFilterApply;
        pidRuntime.dtermLowpassApplyFn = nullFilterApply;
        pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
        pidRuntime.ptermYawLowpassApplyFn = nullFilterApply;
        return;
    }

    const uint32_t pidFrequencyNyquist = pidRuntime.pidFrequency / 2; // No rounding needed

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
        pidRuntime.dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            biquadFilterInit(&pidRuntime.dtermNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    } else {
        pidRuntime.dtermNotchApplyFn = nullFilterApply;
    }

    //1st Dterm Lowpass Filter
    uint16_t dterm_lpf1_init_hz = pidProfile->dterm_lpf1_static_hz;

#ifdef USE_DYN_LPF
    if (pidProfile->dterm_lpf1_dyn_min_hz) {
        dterm_lpf1_init_hz = pidProfile->dterm_lpf1_dyn_min_hz;
    }
#endif

    if (dterm_lpf1_init_hz > 0) {
        switch (pidProfile->dterm_lpf1_type) {
        case FILTER_PT1:
            pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
            break;
        case FILTER_BIQUAD:
            if (pidProfile->dterm_lpf1_static_hz < pidFrequencyNyquist) {
#ifdef USE_DYN_LPF
                pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1;
#else
                pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApply;
#endif
                for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                    biquadFilterInitLPF(&pidRuntime.dtermLowpass[axis].biquadFilter, dterm_lpf1_init_hz, targetPidLooptime);
                }
            } else {
                pidRuntime.dtermLowpassApplyFn = nullFilterApply;
            }
            break;
        case FILTER_PT2:
            pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)pt2FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt2FilterInit(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
            break;
        case FILTER_PT3:
            pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)pt3FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt3FilterInit(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
            break;
        default:
            pidRuntime.dtermLowpassApplyFn = nullFilterApply;
            break;
        }
    } else {
        pidRuntime.dtermLowpassApplyFn = nullFilterApply;
    }

    //2nd Dterm Lowpass Filter
    if (pidProfile->dterm_lpf2_static_hz > 0) {
        switch (pidProfile->dterm_lpf2_type) {
        case FILTER_PT1:
            pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&pidRuntime.dtermLowpass2[axis].pt1Filter, pt1FilterGain(pidProfile->dterm_lpf2_static_hz, pidRuntime.dT));
            }
            break;
        case FILTER_BIQUAD:
            if (pidProfile->dterm_lpf2_static_hz < pidFrequencyNyquist) {
                pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
                for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                    biquadFilterInitLPF(&pidRuntime.dtermLowpass2[axis].biquadFilter, pidProfile->dterm_lpf2_static_hz, targetPidLooptime);
                }
            } else {
                pidRuntime.dtermLowpassApplyFn = nullFilterApply;
            }
            break;
        case FILTER_PT2:
            pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt2FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt2FilterInit(&pidRuntime.dtermLowpass2[axis].pt2Filter, pt2FilterGain(pidProfile->dterm_lpf2_static_hz, pidRuntime.dT));
            }
            break;
        case FILTER_PT3:
            pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt3FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt3FilterInit(&pidRuntime.dtermLowpass2[axis].pt3Filter, pt3FilterGain(pidProfile->dterm_lpf2_static_hz, pidRuntime.dT));
            }
            break;
        default:
            pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
            break;
        }
    } else {
        pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
    }

    if (pidProfile->yaw_lowpass_hz == 0) {
        pidRuntime.ptermYawLowpassApplyFn = nullFilterApply;
    } else {
        pidRuntime.ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
        pt1FilterInit(&pidRuntime.ptermYawLowpass, pt1FilterGain(pidProfile->yaw_lowpass_hz, pidRuntime.dT));
    }

#if defined(USE_THROTTLE_BOOST)
    pt1FilterInit(&throttleLpf, pt1FilterGain(pidProfile->throttle_boost_cutoff, pidRuntime.dT));
#endif

#if defined(USE_ITERM_RELAX)
    if (pidRuntime.itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&pidRuntime.windupLpf[i], pt1FilterGain(pidRuntime.itermRelaxCutoff, pidRuntime.dT));
        }
    }
#endif

#if defined(USE_ABSOLUTE_CONTROL)
    if (pidRuntime.itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&pidRuntime.acLpf[i], pt1FilterGain(pidRuntime.acCutoff, pidRuntime.dT));
        }
    }
#endif

#ifdef USE_D_MAX
    // Initialize the filters for all axis even if the d_max[axis] value is 0
    // Otherwise if the pidProfile->d_max_xxx parameters are ever added to
    // in-flight adjustments and transition from 0 to > 0 in flight the feature
    // won't work because the filter wasn't initialized.
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pt2FilterInit(&pidRuntime.dMaxRange[axis], pt2FilterGain(D_MAX_RANGE_HZ, pidRuntime.dT));
        pt2FilterInit(&pidRuntime.dMaxLowpass[axis], pt2FilterGain(D_MAX_LOWPASS_HZ, pidRuntime.dT));
     }
#endif

#if defined(USE_AIRMODE_LPF)
    if (pidProfile->transient_throttle_limit) {
        pt1FilterInit(&pidRuntime.airmodeThrottleLpf1, pt1FilterGain(7.0f, pidRuntime.dT));
        pt1FilterInit(&pidRuntime.airmodeThrottleLpf2, pt1FilterGain(20.0f, pidRuntime.dT));
    }
#endif

#ifdef USE_ACC
    const float k = pt3FilterGain(ATTITUDE_CUTOFF_HZ, pidRuntime.dT);
    const float angleCutoffHz = 1000.0f / (2.0f * M_PIf * pidProfile->angle_feedforward_smoothing_ms); // default of 80ms -> 2.0Hz, 160ms -> 1.0Hz, approximately
    const float k2 = pt3FilterGain(angleCutoffHz, pidRuntime.dT);
    pidRuntime.horizonDelayMs = pidProfile->horizon_delay_ms;
    if (pidRuntime.horizonDelayMs) {
        const float horizonSmoothingHz = 1e3f / (2.0f * M_PIf * pidProfile->horizon_delay_ms); // default of 500ms means 0.318Hz
        const float kHorizon = pt1FilterGain(horizonSmoothingHz, pidRuntime.dT);
        pt1FilterInit(&pidRuntime.horizonSmoothingPt1, kHorizon);
    }

    for (int axis = 0; axis < 2; axis++) {  // ROLL and PITCH only
        pt3FilterInit(&pidRuntime.attitudeFilter[axis], k);
        pt3FilterInit(&pidRuntime.angleFeedforwardPt3[axis], k2);
    }
    pidRuntime.angleYawSetpoint = 0.0f;
#endif

    pt2FilterInit(&pidRuntime.antiGravityLpf, pt2FilterGain(pidProfile->anti_gravity_cutoff_hz, pidRuntime.dT));
#ifdef USE_WING
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        pidRuntime.spa[axis] = 1.0f; // 1.0 = no PID attenuation in runtime. 0 - full attenuation (no PIDs)
    }
#endif
}

#ifdef USE_ADVANCED_TPA
static float tpaCurveHyperbolicFunction(float x, void *args)
{
    const pidProfile_t *pidProfile = (const pidProfile_t*)args;

    const float thrStall = pidProfile->tpa_curve_stall_throttle / 100.0f;
    const float pidThr0 = pidProfile->tpa_curve_pid_thr0 / 100.0f;

    if (x <= thrStall) {
        return pidThr0;
    }

    const float expoDivider = pidProfile->tpa_curve_expo / 10.0f - 1.0f;
    const float expo = (fabsf(expoDivider) > 1e-3f) ?  1.0f / expoDivider : 1e3f; // avoiding division by zero for const float base = ...

    const float pidThr100 = pidProfile->tpa_curve_pid_thr100 / 100.0f;
    const float xShifted = scaleRangef(x, thrStall, 1.0f, 0.0f, 1.0f);
    const float base = (1 + (powf(pidThr0 / pidThr100, 1.0f / expo) - 1) * xShifted);
    const float divisor = powf(base, expo);

    return pidThr0 / divisor;
}

static void tpaCurveHyperbolicInit(const pidProfile_t *pidProfile)
{
    pwlInitialize(&pidRuntime.tpaCurvePwl, pidRuntime.tpaCurvePwl_yValues, TPA_CURVE_PWL_SIZE, 0.0f, 1.0f);
    pwlFill(&pidRuntime.tpaCurvePwl, tpaCurveHyperbolicFunction, (void*)pidProfile);
}

static void tpaCurveInit(const pidProfile_t *pidProfile)
{
        pidRuntime.tpaCurveType = pidProfile->tpa_curve_type;
        switch (pidRuntime.tpaCurveType) {
        case TPA_CURVE_HYPERBOLIC:
            tpaCurveHyperbolicInit(pidProfile);
            return;
        case TPA_CURVE_CLASSIC:
        default:
            return;
        }
}
#endif // USE_ADVANCED_TPA

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
#ifdef USE_RPM_FILTER
    rpmFilterInit(rpmFilterConfig(), gyro.targetLooptime);
#endif
#ifdef USE_ADVANCED_TPA
    tpaCurveInit(pidProfile);
#endif
}

void pidInitConfig(const pidProfile_t *pidProfile)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidRuntime.pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidRuntime.pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidRuntime.pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        pidRuntime.pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile->pid[axis].F * 0.01f);
    }
#ifdef USE_INTEGRATED_YAW_CONTROL
    if (!pidProfile->use_integrated_yaw)
#endif
    {
        pidRuntime.pidCoefficient[FD_YAW].Ki *= 2.5f;
    }
    pidRuntime.angleGain = pidProfile->pid[PID_LEVEL].P / 10.0f;
    pidRuntime.angleFeedforwardGain = pidProfile->pid[PID_LEVEL].F / 100.0f;
#ifdef USE_ACC
    pidRuntime.angleEarthRef = pidProfile->angle_earth_ref / 100.0f;
#endif
    pidRuntime.horizonGain = MIN(pidProfile->pid[PID_LEVEL].I / 100.0f, 1.0f);
    pidRuntime.horizonIgnoreSticks = (pidProfile->horizon_ignore_sticks) ? 1.0f : 0.0f;

    pidRuntime.horizonLimitSticks = pidProfile->pid[PID_LEVEL].D / 100.0f;
    pidRuntime.horizonLimitSticksInv = (pidProfile->pid[PID_LEVEL].D) ? 1.0f / pidRuntime.horizonLimitSticks : 1.0f;
    pidRuntime.horizonLimitDegrees = (float)pidProfile->horizon_limit_degrees;
    pidRuntime.horizonLimitDegreesInv = (pidProfile->horizon_limit_degrees) ? 1.0f / pidRuntime.horizonLimitDegrees : 1.0f;
#ifdef USE_ACC
    pidRuntime.horizonDelayMs = pidProfile->horizon_delay_ms;
#endif

    pidRuntime.maxVelocity[FD_ROLL] = pidRuntime.maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.antiGravityGain = pidProfile->anti_gravity_gain;
    pidRuntime.crashTimeLimitUs = pidProfile->crash_time * 1000;
    pidRuntime.crashTimeDelayUs = pidProfile->crash_delay * 1000;
    pidRuntime.crashRecoveryAngleDeciDegrees = pidProfile->crash_recovery_angle * 10;
    pidRuntime.crashRecoveryRate = pidProfile->crash_recovery_rate;
    pidRuntime.crashGyroThreshold = pidProfile->crash_gthreshold; // error in deg/s
    pidRuntime.crashDtermThreshold = pidProfile->crash_dthreshold * 1000.0f; // gyro delta in deg/s/s * 1000 to match original 2017 intent
    pidRuntime.crashSetpointThreshold = pidProfile->crash_setpoint_threshold;
    pidRuntime.crashLimitYaw = pidProfile->crash_limit_yaw;

    pidRuntime.itermLimit = 0.01f * pidProfile->itermWindup * pidProfile->pidSumLimit;
    pidRuntime.itermLimitYaw = 0.01f * pidProfile->itermWindup * pidProfile->pidSumLimitYaw;

#if defined(USE_THROTTLE_BOOST)
    throttleBoost = pidProfile->throttle_boost * 0.1f;
#endif
    pidRuntime.itermRotation = pidProfile->iterm_rotation;

    // Calculate the anti-gravity value that will trigger the OSD display when its strength exceeds 25% of max.
    // This gives a useful indication of AG activity without excessive display.
    pidRuntime.antiGravityOsdCutoff = (pidRuntime.antiGravityGain / 10.0f) * 0.25f;
    pidRuntime.antiGravityPGain = ((float)(pidProfile->anti_gravity_p_gain) / 100.0f) * ANTIGRAVITY_KP;

#if defined(USE_ITERM_RELAX)
    pidRuntime.itermRelax = pidProfile->iterm_relax;
    pidRuntime.itermRelaxType = pidProfile->iterm_relax_type;
    pidRuntime.itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
#endif

#ifdef USE_ACRO_TRAINER
    pidRuntime.acroTrainerAngleLimit = pidProfile->acro_trainer_angle_limit;
    pidRuntime.acroTrainerLookaheadTime = (float)pidProfile->acro_trainer_lookahead_ms / 1000.0f;
    pidRuntime.acroTrainerDebugAxis = pidProfile->acro_trainer_debug_axis;
    pidRuntime.acroTrainerGain = (float)pidProfile->acro_trainer_gain / 10.0f;
#endif // USE_ACRO_TRAINER

#if defined(USE_ABSOLUTE_CONTROL)
    pidRuntime.acGain = (float)pidProfile->abs_control_gain;
    pidRuntime.acLimit = (float)pidProfile->abs_control_limit;
    pidRuntime.acErrorLimit = (float)pidProfile->abs_control_error_limit;
    pidRuntime.acCutoff = (float)pidProfile->abs_control_cutoff;
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        float iCorrection = -pidRuntime.acGain * PTERM_SCALE / ITERM_SCALE * pidRuntime.pidCoefficient[axis].Kp;
        pidRuntime.pidCoefficient[axis].Ki = MAX(0.0f, pidRuntime.pidCoefficient[axis].Ki + iCorrection);
    }
#endif

#ifdef USE_DYN_LPF
    if (pidProfile->dterm_lpf1_dyn_min_hz > 0) {
        switch (pidProfile->dterm_lpf1_type) {
        case FILTER_PT1:
            pidRuntime.dynLpfFilter = DYN_LPF_PT1;
            break;
        case FILTER_BIQUAD:
            pidRuntime.dynLpfFilter = DYN_LPF_BIQUAD;
            break;
        case FILTER_PT2:
            pidRuntime.dynLpfFilter = DYN_LPF_PT2;
            break;
        case FILTER_PT3:
            pidRuntime.dynLpfFilter = DYN_LPF_PT3;
            break;
        default:
            pidRuntime.dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        pidRuntime.dynLpfFilter = DYN_LPF_NONE;
    }
    pidRuntime.dynLpfMin = pidProfile->dterm_lpf1_dyn_min_hz;
    pidRuntime.dynLpfMax = pidProfile->dterm_lpf1_dyn_max_hz;
    pidRuntime.dynLpfCurveExpo = pidProfile->dterm_lpf1_dyn_expo;
#endif

#ifdef USE_LAUNCH_CONTROL
    pidRuntime.launchControlMode = pidProfile->launchControlMode;
    if (sensors(SENSOR_ACC)) {
        pidRuntime.launchControlAngleLimit = pidProfile->launchControlAngleLimit;
    } else {
        pidRuntime.launchControlAngleLimit = 0;
    }
    pidRuntime.launchControlKi = ITERM_SCALE * pidProfile->launchControlGain;
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    pidRuntime.useIntegratedYaw = pidProfile->use_integrated_yaw;
    pidRuntime.integratedYawRelax = pidProfile->integrated_yaw_relax;
#endif

#ifdef USE_THRUST_LINEARIZATION
    pidRuntime.thrustLinearization = pidProfile->thrustLinearization / 100.0f;
    pidRuntime.throttleCompensateAmount = pidRuntime.thrustLinearization - 0.5f * sq(pidRuntime.thrustLinearization);
#endif

#ifdef USE_D_MAX
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        const uint8_t dMax = pidProfile->d_max[axis];
        if ((pidProfile->pid[axis].D > 0) && dMax > pidProfile->pid[axis].D) {
            pidRuntime.dMaxPercent[axis] = (float) dMax / pidProfile->pid[axis].D;
            // fraction that Dmax is higher than D, eg if D is 8 and Dmax is 10, Dmax is 1.25 times bigger
        } else {
            pidRuntime.dMaxPercent[axis] = 1.0f;
        }
    }
    pidRuntime.dMaxGyroGain = D_MAX_GAIN_FACTOR * pidProfile->d_max_gain / D_MAX_LOWPASS_HZ;
    pidRuntime.dMaxSetpointGain = D_MAX_SETPOINT_GAIN_FACTOR * pidProfile->d_max_gain * pidProfile->d_max_advance / 100.0f / D_MAX_LOWPASS_HZ;
    // lowpass included inversely in gain since stronger lowpass decreases peak effect
#endif

#if defined(USE_AIRMODE_LPF)
    pidRuntime.airmodeThrottleOffsetLimit = pidProfile->transient_throttle_limit / 100.0f;
#endif

#ifdef USE_FEEDFORWARD
    pidRuntime.feedforwardTransition = pidProfile->feedforward_transition / 100.0f;
    pidRuntime.feedforwardTransitionInv = (pidProfile->feedforward_transition == 0) ? 0.0f : 100.0f / pidProfile->feedforward_transition;
    pidRuntime.feedforwardAveraging = pidProfile->feedforward_averaging;
    pidRuntime.feedforwardSmoothFactor = 1.0f - (0.01f * pidProfile->feedforward_smooth_factor);
    pidRuntime.feedforwardJitterFactor = pidProfile->feedforward_jitter_factor;
    pidRuntime.feedforwardJitterFactorInv = 1.0f / (1.0f + pidProfile->feedforward_jitter_factor);
    pidRuntime.feedforwardBoostFactor = 0.001f * pidProfile->feedforward_boost;
    pidRuntime.feedforwardMaxRateLimit = pidProfile->feedforward_max_rate_limit;
    pidRuntime.feedforwardInterpolate = !(rxRuntimeState.serialrxProvider == SERIALRX_CRSF);
    pidRuntime.feedforwardYawHoldTime = 0.001f * pidProfile->feedforward_yaw_hold_time; // input time constant in milliseconds, converted to seconds
    pidRuntime.feedforwardYawHoldGain = pidProfile->feedforward_yaw_hold_gain;
    // normalise/maintain boost when time constant is small, 1.5x at 50ms, 2x at 25ms, almost 3x at 10ms
    if (pidProfile->feedforward_yaw_hold_time < 100) {
        pidRuntime.feedforwardYawHoldGain *= 150.0f / (float)(pidProfile->feedforward_yaw_hold_time + 50);
    }
#endif

    pidRuntime.levelRaceMode = pidProfile->level_race_mode;
    pidRuntime.tpaBreakpoint = constrainf((pidProfile->tpa_breakpoint - PWM_RANGE_MIN) / 1000.0f, 0.0f, 0.99f);
    // default of 1350 returns 0.35. range limited to 0 to 0.99
    pidRuntime.tpaMultiplier = (pidProfile->tpa_rate / 100.0f) / (1.0f - pidRuntime.tpaBreakpoint);
    // it is assumed that tpaLowBreakpoint is always less than or equal to tpaBreakpoint
    pidRuntime.tpaLowBreakpoint = constrainf((pidProfile->tpa_low_breakpoint - PWM_RANGE_MIN) / 1000.0f, 0.01f, 1.0f);
    pidRuntime.tpaLowBreakpoint = MIN(pidRuntime.tpaLowBreakpoint, pidRuntime.tpaBreakpoint);
    pidRuntime.tpaLowMultiplier = pidProfile->tpa_low_rate / (100.0f * pidRuntime.tpaLowBreakpoint);
    pidRuntime.tpaLowAlways = pidProfile->tpa_low_always;

    pidRuntime.useEzDisarm = pidProfile->landing_disarm_threshold > 0;
    pidRuntime.landingDisarmThreshold = pidProfile->landing_disarm_threshold * 10.0f;

#ifdef USE_WING
    tpaSpeedInit(pidProfile);
#endif
}

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}
