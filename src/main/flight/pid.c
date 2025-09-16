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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/simplified_tuning.h"

#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/autopilot.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_DATA_ZERO_INIT uint32_t targetPidLooptime;
FAST_DATA_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];
FAST_DATA_ZERO_INIT pidRuntime_t pidRuntime;

#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
#endif

#if defined(USE_THROTTLE_BOOST)
FAST_DATA_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 4);

#ifndef DEFAULT_PID_PROCESS_DENOM
#define DEFAULT_PID_PROCESS_DENOM       1
#endif

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = DEFAULT_PID_PROCESS_DENOM,
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 20,  // throttle level % needed to accumulate deactivation time
    .runaway_takeoff_deactivate_delay = 500,    // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = DEFAULT_PID_PROCESS_DENOM,
);
#endif

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

#define CRASH_RECOVERY_DETECTION_DELAY_US 1000000  // 1 second delay before crash recovery detection is active after entering a self-level mode

#define LAUNCH_CONTROL_YAW_ITERM_LIMIT 50 // yaw iterm windup limit when launch mode is "FULL" (all axes)

#ifdef USE_ACC
#define IS_AXIS_IN_ANGLE_MODE(i) (pidRuntime.axisInAngleMode[(i)])
#else
#define IS_AXIS_IN_ANGLE_MODE(i) false
#endif // USE_ACC

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 11);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] =   PID_YAW_DEFAULT,
            [PID_LEVEL] = { 50, 75, 75, 50, 0 },
            [PID_MAG] =   { 40, 0, 0, 0, 0 },
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 100,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindup = 80,         // sets iTerm limit to this percentage below pidSumLimit
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .angle_limit = 60,
        .feedforward_transition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .anti_gravity_gain = 80,
        .crash_time = 500,          // ms
        .crash_delay = 0,           // ms
        .crash_recovery_angle = 10, // degrees
        .crash_recovery_rate = 100, // degrees/second
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
        .horizon_limit_degrees = 135,
        .horizon_ignore_sticks = false,
        .crash_limit_yaw = 200,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
        .abs_control_cutoff = 11,
        .dterm_lpf1_static_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
            // NOTE: dynamic lpf is enabled by default so this setting is actually
            // overridden and the static lowpass 1 is disabled. We can't set this
            // value to 0 otherwise Configurator versions 10.4 and earlier will also
            // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
        .dterm_lpf2_static_hz = DTERM_LPF2_HZ_DEFAULT,   // second Dterm LPF ON by default
        .dterm_lpf1_type = FILTER_PT1,
        .dterm_lpf2_type = FILTER_PT1,
        .dterm_lpf1_dyn_min_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
        .dterm_lpf1_dyn_max_hz = DTERM_LPF1_DYN_MAX_HZ_DEFAULT,
        .launchControlMode = LAUNCH_CONTROL_MODE_PITCHONLY,
        .launchControlThrottlePercent = 20,
        .launchControlAngleLimit = 0,
        .launchControlGain = 40,
        .launchControlAllowTriggerReset = true,
        .use_integrated_yaw = false,
        .integrated_yaw_relax = 200,
        .thrustLinearization = 0,
        .d_max = D_MAX_DEFAULT,
        .d_max_gain = 37,
        .d_max_advance = 20,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .transient_throttle_limit = 0,
        .profileName = { 0 },
        .dyn_idle_min_rpm = 0,
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 50,
        .dyn_idle_d_gain = 50,
        .dyn_idle_max_increase = 150,
        .feedforward_averaging = FEEDFORWARD_AVERAGING_2_POINT,
        .feedforward_max_rate_limit = 90,
        .feedforward_smooth_factor = 65,
        .feedforward_jitter_factor = 7,
        .feedforward_boost = 15,
        .dterm_lpf1_dyn_expo = 5,
        .level_race_mode = false,
        .vbat_sag_compensation = 0,
        .simplified_pids_mode = PID_SIMPLIFIED_TUNING_RPY,
        .simplified_master_multiplier = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_roll_pitch_ratio = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_i_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_d_gain = SIMPLIFIED_TUNING_D_DEFAULT,
        .simplified_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_d_max_gain = SIMPLIFIED_TUNING_D_DEFAULT,
        .simplified_feedforward_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_pitch_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_dterm_filter = true,
        .simplified_dterm_filter_multiplier = SIMPLIFIED_TUNING_DEFAULT,
        .anti_gravity_cutoff_hz = 5,
        .anti_gravity_p_gain = 100,
        .tpa_mode = TPA_MODE_D,
        .tpa_rate = 65,
        .tpa_breakpoint = 1350,
        .angle_feedforward_smoothing_ms = 80,
        .angle_earth_ref = 100,
        .horizon_delay_ms = 500, // 500ms time constant on any increase in horizon strength
        .tpa_low_rate = 20,
        .tpa_low_breakpoint = 1050,
        .tpa_low_always = 0,
        .ez_landing_threshold = 25,
        .ez_landing_limit = 15,
        .ez_landing_speed = 50,
        .spa_center = { 0, 0, 0 },
        .spa_width = { 0, 0, 0 },
        .spa_mode = { 0, 0, 0 },
        .landing_disarm_threshold = 0, // relatively safe values are around 100
        .feedforward_yaw_hold_gain = 15,  // zero disables; 15-20 is OK for 5in
        .feedforward_yaw_hold_time = 100,  // a value of 100 is a time constant of about 100ms, and is OK for a 5in; smaller values decay faster, eg for smaller props
        .tpa_curve_type = TPA_CURVE_CLASSIC,
        .tpa_curve_stall_throttle = 30,
        .tpa_curve_pid_thr0 = 200,
        .tpa_curve_pid_thr100 = 70,
        .tpa_curve_expo = 20,
        .tpa_speed_type = TPA_SPEED_BASIC,
        .tpa_speed_basic_delay = 1000,
        .tpa_speed_basic_gravity = 50,
        .tpa_speed_adv_prop_pitch = 370,
        .tpa_speed_adv_mass = 1000,
        .tpa_speed_adv_drag_k = 1000,
        .tpa_speed_adv_thrust = 2000,
        .tpa_speed_max_voltage = 2520,
        .tpa_speed_pitch_offset = 0,
        .yaw_type = YAW_TYPE_RUDDER,
        .angle_pitch_offset = 0,
        .chirp_lag_freq_hz = 3,
        .chirp_lead_freq_hz = 30,
        .chirp_amplitude_roll = 230,
        .chirp_amplitude_pitch = 230,
        .chirp_amplitude_yaw = 180,
        .chirp_frequency_start_deci_hz = 2,
        .chirp_frequency_end_deci_hz = 6000,
        .chirp_time_seconds = 20,
    );
}

static bool isTpaActive(tpaMode_e tpaMode, term_e term) {
    switch (tpaMode) {
    case TPA_MODE_PD:
        return term == TERM_P || term == TERM_D;
    case TPA_MODE_D:
        return term == TERM_D;
#ifdef USE_WING
    case TPA_MODE_PDS:
        return term == TERM_P || term == TERM_D || term == TERM_S;
#endif
    default:
        return false;
    }
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

// Scale factors to make best use of range with D_LPF debugging, aiming for max +/-16K as debug values are 16 bit
#define D_LPF_RAW_SCALE 25
#define D_LPF_PRE_TPA_SCALE 10

void pidSetItermAccelerator(float newItermAccelerator)
{
    pidRuntime.itermAccelerator = newItermAccelerator;
}

bool pidOsdAntiGravityActive(void)
{
    return (pidRuntime.itermAccelerator > pidRuntime.antiGravityOsdCutoff);
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidRuntime.pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}

#ifdef USE_WING
static float calcWingThrottle(void)
{
    float batteryThrottleFactor = 1.0f;
    if (pidRuntime.tpaSpeed.maxVoltage > 0.0f) {
        batteryThrottleFactor = getBatteryVoltageLatest() / 100.0f / pidRuntime.tpaSpeed.maxVoltage;
        batteryThrottleFactor = constrainf(batteryThrottleFactor, 0.0f, 1.0f);
    }

    return getMotorOutputRms() * batteryThrottleFactor;
}

static float calcWingAcceleration(float throttle, float pitchAngleRadians)
{
    const tpaSpeedParams_t *tpa = &pidRuntime.tpaSpeed;

    const float thrust = (throttle * throttle - throttle * tpa->speed * tpa->inversePropMaxSpeed) * tpa->twr * G_ACCELERATION;
    const float drag = tpa->speed * tpa->speed * tpa->dragMassRatio;
    const float gravity = G_ACCELERATION * sin_approx(pitchAngleRadians);

    return thrust - drag + gravity;
}

static float calcWingTpaArgument(void)
{
    const float t = calcWingThrottle();
    const float pitchRadians = DECIDEGREES_TO_RADIANS(attitude.values.pitch);
    const float rollRadians = DECIDEGREES_TO_RADIANS(attitude.values.roll);

    DEBUG_SET(DEBUG_TPA, 1, lrintf(attitude.values.roll)); // decidegrees
    DEBUG_SET(DEBUG_TPA, 2, lrintf(attitude.values.pitch)); // decidegrees
    DEBUG_SET(DEBUG_TPA, 3, lrintf(t * 1000.0f)); // calculated throttle in the range of 0 - 1000

    // pitchRadians is always -90 to 90 degrees. The bigger the ABS(pitch) the less portion of pitchOffset is needed.
    // If ABS(roll) > 90 degrees - flying inverted, then negative portion of pitchOffset is needed.
    // If ABS(roll) ~ 90 degrees - flying sideways, no pitchOffset is applied.
    const float correctedPitchAnge = pitchRadians + cos_approx(pitchRadians) * cos_approx(rollRadians) * pidRuntime.tpaSpeed.pitchOffset;

    const float a = calcWingAcceleration(t, correctedPitchAnge);

    pidRuntime.tpaSpeed.speed += a * pidRuntime.dT;
    pidRuntime.tpaSpeed.speed = MAX(0.0f, pidRuntime.tpaSpeed.speed);
    const float tpaArgument = constrainf(pidRuntime.tpaSpeed.speed / pidRuntime.tpaSpeed.maxSpeed, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_TPA, 4, lrintf(pidRuntime.tpaSpeed.speed * 10.0f));
    DEBUG_SET(DEBUG_TPA, 5, lrintf(tpaArgument * 1000.0f));

    return tpaArgument;
}

static void updateStermTpaFactor(int axis, float tpaFactor)
{
    float tpaFactorSterm = tpaFactor;
    if (pidRuntime.tpaCurveType == TPA_CURVE_HYPERBOLIC) {
        const float maxSterm = tpaFactorSterm * (float)currentPidProfile->pid[axis].S * S_TERM_SCALE;
        if (maxSterm > 1.0f) {
            tpaFactorSterm *=  1.0f / maxSterm;
        }
    }
    pidRuntime.tpaFactorSterm[axis] = tpaFactorSterm;
}

static void updateStermTpaFactors(void) {
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        float tpaFactor = pidRuntime.tpaFactor;
        if (i == FD_YAW && currentPidProfile->yaw_type == YAW_TYPE_DIFF_THRUST) {
            tpaFactor = pidRuntime.tpaFactorYaw;
        }
        updateStermTpaFactor(i, tpaFactor);
    }
}
#endif // USE_WING

static float wingAdjustSetpoint(float currentPidSetpoint, int axis)
{
#ifdef USE_WING
    float adjustedSetpoint = currentPidSetpoint;
    if (!IS_AXIS_IN_ANGLE_MODE(axis)) {
        const bool skipYaw = axis == FD_YAW && currentPidProfile->yaw_type == YAW_TYPE_DIFF_THRUST;
        if (pidRuntime.tpaFactorSterm[axis] > 0.0f && pidRuntime.tpaFactor > 0.0f && !skipYaw) {
            adjustedSetpoint = currentPidSetpoint * pidRuntime.tpaFactorSterm[axis] / pidRuntime.tpaFactor;
        }
    }

    DEBUG_SET(DEBUG_WING_SETPOINT, 2 * axis, lrintf(currentPidSetpoint));
    DEBUG_SET(DEBUG_WING_SETPOINT, 2 * axis + 1, lrintf(adjustedSetpoint));
    return adjustedSetpoint;
#else
    UNUSED(axis);
    return currentPidSetpoint;
#endif // USE_WING
}

static float getTpaFactorClassic(float tpaArgument)
{
    static bool isTpaLowFaded = false;
    bool isThrottlePastTpaLowBreakpoint = (tpaArgument >= pidRuntime.tpaLowBreakpoint || pidRuntime.tpaLowBreakpoint <= 0.01f);
    float tpaRate = 0.0f;
    if (isThrottlePastTpaLowBreakpoint || isTpaLowFaded) {
        tpaRate = pidRuntime.tpaMultiplier * fmaxf(tpaArgument - pidRuntime.tpaBreakpoint, 0.0f);
        if (!pidRuntime.tpaLowAlways && !isTpaLowFaded) {
            isTpaLowFaded = true;
        }
    } else {
        tpaRate = pidRuntime.tpaLowMultiplier * (pidRuntime.tpaLowBreakpoint - tpaArgument);
    }

    return 1.0f - tpaRate;
}

void pidUpdateTpaFactor(float throttle)
{
    throttle = constrainf(throttle, 0.0f, 1.0f);
    float tpaFactor;

#ifdef USE_WING
    const float tpaArgument = isFixedWing() ?  calcWingTpaArgument() : throttle;
#else
    const float tpaArgument = throttle;
#endif

#ifdef USE_ADVANCED_TPA
    switch (pidRuntime.tpaCurveType) {
    case TPA_CURVE_HYPERBOLIC:
        tpaFactor = pwlInterpolate(&pidRuntime.tpaCurvePwl, tpaArgument);
        break;
    case TPA_CURVE_CLASSIC:
    default:
        tpaFactor = getTpaFactorClassic(tpaArgument);
    }
#else
    tpaFactor = getTpaFactorClassic(tpaArgument);
#endif

    DEBUG_SET(DEBUG_TPA, 0, lrintf(tpaFactor * 1000));
    pidRuntime.tpaFactor = tpaFactor;

#ifdef USE_WING
    switch (currentPidProfile->yaw_type) {
    case YAW_TYPE_DIFF_THRUST:
        pidRuntime.tpaFactorYaw = getTpaFactorClassic(tpaArgument);
        break;
    case YAW_TYPE_RUDDER:
    default:
        pidRuntime.tpaFactorYaw = pidRuntime.tpaFactor;
        break;
    }
    updateStermTpaFactors();
#endif // USE_WING
}

void pidUpdateAntiGravityThrottleFilter(float throttle)
{
    static float previousThrottle = 0.0f;
    const float throttleInv = 1.0f - throttle;
    float throttleDerivative = fabsf(throttle - previousThrottle) * pidRuntime.pidFrequency;
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 0, lrintf(throttleDerivative * 100));
    throttleDerivative *= throttleInv * throttleInv;
    // generally focus on the low throttle period
    if (throttle > previousThrottle) {
        throttleDerivative *= throttleInv * 0.5f;
        // when increasing throttle, focus even more on the low throttle range
    }
    previousThrottle = throttle;
    throttleDerivative = pt2FilterApply(&pidRuntime.antiGravityLpf, throttleDerivative);
    // lower cutoff suppresses peaks relative to troughs and prolongs the effects
    // PT2 smoothing of throttle derivative.
    // 6 is a typical value for the peak boost factor with default cutoff of 6Hz
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(throttleDerivative * 100));
    pidRuntime.antiGravityThrottleD = throttleDerivative;
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    pidRuntime.acroTrainerAxisState[FD_ROLL] = 0;
    pidRuntime.acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

#ifdef USE_THRUST_LINEARIZATION
float pidCompensateThrustLinearization(float throttle)
{
    if (pidRuntime.thrustLinearization != 0.0f) {
        // for whoops where a lot of TL is needed, allow more throttle boost
        const float throttleReversed = (1.0f - throttle);
        throttle /= 1.0f + pidRuntime.throttleCompensateAmount * sq(throttleReversed);
    }
    return throttle;
}

float pidApplyThrustLinearization(float motorOutput)
{
    motorOutput *= 1.0f + pidRuntime.thrustLinearization * sq(1.0f - motorOutput);
    return motorOutput;
}
#endif

#if defined(USE_ACC)
// Calculate strength of horizon leveling; 0 = none, 1.0 = most leveling
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float calcHorizonLevelStrength(void)
{
    const float currentInclination = MAX(abs(attitude.values.roll), abs(attitude.values.pitch)) * 0.1f;
    // 0 when level, 90 when vertical, 180 when inverted (degrees):
    float absMaxStickDeflection = MAX(fabsf(getRcDeflection(FD_ROLL)), fabsf(getRcDeflection(FD_PITCH)));
    // 0-1, smoothed if RC smoothing is enabled

    float horizonLevelStrength = MAX((pidRuntime.horizonLimitDegrees - currentInclination) * pidRuntime.horizonLimitDegreesInv, 0.0f);
    // 1.0 when attitude is 'flat', 0 when angle is equal to, or greater than, horizonLimitDegrees
    horizonLevelStrength *= MAX((pidRuntime.horizonLimitSticks - absMaxStickDeflection) * pidRuntime.horizonLimitSticksInv, pidRuntime.horizonIgnoreSticks);
    // use the value of horizonIgnoreSticks to enable/disable this effect.
    // value should be 1.0 at center stick, 0.0 at max stick deflection:
    horizonLevelStrength *= pidRuntime.horizonGain;

    if (pidRuntime.horizonDelayMs) {
        const float horizonLevelStrengthSmoothed = pt1FilterApply(&pidRuntime.horizonSmoothingPt1, horizonLevelStrength);
        horizonLevelStrength = MIN(horizonLevelStrength, horizonLevelStrengthSmoothed);
    }
    return horizonLevelStrength;
    // 1 means full levelling, 0 means none
}

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim,
                                                        float currentPidSetpoint, float horizonLevelStrength)
{
    // Applies only to axes that are in Angle mode
    // We now use Acro Rates, transformed into the range +/- 1, to provide setpoints
    float angleLimit = pidProfile->angle_limit;
    float angleFeedforward = 0.0f;
    // if user changes rates profile, update the max setpoint for angle mode
    const float maxSetpointRateInv = 1.0f / getMaxRcRate(axis);

#ifdef USE_FEEDFORWARD
    angleFeedforward = angleLimit * getFeedforward(axis) * pidRuntime.angleFeedforwardGain * maxSetpointRateInv;
    //  angle feedforward must be heavily filtered, at the PID loop rate, with limited user control over time constant
    // it MUST be very delayed to avoid early overshoot and being too aggressive
    angleFeedforward = pt3FilterApply(&pidRuntime.angleFeedforwardPt3[axis], angleFeedforward);
#endif

    float angleTarget = angleLimit * currentPidSetpoint * maxSetpointRateInv;
    // use acro rates for the angle target in both horizon and angle modes, converted to -1 to +1 range using maxRate

#ifdef USE_WING
    if (axis == FD_PITCH) {
        angleTarget += (float)pidProfile->angle_pitch_offset / 10.0f;
    }
#endif // USE_WING

#ifdef USE_GPS_RESCUE
    angleTarget += gpsRescueAngle[axis] / 100.0f; // Angle is in centidegrees, stepped on roll at 10Hz but not on pitch
#endif
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        angleFeedforward = 0.0f; // otherwise the lag of the PT3 carries recent stick inputs into the hold
        if (isAutopilotInControl()) {
            // sticks are not deflected
            angleTarget = autopilotAngle[axis]; // autopilotAngle in degrees
            angleLimit = 85.0f; // allow autopilot to use whatever angle it needs to stop
        }
        // limit pilot requested angle to half the autopilot angle to avoid excess speed and chaotic stops
        angleLimit = fminf(0.5f * autopilotConfig()->maxAngle, angleLimit);
    }
#endif

    angleTarget = constrainf(angleTarget, -angleLimit, angleLimit);

    const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f; // stepped at 500hz with some 4ms flat spots
    const float errorAngle = angleTarget - currentAngle;
    float angleRate = errorAngle * pidRuntime.angleGain + angleFeedforward;

    // minimise cross-axis wobble due to faster yaw responses than roll or pitch, and make co-ordinated yaw turns
    // by compensating for the effect of yaw on roll while pitched, and on pitch while rolled
    // earthRef code here takes about 76 cycles, if conditional on angleEarthRef it takes about 100.  sin_approx costs most of those cycles.
    float sinAngle = sin_approx(DEGREES_TO_RADIANS(pidRuntime.angleTarget[axis == FD_ROLL ? FD_PITCH : FD_ROLL]));
    sinAngle *= (axis == FD_ROLL) ? -1.0f : 1.0f; // must be negative for Roll
    const float earthRefGain = FLIGHT_MODE(GPS_RESCUE_MODE | ALT_HOLD_MODE) ? 1.0f : pidRuntime.angleEarthRef;
    angleRate += pidRuntime.angleYawSetpoint * sinAngle * earthRefGain;
    pidRuntime.angleTarget[axis] = angleTarget;  // set target for alternate axis to current axis, for use in preceding calculation

    // smooth final angle rate output to clean up attitude signal steps (500hz), GPS steps (10 or 100hz), RC steps etc
    // this filter runs at ATTITUDE_CUTOFF_HZ, currently 50hz, so GPS roll may be a bit steppy
    angleRate = pt3FilterApply(&pidRuntime.attitudeFilter[axis], angleRate);

    if (FLIGHT_MODE(ANGLE_MODE| GPS_RESCUE_MODE | POS_HOLD_MODE)) {
        currentPidSetpoint = angleRate;
    } else {
        // can only be HORIZON mode - crossfade Angle rate and Acro rate
        currentPidSetpoint = currentPidSetpoint * (1.0f - horizonLevelStrength) + angleRate * horizonLevelStrength;
    }

    //logging
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_ANGLE_MODE, 0, lrintf(angleTarget * 10.0f)); // target angle
        DEBUG_SET(DEBUG_ANGLE_MODE, 1, lrintf(errorAngle * pidRuntime.angleGain * 10.0f)); // un-smoothed error correction in degrees
        DEBUG_SET(DEBUG_ANGLE_MODE, 2, lrintf(angleFeedforward * 10.0f)); // feedforward amount in degrees
        DEBUG_SET(DEBUG_ANGLE_MODE, 3, lrintf(currentAngle * 10.0f)); // angle returned

        DEBUG_SET(DEBUG_ANGLE_TARGET, 0, lrintf(angleTarget * 10.0f));
        DEBUG_SET(DEBUG_ANGLE_TARGET, 1, lrintf(sinAngle * 10.0f)); // modification factor from earthRef
        // debug ANGLE_TARGET 2 is yaw attenuation
        DEBUG_SET(DEBUG_ANGLE_TARGET, 3, lrintf(currentAngle * 10.0f)); // angle returned
    }

    DEBUG_SET(DEBUG_CURRENT_ANGLE, axis, lrintf(currentAngle * 10.0f)); // current angle
    return currentPidSetpoint;
}

static FAST_CODE_NOINLINE void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate)
{
    if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeDelayUs) {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP) {
            BEEP_ON;
        }
        if (axis == FD_YAW) {
            *errorRate = constrainf(*errorRate, -pidRuntime.crashLimitYaw, pidRuntime.crashLimitYaw);
        } else {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            if (sensors(SENSOR_ACC)) {
                // errorAngle is deviation from horizontal
                const float errorAngle =  -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                *currentPidSetpoint = errorAngle * pidRuntime.angleGain;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
        // reset iterm, since accumulated error before crash is now meaningless
        // and iterm windup during crash recovery can be extreme, especially on yaw axis
        pidData[axis].I = 0.0f;
        if (cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeLimitUs
            || (getMotorMixRange() < 1.0f
                   && fabsf(gyro.gyroADCf[FD_ROLL]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_PITCH]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_YAW]) < pidRuntime.crashRecoveryRate)) {
            if (sensors(SENSOR_ACC)) {
                // check aircraft nearly level
                if (abs(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < pidRuntime.crashRecoveryAngleDeciDegrees
                   && abs(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < pidRuntime.crashRecoveryAngleDeciDegrees) {
                    pidRuntime.inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            } else {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
}

static FAST_CODE_NOINLINE void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected()) {
        if (ARMING_FLAG(ARMED)) {
            if (getMotorMixRange() >= 1.0f && !pidRuntime.inCrashRecoveryMode
                && fabsf(delta) > pidRuntime.crashDtermThreshold
                && fabsf(errorRate) > pidRuntime.crashGyroThreshold
                && fabsf(getSetpointRate(axis)) < pidRuntime.crashSetpointThreshold) {
                if (crash_recovery == PID_CRASH_RECOVERY_DISARM) {
                    setArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
                    disarm(DISARM_REASON_CRASH_PROTECTION);
                } else {
                    pidRuntime.inCrashRecoveryMode = true;
                    pidRuntime.crashDetectedAtUs = currentTimeUs;
                }
            }
            if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) < pidRuntime.crashTimeDelayUs && (fabsf(errorRate) < pidRuntime.crashGyroThreshold
                || fabsf(getSetpointRate(axis)) > pidRuntime.crashSetpointThreshold)) {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else if (pidRuntime.inCrashRecoveryMode) {
            pidRuntime.inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}
#endif // USE_ACC

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

    if (!FLIGHT_MODE(ANGLE_MODE  | HORIZON_MODE | GPS_RESCUE_MODE | ALT_HOLD_MODE | POS_HOLD_MODE)) {
        bool resetIterm = false;
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((pidRuntime.acroTrainerAxisState[axis] != 0) && (pidRuntime.acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            pidRuntime.acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > pidRuntime.acroTrainerAngleLimit) && (pidRuntime.acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                pidRuntime.acroTrainerAxisState[axis] = angleSign;
                resetIterm = true;
            }
        }

        if (pidRuntime.acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((pidRuntime.acroTrainerAngleLimit * angleSign) - currentAngle) * pidRuntime.acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {

        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * pidRuntime.acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > pidRuntime.acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((pidRuntime.acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * pidRuntime.acroTrainerGain;
                resetIterm = true;
            }
        }

        if (resetIterm) {
            pidData[axis].I = 0;
        }

        if (axis == pidRuntime.acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, pidRuntime.acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > pidRuntime.maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static void rotateVector(float v[XYZ_AXIS_COUNT], const float rotation[XYZ_AXIS_COUNT])
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

STATIC_UNIT_TESTED void rotateItermAndAxisError(void)
{
    if (pidRuntime.itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR
#endif
        ) {
        const float gyroToAngle = pidRuntime.dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
            rotateVector(axisError, rotationRads);
        }
#endif
        if (pidRuntime.itermRotation) {
            float v[XYZ_AXIS_COUNT];
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

#if defined(USE_ITERM_RELAX)
#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate)
{
    if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
        const float setpointLpf = pt1FilterApply(&pidRuntime.acLpf[axis], *currentPidSetpoint);
        const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
        float acErrorRate = 0;
        const float gmaxac = setpointLpf + 2 * setpointHpf;
        const float gminac = setpointLpf - 2 * setpointHpf;
        if (gyroRate >= gminac && gyroRate <= gmaxac) {
            const float acErrorRate1 = gmaxac - gyroRate;
            const float acErrorRate2 = gminac - gyroRate;
            if (acErrorRate1 * axisError[axis] < 0) {
                acErrorRate = acErrorRate1;
            } else {
                acErrorRate = acErrorRate2;
            }
            if (fabsf(acErrorRate * pidRuntime.dT) > fabsf(axisError[axis]) ) {
                acErrorRate = -axisError[axis] * pidRuntime.pidFrequency;
            }
        } else {
            acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
        }

        if (wasThrottleRaised()) {
            axisError[axis] = constrainf(axisError[axis] + acErrorRate * pidRuntime.dT,
                -pidRuntime.acErrorLimit, pidRuntime.acErrorLimit);
            const float acCorrection = constrainf(axisError[axis] * pidRuntime.acGain, -pidRuntime.acLimit, pidRuntime.acLimit);
            *currentPidSetpoint += acCorrection;
            *itermErrorRate += acCorrection;
            DEBUG_SET(DEBUG_AC_CORRECTION, axis, lrintf(acCorrection * 10));
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(acCorrection * 10));
            }
        }
        DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(axisError[axis] * 10));
    }
}
#endif

STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&pidRuntime.windupLpf[axis], *currentPidSetpoint);
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

    if (pidRuntime.itermRelax) {
        if (axis < FD_YAW || pidRuntime.itermRelax == ITERM_RELAX_RPY || pidRuntime.itermRelax == ITERM_RELAX_RPY_INC) {
            float itermRelaxThreshold = ITERM_RELAX_SETPOINT_THRESHOLD;
            if (FLIGHT_MODE(ANGLE_MODE)) {
                itermRelaxThreshold *= 0.2f;
            }
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / itermRelaxThreshold);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((pidRuntime.itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                // Do Nothing, use the precalculed itermErrorRate
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_SETPOINT) {
                *itermErrorRate *= itermRelaxFactor;
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_GYRO ) {
                *itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorRate));
            }
        }

#if defined(USE_ABSOLUTE_CONTROL)
        applyAbsoluteControl(axis, gyroRate, currentPidSetpoint, itermErrorRate);
#endif
    }
}
#endif

#ifdef USE_AIRMODE_LPF
void pidUpdateAirmodeLpf(float currentOffset)
{
    if (pidRuntime.airmodeThrottleOffsetLimit == 0.0f) {
        return;
    }

    float offsetHpf = currentOffset * 2.5f;
    offsetHpf = offsetHpf - pt1FilterApply(&pidRuntime.airmodeThrottleLpf2, offsetHpf);

    // During high frequency oscillation 2 * currentOffset averages to the offset required to avoid mirroring of the waveform
    pt1FilterApply(&pidRuntime.airmodeThrottleLpf1, offsetHpf);
    // Bring offset up immediately so the filter only applies to the decline
    if (currentOffset * pidRuntime.airmodeThrottleLpf1.state >= 0 && fabsf(currentOffset) > pidRuntime.airmodeThrottleLpf1.state) {
        pidRuntime.airmodeThrottleLpf1.state = currentOffset;
    }
    pidRuntime.airmodeThrottleLpf1.state = constrainf(pidRuntime.airmodeThrottleLpf1.state, -pidRuntime.airmodeThrottleOffsetLimit, pidRuntime.airmodeThrottleOffsetLimit);
}

float pidGetAirmodeThrottleOffset(void)
{
    return pidRuntime.airmodeThrottleLpf1.state;
}
#endif

static FAST_CODE_NOINLINE void disarmOnImpact(void)
{
    // if, being armed, and after takeoff...
    if (wasThrottleRaised()
        // and, either sticks are centred and throttle zeroed,
        && ((getMaxRcDeflectionAbs() < 0.05f && mixerGetRcThrottle() < 0.05f)
#ifdef USE_ALTITUDE_HOLD
            // or, in altitude hold mode, where throttle can be non-zero
            || FLIGHT_MODE(ALT_HOLD_MODE)
#endif
        )) {
        // increase sensitivity by 50% when low and in altitude hold or failsafe landing
        // for more reliable disarm with gentle controlled landings
        float lowAltitudeSensitivity = 1.0f;
#ifdef USE_ALTITUDE_HOLD
        lowAltitudeSensitivity = (FLIGHT_MODE(ALT_HOLD_MODE) && isBelowLandingAltitude()) ? 1.5f : 1.0f;
#endif
        // and disarm if jerk exceeds threshold...
        if ((acc.jerkMagnitude * lowAltitudeSensitivity) > pidRuntime.landingDisarmThreshold) {
            // then disarm
            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH); // NB: need a better message
            disarm(DISARM_REASON_LANDING);
            // note: threshold should be high enough to avoid unwanted disarms in the air on throttle chops, eg around 10
        }
    }
    DEBUG_SET(DEBUG_EZLANDING, 6, lrintf(getMaxRcDeflectionAbs() * 100.0f));
    DEBUG_SET(DEBUG_EZLANDING, 7, lrintf(acc.jerkMagnitude * 1e3f));
}

#ifdef USE_LAUNCH_CONTROL
#define LAUNCH_CONTROL_MAX_RATE 100.0f
#define LAUNCH_CONTROL_MIN_RATE 5.0f
#define LAUNCH_CONTROL_ANGLE_WINDOW 10.0f  // The remaining angle degrees where rate dampening starts

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
static FAST_CODE_NOINLINE float applyLaunchControl(int axis, const rollAndPitchTrims_t *angleTrim)
{
    float ret = 0.0f;

    // Scale the rates based on stick deflection only. Fixed rates with a max of 100deg/sec
    // reached at 50% stick deflection. This keeps the launch control positioning consistent
    // regardless of the user's rates.
    if ((axis == FD_PITCH) || (pidRuntime.launchControlMode != LAUNCH_CONTROL_MODE_PITCHONLY)) {
        const float stickDeflection = constrainf(getRcDeflection(axis), -0.5f, 0.5f);
        ret = LAUNCH_CONTROL_MAX_RATE * stickDeflection * 2;
    }

#if defined(USE_ACC)
    // If ACC is enabled and a limit angle is set, then try to limit forward tilt
    // to that angle and slow down the rate as the limit is approached to reduce overshoot
    if ((axis == FD_PITCH) && (pidRuntime.launchControlAngleLimit > 0) && (ret > 0)) {
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        if (currentAngle >= pidRuntime.launchControlAngleLimit) {
            ret = 0.0f;
        } else {
            //for the last 10 degrees scale the rate from the current input to 5 dps
            const float angleDelta = pidRuntime.launchControlAngleLimit - currentAngle;
            if (angleDelta <= LAUNCH_CONTROL_ANGLE_WINDOW) {
                ret = scaleRangef(angleDelta, 0, LAUNCH_CONTROL_ANGLE_WINDOW, LAUNCH_CONTROL_MIN_RATE, ret);
            }
        }
    }
#else
    UNUSED(angleTrim);
#endif

    return ret;
}
#endif

static float getTpaFactor(const pidProfile_t *pidProfile, int axis, term_e term)
{
    float tpaFactor = pidRuntime.tpaFactor;

#ifdef USE_WING
    if (axis == FD_YAW) {
        tpaFactor = pidRuntime.tpaFactorYaw;
    }
#else
    UNUSED(axis);
#endif

    const bool tpaActive = isTpaActive(pidProfile->tpa_mode, term);
    switch (term) {
    case TERM_P:
        return tpaActive ? tpaFactor : 1.0f;
    case TERM_D:
        return tpaFactor;
#ifdef USE_WING
    case TERM_S:
        return tpaActive ? pidRuntime.tpaFactorSterm[axis] : 1.0f;
#endif
    default:
        return 1.0f;
    }
}

static float getSterm(int axis, const pidProfile_t *pidProfile, float setpoint)
{
#ifdef USE_WING
    float sTerm = setpoint / getMaxRcRate(axis) * 1000.0f *
        (float)pidProfile->pid[axis].S * S_TERM_SCALE;

    DEBUG_SET(DEBUG_S_TERM, 2 * axis, lrintf(sTerm));
    sTerm *= getTpaFactor(pidProfile, axis, TERM_S);
    DEBUG_SET(DEBUG_S_TERM, 2 * axis + 1, lrintf(sTerm));

    return sTerm;
#else
    UNUSED(axis);
    UNUSED(pidProfile);
    UNUSED(setpoint);
    return 0.0f;
#endif
}

NOINLINE static void calculateSpaValues(const pidProfile_t *pidProfile)
{
#ifdef USE_WING
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        float currentRate = getSetpointRate(axis);
        pidRuntime.spa[axis] = 1.0f - smoothStepUpTransition(
            fabsf(currentRate), pidProfile->spa_center[axis], pidProfile->spa_width[axis]);
        DEBUG_SET(DEBUG_SPA, axis, lrintf(pidRuntime.spa[axis] * 1000));
    }
#else
    UNUSED(pidProfile);
#endif // USE_WING
}

NOINLINE static void applySpa(int axis, const pidProfile_t *pidProfile)
{
#ifdef USE_WING
    spaMode_e mode = pidProfile->spa_mode[axis];

    if (pidRuntime.axisInAngleMode[axis]) {
        mode = SPA_MODE_OFF;
    }

    switch(mode) {
        case SPA_MODE_PID:
            pidData[axis].P *= pidRuntime.spa[axis];
            pidData[axis].D *= pidRuntime.spa[axis];
            pidData[axis].I *= pidRuntime.spa[axis];
            break;
        case SPA_MODE_I:
            pidData[axis].I *= pidRuntime.spa[axis];
            break;
        case SPA_MODE_PD_I_FREEZE:
            pidData[axis].P *= pidRuntime.spa[axis];
            pidData[axis].D *= pidRuntime.spa[axis];
            break;
        case SPA_MODE_I_FREEZE:
        case SPA_MODE_OFF:
        default:
            break;
    }
#else
    UNUSED(axis);
    UNUSED(pidProfile);
#endif // USE_WING
}

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    static float previousGyroRateDterm[XYZ_AXIS_COUNT];
    static float previousRawGyroRateDterm[XYZ_AXIS_COUNT];

    calculateSpaValues(pidProfile);

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinActive = gyroYawSpinDetected();
#endif

    const bool launchControlActive = isLaunchControlActive();

#if defined(USE_ACC)
    static timeUs_t levelModeStartTimeUs = 0;
    static bool prevExternalAngleRequest = false;
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    float horizonLevelStrength = 0.0f;

    const bool isExternalAngleModeRequest = FLIGHT_MODE(GPS_RESCUE_MODE)
#ifdef USE_ALTITUDE_HOLD
                || FLIGHT_MODE(ALT_HOLD_MODE) // todo - check if this is needed
#endif
#ifdef USE_POSITION_HOLD
                || FLIGHT_MODE(POS_HOLD_MODE)
#endif
                ;
    levelMode_e levelMode;
    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | GPS_RESCUE_MODE)) {
        if (pidRuntime.levelRaceMode && !isExternalAngleModeRequest) {
            levelMode = LEVEL_MODE_R;
        } else {
            levelMode = LEVEL_MODE_RP;
        }

        // Keep track of when we entered a self-level mode so that we can
        // add a guard time before crash recovery can activate.
        // Also reset the guard time whenever GPS Rescue is activated.
        if ((levelModeStartTimeUs == 0) || (isExternalAngleModeRequest && !prevExternalAngleRequest)) {
            levelModeStartTimeUs = currentTimeUs;
        }

        // Calc horizonLevelStrength if needed
        if (FLIGHT_MODE(HORIZON_MODE)) {
            horizonLevelStrength = calcHorizonLevelStrength();
        }
    } else {
        levelMode = LEVEL_MODE_OFF;
        levelModeStartTimeUs = 0;
    }

    prevExternalAngleRequest = isExternalAngleModeRequest;
#else
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);
#endif

    // Anti Gravity
    if (pidRuntime.antiGravityEnabled) {
        pidRuntime.antiGravityThrottleD *= pidRuntime.antiGravityGain;
        // used later to increase pTerm
        pidRuntime.itermAccelerator = pidRuntime.antiGravityThrottleD * ANTIGRAVITY_KI;
    } else {
        pidRuntime.antiGravityThrottleD = 0.0f;
        pidRuntime.itermAccelerator = 0.0f;
    }
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 2, lrintf((1 + (pidRuntime.itermAccelerator / pidRuntime.pidCoefficient[FD_PITCH].Ki)) * 1000));
    // amount of antigravity added relative to user's pitch iTerm coefficient
    // used later to increase iTerm

    // Precalculate gyro delta for D-term here, this allows loop unrolling
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];

        // Log the unfiltered D for ROLL and PITCH
        if (debugMode == DEBUG_D_LPF && axis != FD_YAW) {
            const float delta = (previousRawGyroRateDterm[axis] - gyroRateDterm[axis]) * pidRuntime.pidFrequency / D_LPF_RAW_SCALE;
            previousRawGyroRateDterm[axis] = gyroRateDterm[axis];
            DEBUG_SET(DEBUG_D_LPF, axis, lrintf(delta)); // debug d_lpf 2 and 3 used for pre-TPA D
        }

        gyroRateDterm[axis] = pidRuntime.dtermNotchApplyFn((filter_t *) &pidRuntime.dtermNotch[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &pidRuntime.dtermLowpass2[axis], gyroRateDterm[axis]);
    }

    rotateItermAndAxisError();

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif

    if (pidRuntime.useEzDisarm) {
        disarmOnImpact();
    }

#ifdef USE_CHIRP

    static int chirpAxis = 0;
    static bool shouldChirpAxisToggle = false;

    float chirp = 0.0f;
    float sinarg = 0.0f;
    if (FLIGHT_MODE(CHIRP_MODE)) {
        shouldChirpAxisToggle = true;  // advance chirp axis on next !CHIRP_MODE
        // update chirp signal
        if (chirpUpdate(&pidRuntime.chirp)) {
            chirp = pidRuntime.chirp.exc;
            sinarg = pidRuntime.chirp.sinarg;
        }
    } else {
        if (shouldChirpAxisToggle) {
            // toggle chirp signal logic and increment to next axis for next run
            shouldChirpAxisToggle = false;
            chirpAxis = (++chirpAxis > FD_YAW) ? 0 : chirpAxis;
            // reset chirp signal generator
            chirpReset(&pidRuntime.chirp);
        }
    }

    // input / excitation shaping
    float chirpFiltered  = phaseCompApply(&pidRuntime.chirpFilter, chirp);

    // ToDo: check if this can be reconstructed offline for rotating filter and if so, remove the debug
    // fit (0...2*pi) into int16_t (-32768 to 32767)
    DEBUG_SET(DEBUG_CHIRP, 0, lrintf(5.0e3f * sinarg));

#endif // USE_CHIRP

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

#ifdef USE_CHIRP
        float currentChirp = 0.0f;
        if(axis == chirpAxis){
            currentChirp = pidRuntime.chirpAmplitude[axis] * chirpFiltered;
        }
#endif // USE_CHIRP

        float currentPidSetpoint = getSetpointRate(axis);
        if (pidRuntime.maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }
        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        // When Race Mode is active PITCH control is also GYRO based in level or horizon mode
#if defined(USE_ACC)
        pidRuntime.axisInAngleMode[axis] = false;
        if (axis < FD_YAW) {
            if (levelMode == LEVEL_MODE_RP || (levelMode == LEVEL_MODE_R && axis == FD_ROLL)) {
                pidRuntime.axisInAngleMode[axis] = true;
                currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint, horizonLevelStrength);
            }
        } else { // yaw axis only
            if (levelMode == LEVEL_MODE_RP) {
                // if earth referencing is requested, attenuate yaw axis setpoint when pitched or rolled
                // and send yawSetpoint to Angle code to modulate pitch and roll
                // code cost is 107 cycles when earthRef enabled, 20 otherwise, nearly all in cos_approx
                const float earthRefGain = FLIGHT_MODE(GPS_RESCUE_MODE) ? 1.0f : pidRuntime.angleEarthRef;
                if (earthRefGain) {
                    pidRuntime.angleYawSetpoint = currentPidSetpoint;
                    float maxAngleTargetAbs = earthRefGain * fmaxf( fabsf(pidRuntime.angleTarget[FD_ROLL]), fabsf(pidRuntime.angleTarget[FD_PITCH]) );
                    maxAngleTargetAbs *= (FLIGHT_MODE(HORIZON_MODE)) ? horizonLevelStrength : 1.0f;
                    // reduce compensation whenever Horizon uses less levelling
                    currentPidSetpoint *= cos_approx(DEGREES_TO_RADIANS(maxAngleTargetAbs));
                    DEBUG_SET(DEBUG_ANGLE_TARGET, 2, currentPidSetpoint); // yaw setpoint after attenuation
                }
            }
        }
#endif

        const float currentPidSetpointBeforeWingAdjust = currentPidSetpoint;
        currentPidSetpoint = wingAdjustSetpoint(currentPidSetpoint, axis);

#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && pidRuntime.acroTrainerActive && !pidRuntime.inCrashRecoveryMode && !launchControlActive) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
#endif // USE_ACRO_TRAINER

#ifdef USE_LAUNCH_CONTROL
        if (launchControlActive) {
#if defined(USE_ACC)
            currentPidSetpoint = applyLaunchControl(axis, angleTrim);
#else
            currentPidSetpoint = applyLaunchControl(axis, NULL);
#endif
        }
#endif

        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && yawSpinActive) {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
#ifdef USE_CHIRP
        currentPidSetpoint += currentChirp;
#endif // USE_CHIRP
        float errorRate = currentPidSetpoint - gyroRate; // r - y
#if defined(USE_ACC)
        handleCrashRecovery(
            pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, gyroRate,
            &currentPidSetpoint, &errorRate);
#endif

        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;

#ifdef USE_ABSOLUTE_CONTROL
        const float uncorrectedSetpoint = currentPidSetpoint;
#endif

#if defined(USE_ITERM_RELAX)
        if (!launchControlActive && !pidRuntime.inCrashRecoveryMode) {
            applyItermRelax(axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
        }
#endif
#ifdef USE_ABSOLUTE_CONTROL
        const float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;
#endif

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------

        // -----calculate P component
        pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate * getTpaFactor(pidProfile, axis, TERM_P);
        if (axis == FD_YAW) {
            pidData[axis].P = pidRuntime.ptermYawLowpassApplyFn((filter_t *) &pidRuntime.ptermYawLowpass, pidData[axis].P);
        }

        // -----calculate I component
        float Ki = pidRuntime.pidCoefficient[axis].Ki;
        float itermLimit = pidRuntime.itermLimit; // windup fraction of pidSumLimit

#ifdef USE_LAUNCH_CONTROL
        // if launch control is active override the iterm gains and apply iterm windup protection to all axes
        if (launchControlActive) {
            Ki = pidRuntime.launchControlKi;
        } else
#endif
        {
            // yaw iTerm has it's own limit based on pidSumLimitYaw
            if (axis == FD_YAW) {
                itermLimit = pidRuntime.itermLimitYaw; // windup fraction of pidSumLimitYaw
                // note that this is a stronger limit than previously
                pidRuntime.itermAccelerator = 0.0f; // no antigravity on yaw iTerm
            }
        }

        float iTermChange = (Ki + pidRuntime.itermAccelerator) * pidRuntime.dT * itermErrorRate;
#ifdef USE_WING
        if (pidProfile->spa_mode[axis] != SPA_MODE_OFF) {
            // slowing down I-term change, or even making it zero if setpoint is high enough
            iTermChange *= pidRuntime.spa[axis];
        }
#endif // USE_WING

        pidData[axis].I = constrainf(previousIterm + iTermChange, -itermLimit, itermLimit);

        // -----calculate D component

        float pidSetpointDelta = 0;

#if defined(USE_FEEDFORWARD) && defined(USE_ACC)
        if (FLIGHT_MODE(ANGLE_MODE) && pidRuntime.axisInAngleMode[axis]) {
            // this axis is fully under self-levelling control
            // it will already have stick based feedforward applied in the input to their angle setpoint
            // a simple setpoint Delta can be used to for PID feedforward element for motor lag on these axes
            // however RC steps come in, via angle setpoint
            // and setpoint RC smoothing must have a cutoff half normal to remove those steps completely
            // the RC stepping does not come in via the feedforward, which is very well smoothed already
            // if uncommented, and the forcing to zero is removed, the two following lines will restore PID feedforward to angle mode axes
            // but for now let's see how we go without it (which was the case before 4.5 anyway)
//            pidSetpointDelta = currentPidSetpoint - pidRuntime.previousPidSetpoint[axis];
//            pidSetpointDelta *= pidRuntime.pidFrequency * pidRuntime.angleFeedforwardGain;
            pidSetpointDelta = 0.0f;
        } else {
            // the axis is operating as a normal acro axis, so use normal feedforard from rc.c
            pidSetpointDelta = getFeedforward(axis);
        }
#endif
        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint; // this is the value sent to blackbox, and used for D-max setpoint

        // disable D if launch control is active
        if ((pidRuntime.pidCoefficient[axis].Kd > 0) && !launchControlActive) {
            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float delta = - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidRuntime.pidFrequency;
            float preTpaD = pidRuntime.pidCoefficient[axis].Kd * delta;

#if defined(USE_ACC)
            if (cmpTimeUs(currentTimeUs, levelModeStartTimeUs) > CRASH_RECOVERY_DETECTION_DELAY_US) {
                detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, delta, errorRate);
            }
#endif

#ifdef USE_D_MAX
            float dMaxMultiplier = 1.0f;
            if (pidRuntime.dMaxPercent[axis] > 1.0f) {
                float dMaxGyroFactor = pt2FilterApply(&pidRuntime.dMaxRange[axis], delta);
                dMaxGyroFactor = fabsf(dMaxGyroFactor) * pidRuntime.dMaxGyroGain;
                const float dMaxSetpointFactor = fabsf(pidSetpointDelta) * pidRuntime.dMaxSetpointGain;
                const float dMaxBoost = fmaxf(dMaxGyroFactor, dMaxSetpointFactor);
                // dMaxBoost starts at zero, and by 1.0 we get Dmax, but it can exceed 1.
                dMaxMultiplier += (pidRuntime.dMaxPercent[axis] - 1.0f) * dMaxBoost;
                dMaxMultiplier = pt2FilterApply(&pidRuntime.dMaxLowpass[axis], dMaxMultiplier);
                // limit the gain to the fraction that DMax is greater than Min
                dMaxMultiplier = MIN(dMaxMultiplier, pidRuntime.dMaxPercent[axis]);
                if (debugMode == DEBUG_D_MAX && axis == gyro.gyroDebugAxis) {
                    DEBUG_SET(DEBUG_D_MAX, 0, lrintf(dMaxGyroFactor * 100));
                    DEBUG_SET(DEBUG_D_MAX, 1, lrintf(dMaxSetpointFactor * 100));
                    DEBUG_SET(DEBUG_D_MAX, 2, lrintf(pidRuntime.pidCoefficient[axis].Kd * dMaxMultiplier * 10 / DTERM_SCALE)); // effective Kd after Dmax boost
                    DEBUG_SET(DEBUG_D_MAX, 3, lrintf(dMaxMultiplier * 100));
                }
            }

            // Apply the gain that increases D towards Dmax
            preTpaD *= dMaxMultiplier;
#endif

            pidData[axis].D = preTpaD * getTpaFactor(pidProfile, axis, TERM_D);

            // Log the value of D pre application of TPA
            if (axis != FD_YAW) {
                DEBUG_SET(DEBUG_D_LPF, axis - FD_ROLL + 2, lrintf(preTpaD * D_LPF_PRE_TPA_SCALE));
            }
        } else {
            pidData[axis].D = 0;
            if (axis != FD_YAW) {
                DEBUG_SET(DEBUG_D_LPF, axis - FD_ROLL + 2, 0);
            }
        }

        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component

#ifdef USE_ABSOLUTE_CONTROL
        // include abs control correction in feedforward
        pidSetpointDelta += setpointCorrection - pidRuntime.oldSetpointCorrection[axis];
        pidRuntime.oldSetpointCorrection[axis] = setpointCorrection;
#endif
        // no feedforward in launch control
        const float feedforwardGain = launchControlActive ? 0.0f : pidRuntime.pidCoefficient[axis].Kf;
        pidData[axis].F = feedforwardGain * pidSetpointDelta;

#ifdef USE_YAW_SPIN_RECOVERY
        if (yawSpinActive) {
            pidData[axis].I = 0;  // in yaw spin always disable I
            if (axis <= FD_PITCH)  {
                // zero PIDs on pitch and roll leaving yaw P to correct spin
                pidData[axis].P = 0;
                pidData[axis].D = 0;
                pidData[axis].F = 0;
                pidData[axis].S = 0;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
        // Disable P/I appropriately based on the launch control mode
        if (launchControlActive) {
            // if not using FULL mode then disable I accumulation on yaw as
            // yaw has a tendency to windup. Otherwise limit yaw iterm accumulation.
            const int launchControlYawItermLimit = (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_FULL) ? LAUNCH_CONTROL_YAW_ITERM_LIMIT : 0;
            pidData[FD_YAW].I = constrainf(pidData[FD_YAW].I, -launchControlYawItermLimit, launchControlYawItermLimit);

            // for pitch-only mode we disable everything except pitch P/I
            if (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY) {
                pidData[FD_ROLL].P = 0;
                pidData[FD_ROLL].I = 0;
                pidData[FD_YAW].P = 0;
                // don't let I go negative (pitch backwards) as front motors are limited in the mixer
                pidData[FD_PITCH].I = MAX(0.0f, pidData[FD_PITCH].I);
            }
        }
#endif

        // Add P boost from antiGravity when sticks are close to zero
        if (axis != FD_YAW) {
            float agSetpointAttenuator = fabsf(currentPidSetpoint) / 50.0f;
            agSetpointAttenuator = MAX(agSetpointAttenuator, 1.0f);
            // attenuate effect if turning more than 50 deg/s, half at 100 deg/s
            const float antiGravityPBoost = 1.0f + (pidRuntime.antiGravityThrottleD / agSetpointAttenuator) * pidRuntime.antiGravityPGain;
            pidData[axis].P *= antiGravityPBoost;
            if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_ANTI_GRAVITY, 3, lrintf(antiGravityPBoost * 1000));
            }
        }

        pidData[axis].S = getSterm(axis, pidProfile, currentPidSetpointBeforeWingAdjust);
        applySpa(axis, pidProfile);

        // calculating the PID sum
        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F + pidData[axis].S;
#ifdef USE_INTEGRATED_YAW_CONTROL
        if (axis == FD_YAW && pidRuntime.useIntegratedYaw) {
            pidData[axis].Sum += pidSum * pidRuntime.dT * 100.0f;
            pidData[axis].Sum -= pidData[axis].Sum * pidRuntime.integratedYawRelax / 100000.0f * pidRuntime.dT / 0.000125f;
        } else
#endif
        {
            pidData[axis].Sum = pidSum;
        }
    }

#ifdef USE_WING
    // When PASSTHRU_MODE is active - reset all PIDs to zero so the aircraft won't snap out of control 
    // because of accumulated PIDs once PASSTHRU_MODE gets disabled.
    bool isFixedWingAndPassthru = isFixedWing() && FLIGHT_MODE(PASSTHRU_MODE);
#endif // USE_WING
    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if (!pidRuntime.pidStabilisationEnabled
        || gyroOverflowDetected()
#ifdef USE_WING
        || isFixedWingAndPassthru
#endif
        ) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;
            pidData[axis].S = 0;

            pidData[axis].Sum = 0;
        }
    } else if (pidRuntime.zeroThrottleItermReset) {
        pidResetIterm();
    }
}

bool crashRecoveryModeActive(void)
{
    return pidRuntime.inCrashRecoveryMode;
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (pidRuntime.acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        pidRuntime.acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER

void pidSetAntiGravityState(bool newState)
{
    if (newState != pidRuntime.antiGravityEnabled) {
        // reset the accelerator on state changes
        pidRuntime.itermAccelerator = 0.0f;
    }
    pidRuntime.antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return pidRuntime.antiGravityEnabled;
}

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    if (pidRuntime.dynLpfFilter != DYN_LPF_NONE) {
        float cutoffFreq;
        if (pidRuntime.dynLpfCurveExpo > 0) {
            cutoffFreq = dynLpfCutoffFreq(throttle, pidRuntime.dynLpfMin, pidRuntime.dynLpfMax, pidRuntime.dynLpfCurveExpo);
        } else {
            cutoffFreq = fmaxf(dynThrottle(throttle) * pidRuntime.dynLpfMax, pidRuntime.dynLpfMin);
        }

        switch (pidRuntime.dynLpfFilter) {
        case DYN_LPF_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_BIQUAD:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&pidRuntime.dtermLowpass[axis].biquadFilter, cutoffFreq, targetPidLooptime);
            }
            break;
        case DYN_LPF_PT2:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_PT3:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        }
    }
}
#endif

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo)
{
    const float expof = expo / 10.0f;
    const float curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

void pidSetItermReset(bool enabled)
{
    pidRuntime.zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return pidRuntime.previousPidSetpoint[axis];
}

float pidGetDT(void)
{
    return pidRuntime.dT;
}

float pidGetPidFrequency(void)
{
    return pidRuntime.pidFrequency;
}

#ifdef USE_CHIRP
bool  pidChirpIsFinished(void)
{
    return pidRuntime.chirp.isFinished;
}
#endif
