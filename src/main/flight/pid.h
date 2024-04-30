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

#pragma once

#include <stdbool.h>
#include "common/time.h"
#include "common/filter.h"
#include "common/axis.h"

#include "pg/pg.h"

#define MAX_PID_PROCESS_DENOM       16
#define PID_CONTROLLER_BETAFLIGHT   1
#define PID_MIXER_SCALING           1000.0f
#define PID_SERVO_MIXER_SCALING     0.7f
#define PIDSUM_LIMIT                500
#define PIDSUM_LIMIT_YAW            400
#define PIDSUM_LIMIT_MIN            100
#define PIDSUM_LIMIT_MAX            1000

#define PID_GAIN_MAX 250
#define F_GAIN_MAX 1000
#define D_MIN_GAIN_MAX 250

// Scaling factors for Pids for better tunable range in configurator for betaflight pid controller. The scaling is based on legacy pid controller or previous float
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f

// The constant scale factor to replace the Kd component of the feedforward calculation.
// This value gives the same "feel" as the previous Kd default of 26 (26 * DTERM_SCALE)
#define FEEDFORWARD_SCALE 0.013754f

// Full iterm suppression in setpoint mode at high-passed setpoint rate > 40deg/sec
#define ITERM_RELAX_SETPOINT_THRESHOLD 40.0f
#define ITERM_RELAX_CUTOFF_DEFAULT 15

// Anti gravity I constant
#define ANTIGRAVITY_KI 0.34f; // if AG gain is 6, about 6 times iTerm will be added
#define ANTIGRAVITY_KP 0.0034f; // one fifth of the I gain on P by default
#define ITERM_ACCELERATOR_GAIN_OFF 0
#define ITERM_ACCELERATOR_GAIN_MAX 250

#define PID_ROLL_DEFAULT  { 45, 80, 40, 120 }
#define PID_PITCH_DEFAULT { 47, 84, 46, 125 }
#define PID_YAW_DEFAULT   { 45, 80,  0, 120 }
#define D_MIN_DEFAULT     { 30, 34, 0 }

#define DTERM_LPF1_DYN_MIN_HZ_DEFAULT 75
#define DTERM_LPF1_DYN_MAX_HZ_DEFAULT 150
#define DTERM_LPF2_HZ_DEFAULT 150

#define TPA_MAX 100

typedef enum {
    TPA_MODE_PD,
    TPA_MODE_D
} tpaMode_e;

typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_LEVEL,
    PID_MAG,
    PID_ITEM_COUNT
} pidIndex_e;

typedef enum {
    SUPEREXPO_YAW_OFF = 0,
    SUPEREXPO_YAW_ON,
    SUPEREXPO_YAW_ALWAYS
} pidSuperExpoYaw_e;

typedef enum {
    PID_STABILISATION_OFF = 0,
    PID_STABILISATION_ON
} pidStabilisationState_e;

typedef enum {
    PID_CRASH_RECOVERY_OFF = 0,
    PID_CRASH_RECOVERY_ON,
    PID_CRASH_RECOVERY_BEEP,
    PID_CRASH_RECOVERY_DISARM,
} pidCrashRecovery_e;

typedef struct pidf_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
    uint16_t F;
} pidf_t;

typedef enum {
    ITERM_RELAX_OFF,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY,
    ITERM_RELAX_RP_INC,
    ITERM_RELAX_RPY_INC,
    ITERM_RELAX_COUNT,
} itermRelax_e;

typedef enum {
    ITERM_RELAX_GYRO,
    ITERM_RELAX_SETPOINT,
    ITERM_RELAX_TYPE_COUNT,
} itermRelaxType_e;

typedef enum feedforwardAveraging_e {
    FEEDFORWARD_AVERAGING_OFF,
    FEEDFORWARD_AVERAGING_2_POINT,
    FEEDFORWARD_AVERAGING_3_POINT,
    FEEDFORWARD_AVERAGING_4_POINT,
} feedforwardAveraging_t;

#define MAX_PROFILE_NAME_LENGTH 8u

typedef struct pidProfile_s {
    uint16_t yaw_lowpass_hz;                // Additional yaw filter when yaw axis too noisy
    uint16_t dterm_lpf1_static_hz;          // Static Dterm lowpass 1 filter cutoff value in hz
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff

    pidf_t  pid[PID_ITEM_COUNT];

    uint8_t dterm_lpf1_type;                // Filter type for dterm lowpass 1
    uint8_t itermWindupPointPercent;        // iterm windup threshold, percent motor saturation
    uint16_t pidSumLimit;
    uint16_t pidSumLimitYaw;
    uint8_t pidAtMinThrottle;               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
    uint8_t angle_limit;                    // Max angle in degrees in Angle mode

    uint8_t horizon_limit_degrees;          // in Horizon mode, zero levelling when the quad's attitude exceeds this angle
    uint8_t horizon_ignore_sticks;          // 0 = default, meaning both stick and attitude attenuation; 1 = only attitude attenuation

    // Betaflight PID controller parameters
    uint8_t anti_gravity_gain;              // AntiGravity Gain (was itermAcceleratorGain)
    uint16_t yawRateAccelLimit;             // yaw accel limiter for deg/sec/ms
    uint16_t rateAccelLimit;                // accel limiter roll/pitch deg/sec/ms
    uint16_t crash_dthreshold;              // dterm crash value
    uint16_t crash_gthreshold;              // gyro crash value
    uint16_t crash_setpoint_threshold;      // setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
    uint16_t crash_time;                    // ms
    uint16_t crash_delay;                   // ms
    uint8_t crash_recovery_angle;           // degrees
    uint8_t crash_recovery_rate;            // degree/second
    uint16_t crash_limit_yaw;               // limits yaw errorRate, so crashes don't cause huge throttle increase
    uint16_t itermLimit;
    uint16_t dterm_lpf2_static_hz;          // Static Dterm lowpass 2 filter cutoff value in hz
    uint8_t crash_recovery;                 // off, on, on and beeps when it is in crash recovery mode
    uint8_t throttle_boost;                 // how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
    uint8_t throttle_boost_cutoff;          // Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
    uint8_t iterm_rotation;                 // rotates iterm to translate world errors to local coordinate system
    uint8_t iterm_relax_type;               // Specifies type of relax algorithm
    uint8_t iterm_relax_cutoff;             // This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
    uint8_t iterm_relax;                    // Enable iterm suppression during stick input
    uint8_t acro_trainer_angle_limit;       // Acro trainer roll/pitch angle limit in degrees
    uint8_t acro_trainer_debug_axis;        // The axis for which record debugging values are captured 0=roll, 1=pitch
    uint8_t acro_trainer_gain;              // The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
    uint16_t acro_trainer_lookahead_ms;     // The lookahead window in milliseconds used to reduce overshoot
    uint8_t abs_control_gain;               // How strongly should the absolute accumulated error be corrected for
    uint8_t abs_control_limit;              // Limit to the correction
    uint8_t abs_control_error_limit;        // Limit to the accumulated error
    uint8_t abs_control_cutoff;             // Cutoff frequency for path estimation in abs control
    uint8_t dterm_lpf2_type;                // Filter type for 2nd dterm lowpass
    uint16_t dterm_lpf1_dyn_min_hz;         // Dterm lowpass filter 1 min hz when in dynamic mode
    uint16_t dterm_lpf1_dyn_max_hz;         // Dterm lowpass filter 1 max hz when in dynamic mode
    uint8_t launchControlMode;              // Whether launch control is limited to pitch only (launch stand or top-mount) or all axes (on battery)
    uint8_t launchControlThrottlePercent;   // Throttle percentage to trigger launch for launch control
    uint8_t launchControlAngleLimit;        // Optional launch control angle limit (requires ACC)
    uint8_t launchControlGain;              // Iterm gain used while launch control is active
    uint8_t launchControlAllowTriggerReset; // Controls trigger behavior and whether the trigger can be reset
    uint8_t use_integrated_yaw;             // Selects whether the yaw pidsum should integrated
    uint8_t integrated_yaw_relax;           // Specifies how much integrated yaw should be reduced to offset the drag based yaw component
    uint8_t thrustLinearization;            // Compensation factor for pid linearization
    uint8_t d_min[XYZ_AXIS_COUNT];          // Minimum D value on each axis
    uint8_t d_min_gain;                     // Gain factor for amount of gyro / setpoint activity required to boost D
    uint8_t d_min_advance;                  // Percentage multiplier for setpoint input to boost algorithm
    uint8_t motor_output_limit;             // Upper limit of the motor output (percent)
    int8_t auto_profile_cell_count;         // Cell count for this profile to be used with if auto PID profile switching is used
    uint8_t transient_throttle_limit;       // Maximum DC component of throttle change to mix into throttle to prevent airmode mirroring noise
    char profileName[MAX_PROFILE_NAME_LENGTH + 1]; // Descriptive name for profile

    uint8_t dyn_idle_min_rpm;               // minimum motor speed enforced by the dynamic idle controller
    uint8_t dyn_idle_p_gain;                // P gain during active control of rpm
    uint8_t dyn_idle_i_gain;                // I gain during active control of rpm
    uint8_t dyn_idle_d_gain;                // D gain for corrections around rapid changes in rpm
    uint8_t dyn_idle_max_increase;          // limit on maximum possible increase in motor idle drive during active control
    uint8_t dyn_idle_start_increase;        // limit on maximum possible increase in motor idle drive with airmode not activated

    uint8_t feedforward_transition;         // Feedforward attenuation around centre sticks
    uint8_t feedforward_averaging;          // Number of packets to average when averaging is on
    uint8_t feedforward_smooth_factor;      // Amount of lowpass type smoothing for feedforward steps
    uint8_t feedforward_jitter_factor;      // Number of RC steps below which to attenuate feedforward
    uint8_t feedforward_boost;              // amount of setpoint acceleration to add to feedforward, 10 means 100% added
    uint8_t feedforward_max_rate_limit;     // Maximum setpoint rate percentage for feedforward

    uint8_t dterm_lpf1_dyn_expo;            // set the curve for dynamic dterm lowpass filter
    uint8_t level_race_mode;                // NFE race mode - when true pitch setpoint calculation is gyro based in level mode
    uint8_t vbat_sag_compensation;          // Reduce motor output by this percentage of the maximum compensation amount

    uint8_t simplified_pids_mode;
    uint8_t simplified_master_multiplier;
    uint8_t simplified_roll_pitch_ratio;
    uint8_t simplified_i_gain;
    uint8_t simplified_d_gain;
    uint8_t simplified_pi_gain;
    uint8_t simplified_dmin_ratio;
    uint8_t simplified_feedforward_gain;
    uint8_t simplified_dterm_filter;
    uint8_t simplified_dterm_filter_multiplier;
    uint8_t simplified_pitch_pi_gain;

    uint8_t anti_gravity_cutoff_hz;
    uint8_t anti_gravity_p_gain;
    uint8_t tpa_mode;                       // Controls which PID terms TPA effects
    uint8_t tpa_rate;                       // Percent reduction in P or D at full throttle
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated

    uint8_t angle_feedforward_smoothing_ms; // Smoothing factor for angle feedforward as time constant in milliseconds
    uint8_t angle_earth_ref;                // Control amount of "co-ordination" from yaw into roll while pitched forward in angle mode
    uint16_t horizon_delay_ms;              // delay when Horizon Strength increases, 50 = 500ms time constant
    uint8_t tpa_low_rate;                   // Percent reduction in P or D at zero throttle
    uint16_t tpa_low_breakpoint;            // Breakpoint where lower TPA is deactivated
    uint8_t tpa_low_always;                 // off, on - if OFF then low TPA is only active until tpa_low_breakpoint is reached the first time

    uint8_t ez_landing_threshold;           // Threshold stick position below which motor output is limited
    uint8_t ez_landing_limit;               // Maximum motor output when all sticks centred and throttle zero
    uint8_t ez_landing_speed;               // Speed below which motor output is limited
} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

typedef struct pidConfig_s {
    uint8_t pid_process_denom;                   // Processing denominator for PID controller vs gyro sampling rate
    uint8_t runaway_takeoff_prevention;          // off, on - enables pidsum runaway disarm logic
    uint16_t runaway_takeoff_deactivate_delay;   // delay in ms for "in-flight" conditions before deactivation (successful flight)
    uint8_t runaway_takeoff_deactivate_throttle; // minimum throttle percent required during deactivation phase
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);

union rollAndPitchTrims_u;
void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;

    float Sum;
} pidAxisData_t;

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
    pt2Filter_t pt2Filter;
    pt3Filter_t pt3Filter;
} dtermLowpass_t;

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

typedef struct pidRuntime_s {
    float dT;
    float pidFrequency;
    bool pidStabilisationEnabled;
    float previousPidSetpoint[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermNotchApplyFn;
    biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpassApplyFn;
    dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpass2ApplyFn;
    dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];
    filterApplyFnPtr ptermYawLowpassApplyFn;
    pt1Filter_t ptermYawLowpass;
    bool antiGravityEnabled;
    pt2Filter_t antiGravityLpf;
    float antiGravityOsdCutoff;
    float antiGravityThrottleD;
    float itermAccelerator;
    uint8_t antiGravityGain;
    float antiGravityPGain;
    pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
    float angleGain;
    float angleFeedforwardGain;
    float horizonGain;
    float horizonLimitSticks;
    float horizonLimitSticksInv;
    float horizonLimitDegrees;
    float horizonLimitDegreesInv;
    float horizonIgnoreSticks;
    float maxVelocity[XYZ_AXIS_COUNT];
    float itermWindupPointInv;
    bool inCrashRecoveryMode;
    timeUs_t crashDetectedAtUs;
    timeDelta_t crashTimeLimitUs;
    timeDelta_t crashTimeDelayUs;
    int32_t crashRecoveryAngleDeciDegrees;
    float crashRecoveryRate;
    float crashGyroThreshold;
    float crashDtermThreshold;
    float crashSetpointThreshold;
    float crashLimitYaw;
    float itermLimit;
    bool itermRotation;
    bool zeroThrottleItermReset;
    bool levelRaceMode;
    float tpaFactor;
    float tpaBreakpoint;
    float tpaMultiplier;
    float tpaLowBreakpoint;
    float tpaLowMultiplier;
    bool tpaLowAlways;

#ifdef USE_ITERM_RELAX
    pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
    uint8_t itermRelax;
    uint8_t itermRelaxType;
    uint8_t itermRelaxCutoff;
#endif

#ifdef USE_ABSOLUTE_CONTROL
    float acCutoff;
    float acGain;
    float acLimit;
    float acErrorLimit;
    pt1Filter_t acLpf[XYZ_AXIS_COUNT];
    float oldSetpointCorrection[XYZ_AXIS_COUNT];
#endif

#ifdef USE_D_MIN
    pt2Filter_t dMinRange[XYZ_AXIS_COUNT];
    pt2Filter_t dMinLowpass[XYZ_AXIS_COUNT];
    float dMinPercent[XYZ_AXIS_COUNT];
    float dMinGyroGain;
    float dMinSetpointGain;
#endif

#ifdef USE_AIRMODE_LPF
    pt1Filter_t airmodeThrottleLpf1;
    pt1Filter_t airmodeThrottleLpf2;
#endif

#ifdef USE_ACRO_TRAINER
    float acroTrainerAngleLimit;
    float acroTrainerLookaheadTime;
    uint8_t acroTrainerDebugAxis;
    float acroTrainerGain;
    bool acroTrainerActive;
    int acroTrainerAxisState[2];  // only need roll and pitch
#endif

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
    uint8_t dynLpfCurveExpo;
#endif

#ifdef USE_LAUNCH_CONTROL
    uint8_t launchControlMode;
    uint8_t launchControlAngleLimit;
    float launchControlKi;
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    bool useIntegratedYaw;
    uint8_t integratedYawRelax;
#endif

#ifdef USE_THRUST_LINEARIZATION
    float thrustLinearization;
    float throttleCompensateAmount;
#endif

#ifdef USE_AIRMODE_LPF
    float airmodeThrottleOffsetLimit;
#endif

#ifdef USE_FEEDFORWARD
    feedforwardAveraging_t feedforwardAveraging;
    float feedforwardSmoothFactor;
    uint8_t feedforwardJitterFactor;
    float feedforwardJitterFactorInv;
    float feedforwardBoostFactor;
    float feedforwardTransition;
    float feedforwardTransitionInv;
    uint8_t feedforwardMaxRateLimit;
    pt3Filter_t angleFeedforwardPt3[XYZ_AXIS_COUNT];
#endif

#ifdef USE_ACC
    pt3Filter_t attitudeFilter[2];  // Only for ROLL and PITCH
    pt1Filter_t horizonSmoothingPt1;
    uint16_t horizonDelayMs;
    float angleYawSetpoint;
    float angleEarthRef;
    float angleTarget[2];
    bool axisInAngleMode[3];
    float maxRcRateInv[2];
#endif
} pidRuntime_t;

extern pidRuntime_t pidRuntime;

extern const char pidNames[];

extern pidAxisData_t pidData[3];

extern uint32_t targetPidLooptime;

extern float throttleBoost;
extern pt1Filter_t throttleLpf;

void resetPidProfile(pidProfile_t *profile);

void pidResetIterm(void);
void pidStabilisationState(pidStabilisationState_e pidControllerState);
void pidSetItermAccelerator(float newItermAccelerator);
bool crashRecoveryModeActive(void);
void pidAcroTrainerInit(void);
void pidSetAcroTrainerState(bool newState);
void pidUpdateTpaFactor(float throttle);
void pidUpdateAntiGravityThrottleFilter(float throttle);
bool pidOsdAntiGravityActive(void);
void pidSetAntiGravityState(bool newState);
bool pidAntiGravityEnabled(void);

#ifdef USE_THRUST_LINEARIZATION
float pidApplyThrustLinearization(float motorValue);
float pidCompensateThrustLinearization(float throttle);
#endif

#ifdef USE_AIRMODE_LPF
void pidUpdateAirmodeLpf(float currentOffset);
float pidGetAirmodeThrottleOffset();
#endif

#ifdef UNIT_TEST
#include "sensors/acceleration.h"
extern float axisError[XYZ_AXIS_COUNT];
void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint);
void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate);
void rotateItermAndAxisError();
float pidLevel(int axis, const pidProfile_t *pidProfile,
    const rollAndPitchTrims_t *angleTrim, float rawSetpoint, float horizonLevelStrength);
float calcHorizonLevelStrength(void);
#endif

void dynLpfDTermUpdate(float throttle);
void pidSetItermReset(bool enabled);
float pidGetPreviousSetpoint(int axis);
float pidGetDT();
float pidGetPidFrequency();

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo);
