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
#include "pg/pg.h"

#define MAX_PID_PROCESS_DENOM       16
#define PID_CONTROLLER_BETAFLIGHT   1
#define PID_MIXER_SCALING           1000.0f
#define PID_SERVO_MIXER_SCALING     0.7f
#define PIDSUM_LIMIT                500
#define PIDSUM_LIMIT_YAW            400
#define PIDSUM_LIMIT_MIN            100
#define PIDSUM_LIMIT_MAX            1000

// Scaling factors for Pids for better tunable range in configurator for betaflight pid controller. The scaling is based on legacy pid controller or previous float
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f

typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_ALT,
    PID_POS,
    PID_POSR,
    PID_NAVR,
    PID_LEVEL,
    PID_MAG,
    PID_VEL,
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
    PID_CRASH_RECOVERY_BEEP
} pidCrashRecovery_e;

typedef struct pid8_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
} pid8_t;

typedef enum {
    ITERM_RELAX_OFF,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY
} itermRelax_e;

typedef enum {
    ITERM_RELAX_GYRO,
    ITERM_RELAX_SETPOINT
} itermRelaxType_e;

typedef struct pidProfile_s {
    uint16_t yaw_lowpass_hz;                // Additional yaw filter when yaw axis too noisy
    uint16_t dterm_lowpass_hz;              // Delta Filter in hz
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff

    pid8_t  pid[PID_ITEM_COUNT];

    uint8_t dterm_filter_type;              // Filter selection for dterm
    uint8_t itermWindupPointPercent;        // Experimental ITerm windup threshold, percent motor saturation
    uint16_t pidSumLimit;
    uint16_t pidSumLimitYaw;
    uint8_t pidAtMinThrottle;               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
    uint8_t levelAngleLimit;                // Max angle in degrees in level mode

    uint8_t horizon_tilt_effect;            // inclination factor for Horizon mode
    uint8_t horizon_tilt_expert_mode;       // OFF or ON

    // Betaflight PID controller parameters
    uint16_t itermThrottleThreshold;        // max allowed throttle delta before iterm accelerated in ms
    uint16_t itermAcceleratorGain;          // Iterm Accelerator Gain when itermThrottlethreshold is hit
    uint16_t dtermSetpointWeight;           // Setpoint weight for Dterm (0= measurement, 1= full error, 1 > aggressive derivative)
    uint16_t yawRateAccelLimit;             // yaw accel limiter for deg/sec/ms
    uint16_t rateAccelLimit;                // accel limiter roll/pitch deg/sec/ms
    uint16_t crash_dthreshold;              // dterm crash value
    uint16_t crash_gthreshold;              // gyro crash value
    uint16_t crash_setpoint_threshold;      // setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
    uint16_t crash_time;                    // ms
    uint16_t crash_delay;                   // ms
    uint8_t crash_recovery_angle;           // degrees
    uint8_t crash_recovery_rate;            // degree/second
    uint8_t vbatPidCompensation;            // Scale PIDsum to battery voltage
    uint8_t setpointRelaxRatio;             // Setpoint weight relaxation effect
    uint16_t crash_limit_yaw;               // limits yaw errorRate, so crashes don't cause huge throttle increase
    uint16_t itermLimit;
    uint16_t dterm_lowpass2_hz;             // Extra PT1 Filter on D in hz
    uint8_t crash_recovery;                 // off, on, on and beeps when it is in crash recovery mode
    uint8_t throttle_boost;                 // how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
    uint8_t throttle_boost_cutoff;          // Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
    uint8_t  iterm_rotation;                // rotates iterm to translate world errors to local coordinate system
    uint8_t  smart_feedforward;             // takes only the larger of P and the D weight feed forward term if they have the same sign.
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
} pidProfile_t;

#ifndef USE_OSD_SLAVE
PG_DECLARE_ARRAY(pidProfile_t, MAX_PROFILE_COUNT, pidProfiles);
#endif

typedef struct pidConfig_s {
    uint8_t pid_process_denom;              // Processing denominator for PID controller vs gyro sampling rate
    uint8_t runaway_takeoff_prevention;          // off, on - enables pidsum runaway disarm logic
    uint16_t runaway_takeoff_deactivate_delay;   // delay in ms for "in-flight" conditions before deactivation (successful flight)
    uint8_t runaway_takeoff_deactivate_throttle; // minimum throttle percent required during deactivation phase
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);

union rollAndPitchTrims_u;
void pidController(const pidProfile_t *pidProfile, const union rollAndPitchTrims_u *angleTrim, timeUs_t currentTimeUs);

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;

    float Sum;
} pidAxisData_t;

extern pidAxisData_t pidData[3];

extern uint32_t targetPidLooptime;

extern float throttleBoost;
extern pt1Filter_t throttleLpf;

void pidResetITerm(void);
void pidStabilisationState(pidStabilisationState_e pidControllerState);
void pidSetItermAccelerator(float newItermAccelerator);
float pidItermAccelerator(void);
void pidInitFilters(const pidProfile_t *pidProfile);
void pidInitConfig(const pidProfile_t *pidProfile);
void pidInit(const pidProfile_t *pidProfile);
void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex);
bool crashRecoveryModeActive(void);
void pidAcroTrainerInit(void);
void pidSetAcroTrainerState(bool newState);
void pidInitSetpointDerivativeLpf(uint16_t filterCutoff, uint8_t debugAxis, uint8_t filterType);
void pidUpdateSetpointDerivativeLpf(uint16_t filterCutoff);
