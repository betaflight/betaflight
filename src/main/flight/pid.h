/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "rx/rx.h"

#pragma once

#define GYRO_I_MAX 256                      // Gyro I limiter
#define YAW_P_LIMIT_MIN 100                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_MAX 500                 // Maximum value for yaw P limiter
#define YAW_JUMP_PREVENTION_LIMIT_LOW 80
#define YAW_JUMP_PREVENTION_LIMIT_HIGH 400

#define DYNAMIC_PTERM_STICK_THRESHOLD 400


// Scaling factors for Pids for better tunable range in configurator for betaflight pid controller. The scaling is based on legacy pid controller or previous float
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f

typedef enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PID_ITEM_COUNT
} pidIndex_e;

typedef enum {
    PID_CONTROLLER_LEGACY = 0,           // Legacy PID controller. Old INT / Rewrite with 2.9 status. Fastest performance....least math. Will stay same in the future
    PID_CONTROLLER_BETAFLIGHT,           // Betaflight PID controller. Old luxfloat -> float evolution. More math added and maintained in the future
    PID_COUNT
} pidControllerType_e;

typedef enum {
    DELTA_FROM_ERROR = 0,
    DELTA_FROM_MEASUREMENT
} pidDeltaType_e;

typedef enum {
    SUPEREXPO_YAW_OFF = 0,
    SUPEREXPO_YAW_ON,
    SUPEREXPO_YAW_ALWAYS
} pidSuperExpoYaw_e;

typedef enum {
    PID_STABILISATION_OFF = 0,
    PID_STABILISATION_ON
} pidStabilisationState_e;

typedef struct pidProfile_s {
    uint8_t pidController;                  // 1 = rewrite betaflight evolved from http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671, 2 = Betaflight PIDc (Evolved Luxfloat)

    uint8_t P8[PID_ITEM_COUNT];
    uint8_t I8[PID_ITEM_COUNT];
    uint8_t D8[PID_ITEM_COUNT];

    uint8_t dterm_filter_type;              // Filter selection for dterm
    uint16_t dterm_lpf_hz;                  // Delta Filter in hz
    uint16_t yaw_lpf_hz;                    // Additional yaw filter when yaw axis too noisy
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff
    uint8_t deltaMethod;                    // Alternative delta Calculation
    uint16_t rollPitchItermIgnoreRate;      // Experimental threshold for resetting iterm for pitch and roll on certain rates
    uint16_t yawItermIgnoreRate;            // Experimental threshold for resetting iterm for yaw on certain rates
    uint16_t yaw_p_limit;
    uint8_t dterm_average_count;            // Configurable delta count for dterm
    uint8_t vbatPidCompensation;            // Scale PIDsum to battery voltage
    uint8_t pidAtMinThrottle;               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.

    // Betaflight PID controller parameters
    uint8_t itermThrottleGain;              // Throttle coupling to iterm. Quick throttle changes will bump iterm
    uint8_t ptermSRateWeight;               // Setpoint super expo ratio for Pterm (lower means that pretty much only P has super expo rates)
    uint8_t dtermSetpointWeight;            // Setpoint weight for Dterm (0= measurement, 1= full error, 1 > agressive derivative)
    uint16_t yawRateAccelLimit;             // yaw accel limiter for deg/sec/ms
    uint16_t rateAccelLimit;                // accel limiter roll/pitch deg/sec/ms

#ifdef GTUNE
    uint8_t  gtune_lolimP[3];               // [0..200] Lower limit of P during G tune
    uint8_t  gtune_hilimP[3];               // [0..200] Higher limit of P during G tune. 0 Disables tuning for that axis.
    uint8_t  gtune_pwr;                     // [0..10] Strength of adjustment
    uint16_t gtune_settle_time;             // [200..1000] Settle time in ms
    uint8_t  gtune_average_cycles;          // [8..128] Number of looptime cycles used for gyro average calculation
#endif
} pidProfile_t;

struct controlRateConfig_s;
union rollAndPitchTrims_u;
struct rxConfig_s;
typedef void (*pidControllerFuncPtr)(const pidProfile_t *pidProfile, uint16_t max_angle_inclination,
        const union rollAndPitchTrims_u *angleTrim, const struct rxConfig_s *rxConfig);            // pid controller function prototype

extern int16_t axisPID[XYZ_AXIS_COUNT];
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
bool airmodeWasActivated;
extern uint32_t targetPidLooptime;

void pidSetController(pidControllerType_e type);
void pidResetErrorGyroState(void);
void pidStabilisationState(pidStabilisationState_e pidControllerState);
void setTargetPidLooptime(uint32_t pidLooptime);

