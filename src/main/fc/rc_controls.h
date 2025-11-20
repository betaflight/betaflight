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

#include "common/axis.h"
#include "common/filter.h"
#include "pg/pg.h"

typedef enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
} rc_alias_e;

#define PRIMARY_CHANNEL_COUNT (THROTTLE + 1)

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

#define AIRMODEDEADBAND 12

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))
#define THR_MASK (3 << (2 * THROTTLE))

#define CONTROL_RATE_CONFIG_RC_EXPO_MAX  100

#define CONTROL_RATE_CONFIG_RC_RATES_MAX  255

#define CONTROL_RATE_CONFIG_RATE_LIMIT_MIN  200
#define CONTROL_RATE_CONFIG_RATE_LIMIT_MAX  1998

// (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
#define CONTROL_RATE_CONFIG_RATE_MAX  255

extern float rcCommand[4];

typedef struct rcSmoothingFilter_s {
    pt3Filter_t filterSetpoint[PRIMARY_CHANNEL_COUNT];
    pt3Filter_t filterRcDeflection[RP_AXIS_COUNT];
    pt3Filter_t filterFeedforward[XYZ_AXIS_COUNT];

    uint8_t setpointCutoffSetting;
    uint8_t throttleCutoffSetting;

    uint16_t setpointCutoffFrequency;
    uint16_t throttleCutoffFrequency;

    uint8_t debugAxis;

    float autoSmoothnessFactorSetpoint;
    float autoSmoothnessFactorThrottle;
} rcSmoothingFilter_t;

typedef struct rcControlsConfig_s {
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    bool yaw_control_reversed;              // invert control direction of yaw
} rcControlsConfig_t;

PG_DECLARE(rcControlsConfig_t, rcControlsConfig);

typedef struct flight3DConfig_s {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
    uint16_t deadband3d_throttle;           // default throttle deadband from MIDRC
    uint16_t limit3d_low;                   // pwm output value for max negative thrust
    uint16_t limit3d_high;                  // pwm output value for max positive thrust
    uint8_t switched_mode3d;                // enable '3D Switched Mode'
} flight3DConfig_t;

PG_DECLARE(flight3DConfig_t, flight3DConfig);

typedef struct armingConfig_s {
    uint8_t gyro_cal_on_first_arm;          // calibrate the gyro right before the first arm
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
    uint8_t prearm_allow_rearm;
} armingConfig_t;

PG_DECLARE(armingConfig_t, armingConfig);

bool areUsingSticksToArm(void);

throttleStatus_e calculateThrottleStatus(void);
void processRcStickPositions(void);

bool isUsingSticksForArming(void);

void rcControlsInit(void);

bool wasUserDisarmRequested(void); // Check if the user has requested a disarm since the last cleared

void clearUserDisarmRequested(void); // Clear the user disarm request flag
