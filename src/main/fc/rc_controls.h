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

#pragma once

#include <stdbool.h>

#include "config/parameter_group.h"

typedef enum rc_alias {
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
    AUX8
} rc_alias_e;

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttleStatus_e;

#define AIRMODEDEADBAND 12

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} rollPitchStatus_e;

typedef enum {
    RC_SMOOTHING_OFF = 0,
    RC_SMOOTHING_DEFAULT,
    RC_SMOOTHING_AUTO,
    RC_SMOOTHING_MANUAL
} rcSmoothing_t;

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

// Roll/pitch rates are a proportion used for mixing, so it tops out at 1.0:
#define CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX  100

/* Meaningful yaw rates are effectively unbounded because they are treated as a rotation rate multiplier: */
#define CONTROL_RATE_CONFIG_YAW_RATE_MAX         255

#define CONTROL_RATE_CONFIG_TPA_MAX              100

extern float rcCommand[4];

typedef struct rcControlsConfig_s {
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yaw_deadband;                   // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t alt_hold_deadband;              // defines the neutral zone of throttle stick during altitude hold, default setting is +/-40
    uint8_t alt_hold_fast_change;           // when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement
    bool yaw_control_reversed;            // invert control direction of yaw
} rcControlsConfig_t;

PG_DECLARE(rcControlsConfig_t, rcControlsConfig);

typedef struct flight3DConfig_s {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
    uint16_t deadband3d_throttle;           // default throttle deadband from MIDRC
} flight3DConfig_t;

PG_DECLARE(flight3DConfig_t, flight3DConfig);

typedef struct armingConfig_s {
    uint8_t gyro_cal_on_first_arm;          // allow disarm/arm on throttle down + roll left/right
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
} armingConfig_t;

PG_DECLARE(armingConfig_t, armingConfig);

bool areUsingSticksToArm(void);

bool areSticksInApModePosition(uint16_t ap_mode);
throttleStatus_e calculateThrottleStatus(void);
void processRcStickPositions(throttleStatus_e throttleStatus);

bool isUsingSticksForArming(void);

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc);
struct pidProfile_s;
struct modeActivationCondition_s;
void useRcControlsConfig(struct pidProfile_s *pidProfileToUse);
