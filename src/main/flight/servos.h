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

#include "config/parameter_group.h"

#if defined(USE_QUAD_MIXER_ONLY)
#define MAX_SUPPORTED_SERVOS 1
#else
#define MAX_SUPPORTED_SERVOS 8
#endif

// These must be consecutive, see 'reversedSources'
enum {
    INPUT_STABILIZED_ROLL       = 0,
    INPUT_STABILIZED_PITCH      = 1,
    INPUT_STABILIZED_YAW        = 2,
    INPUT_STABILIZED_THROTTLE   = 3,
    INPUT_RC_ROLL               = 4,
    INPUT_RC_PITCH              = 5,
    INPUT_RC_YAW                = 6,
    INPUT_RC_THROTTLE           = 7,
    INPUT_RC_AUX1               = 8,
    INPUT_RC_AUX2               = 9,
    INPUT_RC_AUX3               = 10,
    INPUT_RC_AUX4               = 11,
    INPUT_GIMBAL_PITCH          = 12,
    INPUT_GIMBAL_ROLL           = 13,
    INPUT_FEATURE_FLAPS         = 14,

    INPUT_SOURCE_COUNT
} inputSource_e;

// target servo channels
typedef enum {
    SERVO_GIMBAL_PITCH = 0,
    SERVO_GIMBAL_ROLL = 1,
    SERVO_ELEVATOR = 2,
    SERVO_FLAPPERON_1 = 3,
    SERVO_FLAPPERON_2 = 4,
    SERVO_RUDDER = 5,

    SERVO_BICOPTER_LEFT = 4,
    SERVO_BICOPTER_RIGHT = 5,

    SERVO_DUALCOPTER_LEFT = 4,
    SERVO_DUALCOPTER_RIGHT = 5,

    SERVO_SINGLECOPTER_1 = 3,
    SERVO_SINGLECOPTER_2 = 4,
    SERVO_SINGLECOPTER_3 = 5,
    SERVO_SINGLECOPTER_4 = 6,

} servoIndex_e; // FIXME rename to servoChannel_e

#define SERVO_PLANE_INDEX_MIN SERVO_ELEVATOR
#define SERVO_PLANE_INDEX_MAX SERVO_RUDDER

#define SERVO_DUALCOPTER_INDEX_MIN SERVO_DUALCOPTER_LEFT
#define SERVO_DUALCOPTER_INDEX_MAX SERVO_DUALCOPTER_RIGHT

#define SERVO_SINGLECOPTER_INDEX_MIN SERVO_SINGLECOPTER_1
#define SERVO_SINGLECOPTER_INDEX_MAX SERVO_SINGLECOPTER_4

#define SERVO_FLAPPERONS_MIN SERVO_FLAPPERON_1
#define SERVO_FLAPPERONS_MAX SERVO_FLAPPERON_2

#define FLAPERON_THROW_DEFAULT 200
#define FLAPERON_THROW_MIN 100
#define FLAPERON_THROW_MAX 400

typedef struct servoMixer_s {
    uint8_t targetChannel;                  // servo that receives the output of the rule
    uint8_t inputSource;                    // input channel for this rule
    int8_t rate;                            // range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
    uint8_t speed;                          // reduces the speed of the rule, 0=unlimited speed
} servoMixer_t;

#define MAX_SERVO_RULES (2 * MAX_SUPPORTED_SERVOS)
#define MAX_SERVO_SPEED UINT8_MAX
#define MAX_SERVO_BOXES 3

#define SERVO_MIXER_INPUT_WIDTH 1000

PG_DECLARE_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers);

typedef struct servoParam_s {
    uint32_t reversedSources;               // the direction of servo movement for each input source of the servo mixer, bit set=inverted
    int16_t min;                            // servo min
    int16_t max;                            // servo max
    int16_t middle;                         // servo middle
    int8_t rate;                            // range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
    int8_t forwardFromChannel;              // RX channel index, 0 based.  See CHANNEL_FORWARDING_DISABLED
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);

typedef struct servoConfig_s {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    uint16_t servoCenterPulse;              // This is the value for servos when they should be in the middle. e.g. 1500.
    uint16_t servoPwmRate;                  // The update rate of servo outputs (50-498Hz)
    int16_t servo_lowpass_freq;             // lowpass servo filter frequency selection; 1/1000ths of loop freq
    uint16_t flaperon_throw_offset;
    uint8_t __reserved;
    uint8_t tri_unarmed_servo;              // send tail servo correction pulses even when unarmed
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);

typedef struct servoMetadata_s {
    float scaleMax;
    float scaleMin;
} servoMetadata_t;

extern int16_t servo[MAX_SUPPORTED_SERVOS];

bool isServoOutputEnabled(void);
bool isMixerUsingServos(void);
void writeServos(void);
void servoMixerLoadMix(int index);
void loadCustomServoMixer(void);
int servoDirection(int servoIndex, int fromChannel);
void servoMixer(float dT);
void servosInit(void);
