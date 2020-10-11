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

#include "pg/pg.h"
#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

// These must be consecutive, see 'reversedSources'
enum {
    INPUT_STABILIZED_ROLL = 0,
    INPUT_STABILIZED_PITCH,
    INPUT_STABILIZED_YAW,
    INPUT_STABILIZED_THROTTLE,
    INPUT_RC_ROLL,
    INPUT_RC_PITCH,
    INPUT_RC_YAW,
    INPUT_RC_THROTTLE,
    INPUT_RC_AUX1,
    INPUT_RC_AUX2,
    INPUT_RC_AUX3,
    INPUT_RC_AUX4,
    INPUT_GIMBAL_PITCH,
    INPUT_GIMBAL_ROLL,
    INPUT_SOURCE_COUNT
};

// target servo channels
typedef enum {
    SERVO_GIMBAL_PITCH = 0,
    SERVO_GIMBAL_ROLL = 1,
    SERVO_FLAPS = 2,
    SERVO_FLAPPERON_1 = 3,
    SERVO_FLAPPERON_2 = 4,
    SERVO_RUDDER = 5,
    SERVO_ELEVATOR = 6,
    SERVO_THROTTLE = 7, // for internal combustion (IC) planes

    SERVO_BICOPTER_LEFT = 4,
    SERVO_BICOPTER_RIGHT = 5,

    SERVO_DUALCOPTER_LEFT = 4,
    SERVO_DUALCOPTER_RIGHT = 5,

    SERVO_SINGLECOPTER_1 = 3,
    SERVO_SINGLECOPTER_2 = 4,
    SERVO_SINGLECOPTER_3 = 5,
    SERVO_SINGLECOPTER_4 = 6,

    SERVO_HELI_LEFT = 0,
    SERVO_HELI_RIGHT = 1,
    SERVO_HELI_TOP = 2,
    SERVO_HELI_RUD = 3

} servoIndex_e; // FIXME rename to servoChannel_e

#define SERVO_PLANE_INDEX_MIN SERVO_FLAPS
#define SERVO_PLANE_INDEX_MAX SERVO_THROTTLE

#define SERVO_DUALCOPTER_INDEX_MIN SERVO_DUALCOPTER_LEFT
#define SERVO_DUALCOPTER_INDEX_MAX SERVO_DUALCOPTER_RIGHT

#define SERVO_SINGLECOPTER_INDEX_MIN SERVO_SINGLECOPTER_1
#define SERVO_SINGLECOPTER_INDEX_MAX SERVO_SINGLECOPTER_4

#define SERVO_FLAPPERONS_MIN SERVO_FLAPPERON_1
#define SERVO_FLAPPERONS_MAX SERVO_FLAPPERON_2

#define MAX_SERVO_RULES (2 * MAX_SUPPORTED_SERVOS)

typedef struct servoMixer_s {
    uint8_t targetChannel;                  // servo that receives the output of the rule
    uint8_t inputSource;                    // input channel for this rule
    int8_t rate;                            // range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
    uint8_t speed;                          // reduces the speed of the rule, 0=unlimited speed
    int8_t min;                             // lower bound of rule range [0;100]% of servo max-min
    int8_t max;                             // lower bound of rule range [0;100]% of servo max-min
    uint8_t box;                            // active rule if box is enabled, range [0;3], 0=no box, 1=BOXSERVO1, 2=BOXSERVO2, 3=BOXSERVO3
} servoMixer_t;

PG_DECLARE_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers);

#define MAX_SERVO_SPEED UINT8_MAX
#define MAX_SERVO_BOXES 3

// Custom mixer configuration
typedef struct mixerRules_s {
    uint8_t servoRuleCount;
    const servoMixer_t *rule;
} mixerRules_t;

extern const mixerRules_t servoMixers[];

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
    servoDevConfig_t dev;
    uint16_t servo_lowpass_freq;            // lowpass servo filter frequency selection; 1/1000ths of loop freq
    uint8_t tri_unarmed_servo;              // send tail servo correction pulses even when unarmed
    uint8_t channelForwardingStartChannel;
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);

typedef struct servoProfile_s {
    servoParam_t servoConf[MAX_SUPPORTED_SERVOS];
} servoProfile_t;

extern int16_t servo[MAX_SUPPORTED_SERVOS];

bool isMixerUsingServos(void);
void writeServos(void);
void servoMixerLoadMix(int index);
void loadCustomServoMixer(void);
int servoDirection(int servoIndex, int fromChannel);
void servosInit(void);
void servosFilterInit(void);
void servoMixer(void);
// tricopter specific
void servosTricopterInit(void);
void servosTricopterMixer(void);
bool servosTricopterIsEnabledServoUnarmed(void);
