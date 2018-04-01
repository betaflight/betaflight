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

#include "platform.h"

#include "common/time.h"
#include "pg/pg.h"
#include "drivers/pwm_output_counts.h"
#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

#define QUAD_MOTOR_COUNT 4
#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 480

// Digital protocol has fixed values
#define DSHOT_DISARM_COMMAND      0
#define DSHOT_MIN_THROTTLE       48
#define DSHOT_MAX_THROTTLE     2047
#define DSHOT_3D_DEADBAND_LOW  1047
#define DSHOT_3D_DEADBAND_HIGH 1048

// Note: this is called MultiType/MULTITYPE_* in baseflight.
typedef enum mixerMode
{
    MIXER_TRI = 1,
    MIXER_QUADP = 2,
    MIXER_QUADX = 3,
    MIXER_BICOPTER = 4,
    MIXER_GIMBAL = 5,
    MIXER_Y6 = 6,
    MIXER_HEX6 = 7,
    MIXER_FLYING_WING = 8,
    MIXER_Y4 = 9,
    MIXER_HEX6X = 10,
    MIXER_OCTOX8 = 11,
    MIXER_OCTOFLATP = 12,
    MIXER_OCTOFLATX = 13,
    MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    MIXER_HELI_120_CCPM = 15,
    MIXER_HELI_90_DEG = 16,
    MIXER_VTAIL4 = 17,
    MIXER_HEX6H = 18,
    MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
    MIXER_DUALCOPTER = 20,
    MIXER_SINGLECOPTER = 21,
    MIXER_ATAIL4 = 22,
    MIXER_CUSTOM = 23,
    MIXER_CUSTOM_AIRPLANE = 24,
    MIXER_CUSTOM_TRI = 25,
    MIXER_QUADX_1234 = 26
} mixerMode_e;

// Custom mixer data per motor
typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

PG_DECLARE_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer);

// Custom mixer configuration
typedef struct mixer_s {
    uint8_t motorCount;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

typedef struct mixerConfig_s {
    uint8_t mixerMode;
    bool yaw_motors_reversed;
} mixerConfig_t;

PG_DECLARE(mixerConfig_t, mixerConfig);

typedef struct motorConfig_s {
    motorDevConfig_t dev;
    uint16_t digitalIdleOffsetValue;        // Idle value for DShot protocol, full motor output = 10000
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
} motorConfig_t;

PG_DECLARE(motorConfig_t, motorConfig);

#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

extern const mixer_t mixers[];
extern float motor[MAX_SUPPORTED_MOTORS];
extern float motor_disarmed[MAX_SUPPORTED_MOTORS];
extern float motorOutputHigh, motorOutputLow;
struct rxConfig_s;

uint8_t getMotorCount(void);
float getMotorMixRange(void);
bool areMotorsRunning(void);
bool mixerIsOutputSaturated(int axis, float errorRate);

void mixerLoadMix(int index, motorMixer_t *customMixers);
void mixerInit(mixerMode_e mixerMode);

void mixerConfigureOutput(void);

void mixerResetDisarmedMotors(void);
void mixTable(timeUs_t currentTimeUs, uint8_t vbatPidCompensation);
void syncMotors(bool enabled);
void writeMotors(void);
void stopMotors(void);
void stopPwmAllMotors(void);

float convertExternalToMotor(uint16_t externalValue);
uint16_t convertMotorToExternal(float motorValue);
bool mixerIsTricopter(void);
