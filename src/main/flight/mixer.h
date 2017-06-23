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
#define MAX_SUPPORTED_MOTORS 4
#elif defined(TARGET_MOTOR_COUNT)
#define MAX_SUPPORTED_MOTORS TARGET_MOTOR_COUNT
#else
#define MAX_SUPPORTED_MOTORS 12
#endif

#define YAW_JUMP_PREVENTION_LIMIT_LOW 80
#define YAW_JUMP_PREVENTION_LIMIT_HIGH 500


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
    MIXER_CUSTOM_TRI = 25
} mixerMode_e;

#define DEFAULT_MIXER MIXER_QUADX

typedef struct motorAxisCorrectionLimits_s {
    int16_t min;
    int16_t max;
} motorAxisCorrectionLimits_t;

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
    mixerMode_e mixerMode;
    const motorMixer_t *motor;
    uint8_t flyingPlatformType;     // MC, FW or HELI
    uint8_t motorCount;
    bool useServos;
    bool hasFlaps;
} mixer_t;

typedef struct mixerConfig_s {
    uint8_t mixerMode;
    int8_t yaw_motor_direction;
    uint16_t yaw_jump_prevention_limit;      // make limit configurable (original fixed value was 100)
} mixerConfig_t;

PG_DECLARE(mixerConfig_t, mixerConfig);

typedef struct flight3DConfig_s {
    uint16_t deadband3d_low;                // min 3d value
    uint16_t deadband3d_high;               // max 3d value
    uint16_t neutral3d;                     // center 3d value
} flight3DConfig_t;

PG_DECLARE(flight3DConfig_t, flight3DConfig);

typedef struct motorConfig_s {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t motorPwmRate;                  // The update rate of motor outputs (50-498Hz)
    uint8_t  motorPwmProtocol;
} motorConfig_t;

PG_DECLARE(motorConfig_t, motorConfig);

#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

extern int16_t motor[MAX_SUPPORTED_MOTORS];
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

uint8_t getMotorCount(void);
bool mixerIsOutputSaturated(void);

void writeAllMotors(int16_t mc);
void mixerLoadMix(int index, motorMixer_t *customMixers);
void mixerUsePWMIOConfiguration(void);
void mixerUpdateStateFlags(void);
void mixerResetDisarmedMotors(void);
void mixTable(void);
void writeMotors(void);
void processServoTilt(void);
void processServoAutotrim(void);
void stopMotors(void);
void stopPwmAllMotors(void);

int getFlyingPlatformType(void);

bool isMixerEnabled(mixerMode_e mixerMode);
