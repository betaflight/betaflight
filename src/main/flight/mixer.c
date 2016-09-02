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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "build/debug.h"


#include "build/build_config.h"


#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"

#include "rx/rx.h"

#include "io/gimbal.h"
#include "io/escservo.h"
#include "fc/rc_controls.h"


#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"

#include "fc/runtime_config.h"

#include "config/config.h"
#include "config/config_profile.h"

//#define MIXER_DEBUG

uint8_t motorCount;

int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

bool motorLimitReached = false;

static mixerConfig_t *mixerConfig;
static flight3DConfig_t *flight3DConfig;
static escAndServoConfig_t *escAndServoConfig;
static rxConfig_t *rxConfig;

static mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];


#ifdef USE_SERVOS
static uint8_t servoRuleCount = 0;
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];
static gimbalConfig_t *gimbalConfig;
int16_t servo[MAX_SUPPORTED_SERVOS];
static int servoOutputEnabled;

static uint8_t mixerUsesServos;
static uint8_t minServoIndex;
static uint8_t maxServoIndex;

static servoParam_t *servoConf;
static biquadFilter_t servoFitlerState[MAX_SUPPORTED_SERVOS];
static bool servoFilterIsSet;
#endif

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};
#ifndef USE_QUAD_MIXER_ONLY
static const motorMixer_t mixerTricopter[] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

#ifndef DISABLE_UNCOMMON_MIXERS
static const motorMixer_t mixerVtail4[] = {
    { 1.0f,  -0.58f,  0.58f, 1.0f },        // REAR_R
    { 1.0f,  -0.46f, -0.39f, -0.5f },       // FRONT_R
    { 1.0f,  0.58f,  0.58f, -1.0f },        // REAR_L
    { 1.0f,  0.46f, -0.39f, 0.5f },         // FRONT_L
};

static const motorMixer_t mixerAtail4[] = {
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -0.0f },          // FRONT_L
};

static const motorMixer_t mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};

#if (MAX_SUPPORTED_MOTORS >= 6)
static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },     // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },     // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },     // LEFT
};

static const motorMixer_t mixerY6[] = {
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};

static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
    { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
    { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
};
#endif

#if (MAX_SUPPORTED_MOTORS >= 8)
static const motorMixer_t mixerOctoFlatP[] = {
    { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
    { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
    { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
    { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
    { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
};

static const motorMixer_t mixerOctoFlatX[] = {
    { 1.0f,  1.0f, -0.414178f,  1.0f },      // MIDFRONT_L
    { 1.0f, -0.414178f, -1.0f,  1.0f },      // FRONT_R
    { 1.0f, -1.0f,  0.414178f,  1.0f },      // MIDREAR_R
    { 1.0f,  0.414178f,  1.0f,  1.0f },      // REAR_L
    { 1.0f,  0.414178f, -1.0f, -1.0f },      // FRONT_L
    { 1.0f, -1.0f, -0.414178f, -1.0f },      // MIDFRONT_R
    { 1.0f, -0.414178f,  1.0f, -1.0f },      // REAR_R
    { 1.0f,  1.0f,  0.414178f, -1.0f },      // MIDREAR_L
};

static const motorMixer_t mixerOctoX8[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};
#endif
#endif //DISABLE_UNCOMMON_MIXERS

#if (MAX_SUPPORTED_MOTORS >= 6)
static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};
#endif

static const motorMixer_t mixerSingleProp[] = {
    { 1.0f,  0.0f,  0.0f, 0.0f },
};

// Keep synced with mixerMode_e
const mixer_t mixers[] = {
    // motors, use servo, motor mixer
    { 0, false, NULL, true },                // entry 0
    { 3, true,  mixerTricopter, true },      // MIXER_TRI
    { 4, false, mixerQuadP, true },          // MIXER_QUADP
    { 4, false, mixerQuadX, true },          // MIXER_QUADX

    { 0, false, NULL, false },               // MIXER_BICOPTER
    { 0, false, NULL, false },               // MIXER_GIMBAL -> this mixer was never implemented in CF, use feature(FEATURE_SERVO_TILT) instead
    #if !defined(DISABLE_UNCOMMON_MIXERS) && (MAX_SUPPORTED_MOTORS >= 6)
        { 6, false, mixerY6, true },         // MIXER_Y6
        { 6, false, mixerHex6P, true },      // MIXER_HEX6
    #else
        { 0, false, NULL, false },           // MIXER_Y6
        { 0, false, NULL, false },           // MIXER_HEX6
    #endif
    { 1, true,  mixerSingleProp, true },     // MIXER_FLYING_WING
    #if !defined(DISABLE_UNCOMMON_MIXERS)
        { 4, false, mixerY4, true },         // MIXER_Y4
    #else
        { 0, false, NULL, false },           // MIXER_Y4
    #endif
    #if (MAX_SUPPORTED_MOTORS >= 6)
        { 6, false, mixerHex6X, true },          // MIXER_HEX6X
    #else
        { 0, false, NULL, false },          // MIXER_HEX6X
    #endif
    #if !defined(DISABLE_UNCOMMON_MIXERS) && (MAX_SUPPORTED_MOTORS >= 8)
        { 8, false, mixerOctoX8, true },     // MIXER_OCTOX8
        { 8, false, mixerOctoFlatP, true },  // MIXER_OCTOFLATP
        { 8, false, mixerOctoFlatX, true },  // MIXER_OCTOFLATX
    #else
        { 0, false, NULL, false },           // MIXER_OCTOX8
        { 0, false, NULL, false },           // MIXER_OCTOFLATP
        { 0, false, NULL, false },           // MIXER_OCTOFLATX
    #endif
    { 1, true,  mixerSingleProp, true },     // * MIXER_AIRPLANE
    { 0, true,  NULL, false },               // * MIXER_HELI_120_CCPM -> disabled, never fully implemented in CF
    { 0, true,  NULL, false },               // * MIXER_HELI_90_DEG -> disabled, never fully implemented in CF
    #if !defined(DISABLE_UNCOMMON_MIXERS)
        { 4, false, mixerVtail4, true },     // MIXER_VTAIL4
    #if (MAX_SUPPORTED_MOTORS >= 6)
        { 6, false, mixerHex6H, true },      // MIXER_HEX6H
    #else
        { 0, false, NULL, false },           // MIXER_HEX6H
    #endif
    #else
        { 0, false, NULL, false },           // MIXER_VTAIL4
        { 0, false, NULL, false },           // MIXER_HEX6H
    #endif
    { 0, true,  NULL, false },               // * MIXER_PPM_TO_SERVO -> looks like this is not implemented at all
    { 0, false, NULL, false },               // MIXER_DUALCOPTER
    { 0, false, NULL, false },               // MIXER_SINGLECOPTER
    #if !defined(DISABLE_UNCOMMON_MIXERS)
        { 4, false, mixerAtail4, true },     // MIXER_ATAIL4
    #else
        { 0, false, NULL, false },           // MIXER_ATAIL4
    #endif
    { 0, false, NULL, true },                // MIXER_CUSTOM
    { 2, true,  NULL, true },                // MIXER_CUSTOM_AIRPLANE
    { 3, true,  NULL, true },                // MIXER_CUSTOM_TRI
};
#endif // USE_QUAD_MIXER_ONLY

#ifdef USE_SERVOS

#define COUNT_SERVO_RULES(rules) (sizeof(rules) / sizeof(servoMixer_t))
// mixer rule format servo, input, rate, speed, min, max, box
static const servoMixer_t servoMixerAirplane[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_ELEVATOR, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_THROTTLE, INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerFlyingWing[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  -100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_THROTTLE, INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerTri[] = {
    { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
};

const mixerRules_t servoMixers[] = {
    { 0, 0, 0, NULL },                // entry 0
    { COUNT_SERVO_RULES(servoMixerTri), SERVO_RUDDER, SERVO_RUDDER, servoMixerTri },       // MULTITYPE_TRI
    { 0, 0, 0, NULL },                // MULTITYPE_QUADP
    { 0, 0, 0, NULL },                // MULTITYPE_QUADX
    { 0, 0, 0, NULL },                // MULTITYPE_BI
    { 0, 0, 0, NULL },                // MULTITYPE_GIMBAL / MIXER_GIMBAL -> disabled
    { 0, 0, 0, NULL },                // MULTITYPE_Y6
    { 0, 0, 0, NULL },                // MULTITYPE_HEX6
    { COUNT_SERVO_RULES(servoMixerFlyingWing), SERVO_FLAPPERONS_MIN, SERVO_FLAPPERONS_MAX, servoMixerFlyingWing },// * MULTITYPE_FLYING_WING
    { 0, 0, 0, NULL },                // MULTITYPE_Y4
    { 0, 0, 0, NULL },                // MULTITYPE_HEX6X
    { 0, 0, 0, NULL },                // MULTITYPE_OCTOX8
    { 0, 0, 0, NULL },                // MULTITYPE_OCTOFLATP
    { 0, 0, 0, NULL },                // MULTITYPE_OCTOFLATX
    { COUNT_SERVO_RULES(servoMixerAirplane), SERVO_PLANE_INDEX_MIN, SERVO_PLANE_INDEX_MAX, servoMixerAirplane },  // * MULTITYPE_AIRPLANE
    { 0, 0, 0, NULL },                // * MIXER_HELI_120_CCPM -> disabled, never fully implemented in CF
    { 0, 0, 0, NULL },                // * MIXER_HELI_90_DEG -> disabled, never fully implemented in CF
    { 0, 0, 0, NULL },                // MULTITYPE_VTAIL4
    { 0, 0, 0, NULL },                // MULTITYPE_HEX6H
    { 0, 0, 0, NULL },                // * MULTITYPE_PPM_TO_SERVO
    { 0, 0, 0, NULL },                // MULTITYPE_DUALCOPTER
    { 0, 0, 0, NULL },                // MULTITYPE_SINGLECOPTER
    { 0, 0, 0, NULL },                // MULTITYPE_ATAIL4
    { 0, 2, 5, NULL },                // MULTITYPE_CUSTOM
    { 0, SERVO_PLANE_INDEX_MIN, SERVO_PLANE_INDEX_MAX, NULL },                // MULTITYPE_CUSTOM_PLANE
    { 0, SERVO_RUDDER, SERVO_RUDDER, NULL },                // MULTITYPE_CUSTOM_TRI
};

static servoMixer_t *customServoMixers;
#endif


static motorMixer_t *customMixers;

void mixerUseConfigs(
#ifdef USE_SERVOS
        servoParam_t *servoConfToUse,
        gimbalConfig_t *gimbalConfigToUse,
#endif
        flight3DConfig_t *flight3DConfigToUse,
        escAndServoConfig_t *escAndServoConfigToUse,
        mixerConfig_t *mixerConfigToUse,
        rxConfig_t *rxConfigToUse)
{
#ifdef USE_SERVOS
    servoConf = servoConfToUse;
    gimbalConfig = gimbalConfigToUse;
#endif
    flight3DConfig = flight3DConfigToUse;
    escAndServoConfig = escAndServoConfigToUse;
    mixerConfig = mixerConfigToUse;
    rxConfig = rxConfigToUse;
}

#ifdef USE_SERVOS

int16_t getFlaperonDirection(uint8_t servoPin) {
    if (servoPin == SERVO_FLAPPERON_2) {
        return -1;
    } else {
        return 1;
    }
}

int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    uint8_t channelToForwardFrom = servoConf[servoIndex].forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeConfig.channelCount) {
        return rcData[channelToForwardFrom];
    }

    return servoConf[servoIndex].middle;
}


int servoDirection(int servoIndex, int inputSource)
{
    // determine the direction (reversed or not) from the direction bitfield of the servo
    if (servoConf[servoIndex].reversedSources & (1 << inputSource))
        return -1;
    else
        return 1;
}
#endif

bool isMixerEnabled(mixerMode_e mixerMode)
{
#ifdef USE_QUAD_MIXER_ONLY
    UNUSED(mixerMode);
    return true;
#else
    return mixers[mixerMode].enabled;
#endif
}

#ifdef USE_SERVOS
void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMotorMixers, servoMixer_t *initialCustomServoMixers)
{
    int i;

    currentMixerMode = mixerMode;

    // set flag that we're on something with wings
    if (currentMixerMode == MIXER_FLYING_WING ||
        currentMixerMode == MIXER_AIRPLANE ||
        currentMixerMode == MIXER_CUSTOM_AIRPLANE
    ) {
        ENABLE_STATE(FIXED_WING);
    } else {
        DISABLE_STATE(FIXED_WING);
    }

    if (currentMixerMode == MIXER_AIRPLANE || currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
        ENABLE_STATE(FLAPERON_AVAILABLE);
    } else {
        DISABLE_STATE(FLAPERON_AVAILABLE);
    }

    customMixers = initialCustomMotorMixers;
    customServoMixers = initialCustomServoMixers;

    minServoIndex = servoMixers[currentMixerMode].minServoIndex;
    maxServoIndex = servoMixers[currentMixerMode].maxServoIndex;

    // enable servos for mixes that require them. note, this shifts motor counts.
    mixerUsesServos = mixers[currentMixerMode].useServo || feature(FEATURE_SERVO_TILT);

    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    servoOutputEnabled = mixerUsesServos || feature(FEATURE_CHANNEL_FORWARDING);

    // give all servos a default command
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }

    /*
     * If mixer has predefined servo mixes, load them
     */
    if (mixerUsesServos) {
        servoRuleCount = servoMixers[currentMixerMode].servoRuleCount;
        if (servoMixers[currentMixerMode].rule) {
            for (i = 0; i < servoRuleCount; i++)
                currentServoMixer[i] = servoMixers[currentMixerMode].rule[i];
        }
    }

    /*
     * When we are dealing with CUSTOM mixers, load mixes defined with
     * smix and update internal variables
     */
    if (currentMixerMode == MIXER_CUSTOM_AIRPLANE ||
        currentMixerMode == MIXER_CUSTOM_TRI ||
        currentMixerMode == MIXER_CUSTOM)
    {
        loadCustomServoMixer();

        // If there are servo rules after all, update variables
        if (servoRuleCount > 0) {
            servoOutputEnabled = 1;
            mixerUsesServos = 1;
        }

    }
}
#else
void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers)
{
    currentMixerMode = mixerMode;
    customMixers = initialCustomMixers;
}
#endif

#ifdef USE_SERVOS
void mixerUsePWMIOConfiguration(void)
{
    int i;

    motorCount = 0;

    if (currentMixerMode == MIXER_CUSTOM || currentMixerMode == MIXER_CUSTOM_TRI || currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
        // load custom mixer into currentMixer
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            // check if done
            if (customMixers[i].throttle == 0.0f)
                break;
            currentMixer[i] = customMixers[i];
            motorCount++;
        }
    } else {
        motorCount = mixers[currentMixerMode].motorCount;
        // copy motor-based mixers
        if (mixers[currentMixerMode].motor) {
            for (i = 0; i < motorCount; i++)
                currentMixer[i] = mixers[currentMixerMode].motor[i];
        }
    }

    // in 3D mode, mixer gain has to be halved
    if (feature(FEATURE_3D)) {
        if (motorCount > 1) {
            for (i = 0; i < motorCount; i++) {
                currentMixer[i].pitch *= 0.5f;
                currentMixer[i].roll *= 0.5f;
                currentMixer[i].yaw *= 0.5f;
            }
        }
    }

    mixerResetDisarmedMotors();
}
#else
void mixerUsePWMIOConfiguration(void)
{
    motorCount = 4;
    int i;
    for (i = 0; i < motorCount; i++) {
        currentMixer[i] = mixerQuadX[i];
    }
    mixerResetDisarmedMotors();
}
#endif


#ifndef USE_QUAD_MIXER_ONLY
#ifdef USE_SERVOS
void loadCustomServoMixer(void)
{
    int i;

    // reset settings
    servoRuleCount = 0;
    minServoIndex = 255;
    maxServoIndex = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));

    // load custom mixer into currentServoMixer
    for (i = 0; i < MAX_SERVO_RULES; i++) {
        // check if done
        if (customServoMixers[i].rate == 0)
            break;

        if (customServoMixers[i].targetChannel < minServoIndex) {
            minServoIndex = customServoMixers[i].targetChannel;
        }

        if (customServoMixers[i].targetChannel > maxServoIndex) {
            maxServoIndex = customServoMixers[i].targetChannel;
        }

        currentServoMixer[i] = customServoMixers[i];
        servoRuleCount++;
    }
}

void servoMixerLoadMix(int index, servoMixer_t *customServoMixers)
{
    int i;

    // we're 1-based
    index++;
    // clear existing
    for (i = 0; i < MAX_SERVO_RULES; i++)
        customServoMixers[i].targetChannel = customServoMixers[i].inputSource = customServoMixers[i].rate = customServoMixers[i].box = 0;

    for (i = 0; i < servoMixers[index].servoRuleCount; i++) {
        customServoMixers[i] = servoMixers[index].rule[i];
    }
}
#endif


void mixerLoadMix(int index, motorMixer_t *customMixers)
{
    int i;

    // we're 1-based
    index++;
    // clear existing
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        customMixers[i].throttle = 0.0f;

    // do we have anything here to begin with?
    if (mixers[index].motor != NULL) {
        for (i = 0; i < mixers[index].motorCount; i++)
            customMixers[i] = mixers[index].motor[i];
    }
}

#endif

void mixerResetDisarmedMotors(void)
{
    int i;
    // set disarmed motor values
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motor_disarmed[i] = feature(FEATURE_3D) ? flight3DConfig->neutral3d : escAndServoConfig->mincommand;
}

#ifdef USE_SERVOS

STATIC_UNIT_TESTED void forwardAuxChannelsToServos(uint8_t firstServoIndex)
{
    // start forwarding from this channel
    uint8_t channelOffset = AUX1;

    int servoOffset;
    for (servoOffset = 0; servoOffset < MAX_AUX_CHANNEL_COUNT && channelOffset < MAX_SUPPORTED_RC_CHANNEL_COUNT; servoOffset++) {
        pwmWriteServo(firstServoIndex + servoOffset, rcData[channelOffset++]);
    }
}

void writeServos(void)
{
    int servoIndex = 0;

    bool zeroServoValue = false;

    /*
     * in case of tricopters, there might me a need to zero servo output when unarmed
     */
    if ((currentMixerMode == MIXER_TRI || currentMixerMode == MIXER_CUSTOM_TRI) && !ARMING_FLAG(ARMED) && !mixerConfig->tri_unarmed_servo) {
        zeroServoValue = true;
    }

    // Write mixer servo outputs
    //      mixerUsesServos might indicate SERVO_TILT, servoRuleCount indicate if mixer alone uses servos
    if (mixerUsesServos && servoRuleCount) {
        for (int i = minServoIndex; i <= maxServoIndex; i++) {
            if (zeroServoValue) {
                pwmWriteServo(servoIndex++, 0);
            } else {
                pwmWriteServo(servoIndex++, servo[i]);
            }
        }
    }

    if (feature(FEATURE_SERVO_TILT)) {
        pwmWriteServo(servoIndex++, servo[SERVO_GIMBAL_PITCH]);
        pwmWriteServo(servoIndex++, servo[SERVO_GIMBAL_ROLL]);
    }

    // forward AUX to remaining servo outputs (not constrained)
    if (feature(FEATURE_CHANNEL_FORWARDING)) {
        forwardAuxChannelsToServos(servoIndex);
        servoIndex += MAX_AUX_CHANNEL_COUNT;
    }
}
#endif

void writeMotors(void)
{
    int i;

    for (i = 0; i < motorCount; i++)
        pwmWriteMotor(i, motor[i]);


    if (feature(FEATURE_ONESHOT125)) {
        pwmCompleteOneshotMotorUpdate(motorCount);
    }
}

void writeAllMotors(int16_t mc)
{
    int i;

    // Sends commands to all motors
    for (i = 0; i < motorCount; i++)
        motor[i] = mc;
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(feature(FEATURE_3D) ? flight3DConfig->neutral3d : escAndServoConfig->mincommand);

    delay(50); // give the timers and ESCs a chance to react.
}

void StopPwmAllMotors()
{
    pwmShutdownPulsesForAllMotors(motorCount);
}

void mixTable(void)
{
    int i;

    if (motorCount >= 4 && mixerConfig->yaw_jump_prevention_limit < YAW_JUMP_PREVENTION_LIMIT_HIGH) {
        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -mixerConfig->yaw_jump_prevention_limit - ABS(rcCommand[YAW]), mixerConfig->yaw_jump_prevention_limit + ABS(rcCommand[YAW]));
    }

    // Initial mixer concept by bdoiron74 reused and optimized for Air Mode
    int16_t rpyMix[MAX_SUPPORTED_MOTORS];
    int16_t rpyMixMax = 0; // assumption: symetrical about zero.
    int16_t rpyMixMin = 0;

    // motors for non-servo mixes
    for (i = 0; i < motorCount; i++) {
        rpyMix[i] =
            axisPID[PITCH] * currentMixer[i].pitch +
            axisPID[ROLL] * currentMixer[i].roll +
            -mixerConfig->yaw_motor_direction * axisPID[YAW] * currentMixer[i].yaw;

        if (rpyMix[i] > rpyMixMax) rpyMixMax = rpyMix[i];
        if (rpyMix[i] < rpyMixMin) rpyMixMin = rpyMix[i];
    }

    int16_t rpyMixRange = rpyMixMax - rpyMixMin;
    int16_t throttleRange, throttleCommand;
    int16_t throttleMin, throttleMax;
    static int16_t throttlePrevious = 0;   // Store the last throttle direction for deadband transitions

    // Find min and max throttle based on condition.
    if (feature(FEATURE_3D)) {
        if (!ARMING_FLAG(ARMED)) throttlePrevious = rxConfig->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.

        if ((rcCommand[THROTTLE] <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle))) { // Out of band handling
            throttleMax = flight3DConfig->deadband3d_low;
            throttleMin = escAndServoConfig->minthrottle;
            throttlePrevious = throttleCommand = rcCommand[THROTTLE];
        } else if (rcCommand[THROTTLE] >= (rxConfig->midrc + flight3DConfig->deadband3d_throttle)) { // Positive handling
            throttleMax = escAndServoConfig->maxthrottle;
            throttleMin = flight3DConfig->deadband3d_high;
            throttlePrevious = throttleCommand = rcCommand[THROTTLE];
        } else if ((throttlePrevious <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle)))  { // Deadband handling from negative to positive
            throttleCommand = throttleMax = flight3DConfig->deadband3d_low;
            throttleMin = escAndServoConfig->minthrottle;
        } else {  // Deadband handling from positive to negative
            throttleMax = escAndServoConfig->maxthrottle;
            throttleCommand = throttleMin = flight3DConfig->deadband3d_high;
        }
    } else {
        throttleCommand = rcCommand[THROTTLE];
        throttleMin = escAndServoConfig->minthrottle;
        throttleMax = escAndServoConfig->maxthrottle;
    }

    throttleRange = throttleMax - throttleMin;

    #define THROTTLE_CLIPPING_FACTOR    0.33f
    if (rpyMixRange > throttleRange) {
        motorLimitReached = true;
        float mixReduction = (float)throttleRange / rpyMixRange;

        for (i = 0; i < motorCount; i++) {
            rpyMix[i] =  mixReduction  * rpyMix[i];
        }

        // Allow some clipping on edges to soften correction response
        throttleMin = throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2);
        throttleMax = throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2);
    } else {
        motorLimitReached = false;
        throttleMin = MIN(throttleMin + (rpyMixRange / 2), throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
        throttleMax = MAX(throttleMax - (rpyMixRange / 2), throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
    }

    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    if (ARMING_FLAG(ARMED)) {
        bool isFailsafeActive = failsafeIsActive();

        for (i = 0; i < motorCount; i++) {
            motor[i] = rpyMix[i] + constrain(throttleCommand * currentMixer[i].throttle, throttleMin, throttleMax);

            if (isFailsafeActive) {
                motor[i] = constrain(motor[i], escAndServoConfig->mincommand, escAndServoConfig->maxthrottle);
            } else if (feature(FEATURE_3D)) {
                if (throttlePrevious <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle)) {
                    motor[i] = constrain(motor[i], escAndServoConfig->minthrottle, flight3DConfig->deadband3d_low);
                } else {
                    motor[i] = constrain(motor[i], flight3DConfig->deadband3d_high, escAndServoConfig->maxthrottle);
                }
            } else {
                motor[i] = constrain(motor[i], escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
            }

            // Motor stop handling
            if (feature(FEATURE_MOTOR_STOP) && ARMING_FLAG(ARMED) && !feature(FEATURE_3D)) {
                if (((rcData[THROTTLE]) < rxConfig->mincheck)) {
                    motor[i] = escAndServoConfig->mincommand;
                }
            }
        }
    } else {
        for (i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

#ifdef USE_SERVOS

void servoMixer(uint16_t flaperon_throw_offset, uint8_t flaperon_throw_inverted)
{
    int16_t input[INPUT_SOURCE_COUNT]; // Range [-500:+500]
    static int16_t currentOutput[MAX_SERVO_RULES];
    int i;

    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        // Direct passthru from RX
        input[INPUT_STABILIZED_ROLL] = rcCommand[ROLL];
        input[INPUT_STABILIZED_PITCH] = rcCommand[PITCH];
        input[INPUT_STABILIZED_YAW] = rcCommand[YAW];
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        input[INPUT_STABILIZED_ROLL] = axisPID[ROLL];
        input[INPUT_STABILIZED_PITCH] = axisPID[PITCH];
        input[INPUT_STABILIZED_YAW] = axisPID[YAW];

        // Reverse yaw servo when inverted in 3D mode
        if (feature(FEATURE_3D) && (rcData[THROTTLE] < rxConfig->midrc)) {
            input[INPUT_STABILIZED_YAW] *= -1;
        }
    }

    input[INPUT_GIMBAL_PITCH] = scaleRange(attitude.values.pitch, -1800, 1800, -500, +500);
    input[INPUT_GIMBAL_ROLL] = scaleRange(attitude.values.roll, -1800, 1800, -500, +500);

    input[INPUT_STABILIZED_THROTTLE] = motor[0] - 1000 - 500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]

    // center the RC input value around the RC middle value
    // by subtracting the RC middle value from the RC input value, we get:
    // data - middle = input
    // 2000 - 1500 = +500
    // 1500 - 1500 = 0
    // 1000 - 1500 = -500
    input[INPUT_RC_ROLL]     = rcData[ROLL]     - rxConfig->midrc;
    input[INPUT_RC_PITCH]    = rcData[PITCH]    - rxConfig->midrc;
    input[INPUT_RC_YAW]      = rcData[YAW]      - rxConfig->midrc;
    input[INPUT_RC_THROTTLE] = rcData[THROTTLE] - rxConfig->midrc;
    input[INPUT_RC_AUX1]     = rcData[AUX1]     - rxConfig->midrc;
    input[INPUT_RC_AUX2]     = rcData[AUX2]     - rxConfig->midrc;
    input[INPUT_RC_AUX3]     = rcData[AUX3]     - rxConfig->midrc;
    input[INPUT_RC_AUX4]     = rcData[AUX4]     - rxConfig->midrc;

    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++)
        servo[i] = 0;

    // mix servos according to rules
    for (i = 0; i < servoRuleCount; i++) {
        // consider rule if no box assigned or box is active
        if (currentServoMixer[i].box == 0 || IS_RC_MODE_ACTIVE(BOXSERVO1 + currentServoMixer[i].box - 1)) {
            uint8_t target = currentServoMixer[i].targetChannel;
            uint8_t from = currentServoMixer[i].inputSource;
            uint16_t servo_width = servoConf[target].max - servoConf[target].min;
            int16_t min = currentServoMixer[i].min * servo_width / 100 - servo_width / 2;
            int16_t max = currentServoMixer[i].max * servo_width / 100 - servo_width / 2;

            if (currentServoMixer[i].speed == 0) {
                currentOutput[i] = input[from];
            } else {
                if (currentOutput[i] < input[from]) {
                    currentOutput[i] = constrain(currentOutput[i] + currentServoMixer[i].speed, currentOutput[i], input[from]);
                } else if (currentOutput[i] > input[from]) {
                    currentOutput[i] = constrain(currentOutput[i] - currentServoMixer[i].speed, input[from], currentOutput[i]);
                }
            }

            /*
            Flaperon fligh mode
            */
            if (FLIGHT_MODE(FLAPERON) && (target == SERVO_FLAPPERON_1 || target == SERVO_FLAPPERON_2)) {
                int8_t multiplier = 1;

                if (flaperon_throw_inverted == 1) {
                    multiplier = -1;
                }
                currentOutput[i] += flaperon_throw_offset * getFlaperonDirection(target) * multiplier;
            }

            servo[target] += servoDirection(target, from) * constrain(((int32_t)currentOutput[i] * currentServoMixer[i].rate) / 100, min, max);
        } else {
            currentOutput[i] = 0;
        }
    }

    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = ((int32_t)servoConf[i].rate * servo[i]) / 100L;
        servo[i] += determineServoMiddleOrForwardFromChannel(i);
    }
}

void processServoTilt(void) {
    // center at fixed position, or vary either pitch or roll by RC channel
    servo[SERVO_GIMBAL_PITCH] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
    servo[SERVO_GIMBAL_ROLL] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);

    if (IS_RC_MODE_ACTIVE(BOXCAMSTAB)) {
        if (gimbalConfig->mode == GIMBAL_MODE_MIXTILT) {
            servo[SERVO_GIMBAL_PITCH] -= (-(int32_t)servoConf[SERVO_GIMBAL_PITCH].rate) * attitude.values.pitch / 50 - (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll / 50;
            servo[SERVO_GIMBAL_ROLL] += (-(int32_t)servoConf[SERVO_GIMBAL_PITCH].rate) * attitude.values.pitch / 50 + (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll / 50;
        } else {
            servo[SERVO_GIMBAL_PITCH] += (int32_t)servoConf[SERVO_GIMBAL_PITCH].rate * attitude.values.pitch / 50;
            servo[SERVO_GIMBAL_ROLL] += (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll  / 50;
        }
    }
}

bool isServoOutputEnabled(void)
{
    return servoOutputEnabled;
}

bool isMixerUsingServos(void) {
    return mixerUsesServos;
}

void filterServos(void)
{
    int servoIdx;

    if (mixerConfig->servo_lowpass_enable) {
        // Initialize servo lowpass filter (servos are calculated at looptime rate)
        if (!servoFilterIsSet) {
            for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
                biquadFilterInit(&servoFitlerState[servoIdx], mixerConfig->servo_lowpass_freq, gyro.targetLooptime);
            }

            servoFilterIsSet = true;
        }

        for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            // Apply servo lowpass filter and do sanity cheching
            servo[servoIdx] = (int16_t) biquadFilterApply(&servoFitlerState[servoIdx], (float)servo[servoIdx]);
        }
    }

    for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
        servo[servoIdx] = constrain(servo[servoIdx], servoConf[servoIdx].min, servoConf[servoIdx].max);
    }
}
#endif
