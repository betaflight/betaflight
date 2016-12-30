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

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"

#include "rx/rx.h"

#include "io/gimbal.h"
#include "io/motors.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/config_profile.h"
#include "config/feature.h"

//#define MIXER_DEBUG

uint8_t motorCount;

int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

bool motorLimitReached = false;

mixerConfig_t *mixerConfig;
static flight3DConfig_t *flight3DConfig;
static motorConfig_t *motorConfig;
static rxConfig_t *rxConfig;

mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];


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

static motorMixer_t *customMixers;

void mixerUseConfigs(
        flight3DConfig_t *flight3DConfigToUse,
        motorConfig_t *motorConfigToUse,
        mixerConfig_t *mixerConfigToUse,
        rxConfig_t *rxConfigToUse)
{
    flight3DConfig = flight3DConfigToUse;
    motorConfig = motorConfigToUse;
    mixerConfig = mixerConfigToUse;
    rxConfig = rxConfigToUse;
}

bool isMixerEnabled(mixerMode_e mixerMode)
{
#ifdef USE_QUAD_MIXER_ONLY
    UNUSED(mixerMode);
    return true;
#else
    return mixers[mixerMode].enabled;
#endif
}

// mixerInit must be called before servosInit
void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers)
{
    currentMixerMode = mixerMode;
    customMixers = initialCustomMixers;
}

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
        motor_disarmed[i] = feature(FEATURE_3D) ? flight3DConfig->neutral3d : motorConfig->mincommand;
}

void writeMotors(void)
{
    int i;

    for (i = 0; i < motorCount; i++)
        pwmWriteMotor(i, motor[i]);
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
    writeAllMotors(feature(FEATURE_3D) ? flight3DConfig->neutral3d : motorConfig->mincommand);

    delay(50); // give the timers and ESCs a chance to react.
}

void stopPwmAllMotors()
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
            throttleMin = motorConfig->minthrottle;
            throttlePrevious = throttleCommand = rcCommand[THROTTLE];
        } else if (rcCommand[THROTTLE] >= (rxConfig->midrc + flight3DConfig->deadband3d_throttle)) { // Positive handling
            throttleMax = motorConfig->maxthrottle;
            throttleMin = flight3DConfig->deadband3d_high;
            throttlePrevious = throttleCommand = rcCommand[THROTTLE];
        } else if ((throttlePrevious <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle)))  { // Deadband handling from negative to positive
            throttleCommand = throttleMax = flight3DConfig->deadband3d_low;
            throttleMin = motorConfig->minthrottle;
        } else {  // Deadband handling from positive to negative
            throttleMax = motorConfig->maxthrottle;
            throttleCommand = throttleMin = flight3DConfig->deadband3d_high;
        }
    } else {
        throttleCommand = rcCommand[THROTTLE];
        throttleMin = motorConfig->minthrottle;
        throttleMax = motorConfig->maxthrottle;
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
                motor[i] = constrain(motor[i], motorConfig->mincommand, motorConfig->maxthrottle);
            } else if (feature(FEATURE_3D)) {
                if (throttlePrevious <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle)) {
                    motor[i] = constrain(motor[i], motorConfig->minthrottle, flight3DConfig->deadband3d_low);
                } else {
                    motor[i] = constrain(motor[i], flight3DConfig->deadband3d_high, motorConfig->maxthrottle);
                }
            } else {
                motor[i] = constrain(motor[i], motorConfig->minthrottle, motorConfig->maxthrottle);
            }

            // Motor stop handling
            if (feature(FEATURE_MOTOR_STOP) && ARMING_FLAG(ARMED) && !feature(FEATURE_3D) && !isFailsafeActive) {
                if (((rcData[THROTTLE]) < rxConfig->mincheck) || STATE(NAV_MOTOR_STOP_OR_IDLE)) {
                    motor[i] = motorConfig->mincommand;
                }
            }
        }
    } else {
        for (i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}
