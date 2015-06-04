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

#include "platform.h"
#include "debug.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"

#include "rx/rx.h"

#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/lowpass.h"

#include "config/runtime_config.h"
#include "config/config.h"

typedef enum {
    SERVO_GIMBAL_PITCH = 0,
    SERVO_GIMBAL_ROLL = 1,
    SERVO_FLAPS = 2,
    SERVO_FLAPPERON_1 = 3,
    SERVO_FLAPPERON_2 = 4,
    SERVO_RUDDER = 5,
    SERVO_ELEVATOR = 6,
    SERVO_THROTTLE = 7, // for internal combustion (IC) planes

    SERVO_BIPLANE_LEFT = 4,
    SERVO_BIPLANE_RIGHT = 5,

    SERVO_DUALCOPTER_LEFT = 4,
    SERVO_DUALCOPTER_RIGHT = 5,
    
    SERVO_SINGLECOPTER_1 = 3,
    SERVO_SINGLECOPTER_2 = 4,
    SERVO_SINGLECOPTER_3 = 5,
    SERVO_SINGLECOPTER_4 = 6,
    
} servoIndex_e;

#define SERVO_PLANE_INDEX_MIN SERVO_FLAPS
#define SERVO_PLANE_INDEX_MAX SERVO_ELEVATOR

#define SERVO_DUALCOPTER_INDEX_MIN SERVO_DUALCOPTER_LEFT
#define SERVO_DUALCOPTER_INDEX_MAX SERVO_DUALCOPTER_RIGHT

#define SERVO_SINGLECOPTER_INDEX_MIN SERVO_SINGLECOPTER_1
#define SERVO_SINGLECOPTER_INDEX_MAX SERVO_SINGLECOPTER_4

#define SERVO_FLAPPERONS_MIN SERVO_FLAPPERON_1
#define SERVO_FLAPPERONS_MAX SERVO_FLAPPERON_2

#define AUX_FORWARD_CHANNEL_TO_SERVO_COUNT 4

//#define MIXER_DEBUG

uint8_t motorCount = 0;
int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

static mixerConfig_t *mixerConfig;
static flight3DConfig_t *flight3DConfig;
static escAndServoConfig_t *escAndServoConfig;
static airplaneConfig_t *airplaneConfig;
static rxConfig_t *rxConfig;

static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];
static mixerMode_e currentMixerMode;

#ifdef USE_SERVOS
static gimbalConfig_t *gimbalConfig;
int16_t servo[MAX_SUPPORTED_SERVOS];
static int useServo;
STATIC_UNIT_TESTED uint8_t servoCount;
static servoParam_t *servoConf;
static lowpass_t lowpassFilters[MAX_SUPPORTED_SERVOS];
#endif

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};
#ifndef USE_QUAD_MIXER_ONLY
static const motorMixer_t mixerTri[] = {
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

static const motorMixer_t mixerBi[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
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

static const motorMixer_t mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};

static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
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
    { 1.0f,  1.0f, -0.5f,  1.0f },          // MIDFRONT_L
    { 1.0f, -0.5f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  0.5f,  1.0f },          // MIDREAR_R
    { 1.0f,  0.5f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  0.5f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -0.5f, -1.0f },          // MIDFRONT_R
    { 1.0f, -0.5f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  0.5f, -1.0f },          // MIDREAR_L
};

static const motorMixer_t mixerVtail4[] = {
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -0.0f },          // FRONT_L
};

static const motorMixer_t mixerAtail4[] = {
    { 1.0f, 0.0f, 1.0f, 1.0f },             // REAR_R
    { 1.0f, -1.0f, -1.0f, 0.0f },           // FRONT_R
    { 1.0f, 0.0f, 1.0f, -1.0f },            // REAR_L
    { 1.0f, 1.0f, -1.0f, -0.0f },           // FRONT_L
};

static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },     // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },     // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },     // LEFT
};

static const motorMixer_t mixerDualcopter[] = {
    { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
    { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
};

// Keep synced with mixerMode_e
const mixer_t mixers[] = {
    // motors, servos, motor mixer
    { 0, 0, NULL },                // entry 0
    { 3, 1, mixerTri },            // MIXER_TRI
    { 4, 0, mixerQuadP },          // MIXER_QUADP
    { 4, 0, mixerQuadX },          // MIXER_QUADX
    { 2, 1, mixerBi },             // MIXER_BI
    { 0, 1, NULL },                // * MIXER_GIMBAL
    { 6, 0, mixerY6 },             // MIXER_Y6
    { 6, 0, mixerHex6P },          // MIXER_HEX6
    { 1, 1, NULL },                // * MIXER_FLYING_WING
    { 4, 0, mixerY4 },             // MIXER_Y4
    { 6, 0, mixerHex6X },          // MIXER_HEX6X
    { 8, 0, mixerOctoX8 },         // MIXER_OCTOX8
    { 8, 0, mixerOctoFlatP },      // MIXER_OCTOFLATP
    { 8, 0, mixerOctoFlatX },      // MIXER_OCTOFLATX
    { 1, 1, NULL },                // * MIXER_AIRPLANE
    { 0, 1, NULL },                // * MIXER_HELI_120_CCPM
    { 0, 1, NULL },                // * MIXER_HELI_90_DEG
    { 4, 0, mixerVtail4 },         // MIXER_VTAIL4
    { 6, 0, mixerHex6H },          // MIXER_HEX6H
    { 0, 1, NULL },                // * MIXER_PPM_TO_SERVO
    { 2, 1, mixerDualcopter },     // MIXER_DUALCOPTER
    { 1, 1, NULL },                // MIXER_SINGLECOPTER
    { 4, 0, mixerAtail4 },         // MIXER_ATAIL4
    { 0, 0, NULL },                // MIXER_CUSTOM
};
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
        airplaneConfig_t *airplaneConfigToUse,
        rxConfig_t *rxConfigToUse)
{
#ifdef USE_SERVOS
    servoConf = servoConfToUse;
    gimbalConfig = gimbalConfigToUse;
#endif
    flight3DConfig = flight3DConfigToUse;
    escAndServoConfig = escAndServoConfigToUse;
    mixerConfig = mixerConfigToUse;
    airplaneConfig = airplaneConfigToUse;
    rxConfig = rxConfigToUse;
}

#ifdef USE_SERVOS
int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    uint8_t channelToForwardFrom = servoConf[servoIndex].forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeConfig.channelCount) {
        return rcData[channelToForwardFrom];
    }

    return servoConf[servoIndex].middle;
}

int servoDirection(servoIndex_e servoIndex, int lr)
{
    // servo.rate is overloaded for servos that don't have a rate, but only need direction
    // bit set = negative, clear = positive
    // rate[2] = ???_direction
    // rate[1] = roll_direction
    // rate[0] = pitch_direction
    // servo.rate is also used as gimbal gain multiplier (yeah)
    if (servoConf[servoIndex].rate & lr)
        return -1;
    else
        return 1;
}
#endif

#ifndef USE_QUAD_MIXER_ONLY
void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers)
{
    currentMixerMode = mixerMode;

    customMixers = initialCustomMixers;

    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo = mixers[currentMixerMode].useServo;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (feature(FEATURE_SERVO_TILT))
        useServo = 1;

    // give all servos a default command
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }
}

void mixerUsePWMOutputConfiguration(pwmOutputConfiguration_t *pwmOutputConfiguration)
{
    int i;

    servoCount = pwmOutputConfiguration->servoCount;

    if (currentMixerMode == MIXER_CUSTOM) {
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

    // set flag that we're on something with wings
    if (currentMixerMode == MIXER_FLYING_WING ||
            currentMixerMode == MIXER_AIRPLANE)
        ENABLE_STATE(FIXED_WING);
    else
        DISABLE_STATE(FIXED_WING);

    mixerResetMotors();
}

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

#else

void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers)
{
    currentMixerMode = mixerMode;

    customMixers = initialCustomMixers;
}

void mixerUsePWMOutputConfiguration(pwmOutputConfiguration_t *pwmOutputConfiguration)
{
    UNUSED(pwmOutputConfiguration);
    motorCount = 4;
#ifdef USE_SERVOS
    servoCount = 0;
#endif

    uint8_t i;
    for (i = 0; i < motorCount; i++) {
        currentMixer[i] = mixerQuadX[i];
    }

    mixerResetMotors();
}
#endif

void mixerResetMotors(void)
{
    int i;
    // set disarmed motor values
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motor_disarmed[i] = feature(FEATURE_3D) ? flight3DConfig->neutral3d : escAndServoConfig->mincommand;
}

#ifdef USE_SERVOS

STATIC_UNIT_TESTED void forwardAuxChannelsToServos(void)
{
    // offset servos based off number already used in mixer types
    // airplane and servo_tilt together can't be used
    int8_t firstServo = servoCount - AUX_FORWARD_CHANNEL_TO_SERVO_COUNT;

    // start forwarding from this channel
    uint8_t channelOffset = AUX1;

    int8_t servoOffset;
    for (servoOffset = 0; servoOffset < AUX_FORWARD_CHANNEL_TO_SERVO_COUNT; servoOffset++) {
        if (firstServo + servoOffset < 0) {
            continue; // there are not enough servos to forward all the AUX channels.
        }
        pwmWriteServo(firstServo + servoOffset, rcData[channelOffset++]);
    }
}

static void updateGimbalServos(void)
{
    pwmWriteServo(0, servo[SERVO_GIMBAL_PITCH]);
    pwmWriteServo(1, servo[SERVO_GIMBAL_ROLL]);
}

void writeServos(void)
{
    if (!useServo)
        return;

    switch (currentMixerMode) {
        case MIXER_BI:
            pwmWriteServo(0, servo[SERVO_BIPLANE_LEFT]);
            pwmWriteServo(1, servo[SERVO_BIPLANE_RIGHT]);
            break;

        case MIXER_TRI:
            if (mixerConfig->tri_unarmed_servo) {
                // if unarmed flag set, we always move servo
                pwmWriteServo(0, servo[SERVO_RUDDER]);
            } else {
                // otherwise, only move servo when copter is armed
                if (ARMING_FLAG(ARMED))
                    pwmWriteServo(0, servo[SERVO_RUDDER]);
                else
                    pwmWriteServo(0, 0); // kill servo signal completely.
            }
            break;

        case MIXER_FLYING_WING:
            pwmWriteServo(0, servo[SERVO_FLAPPERON_1]);
            pwmWriteServo(1, servo[SERVO_FLAPPERON_2]);
            break;

        case MIXER_GIMBAL:
            updateGimbalServos();
            break;

        case MIXER_DUALCOPTER:
            pwmWriteServo(0, servo[SERVO_DUALCOPTER_LEFT]);
            pwmWriteServo(1, servo[SERVO_DUALCOPTER_RIGHT]);
            break;

        case MIXER_AIRPLANE:
            for (int i = SERVO_PLANE_INDEX_MIN; i <= SERVO_PLANE_INDEX_MAX; i++) {
                pwmWriteServo(i - SERVO_PLANE_INDEX_MIN, servo[i]);
            }
            break;

        case MIXER_SINGLECOPTER:
            for (int i = SERVO_SINGLECOPTER_INDEX_MIN; i <= SERVO_SINGLECOPTER_INDEX_MAX; i++) {
                pwmWriteServo(i - SERVO_SINGLECOPTER_INDEX_MIN, servo[i]);
            }
            break;

        default:
            // Two servos for SERVO_TILT, if enabled
            if (feature(FEATURE_SERVO_TILT)) {
                updateGimbalServos();
            }
            break;
    }
}
#endif

void writeMotors(void)
{
    uint8_t i;

    for (i = 0; i < motorCount; i++)
        pwmWriteMotor(i, motor[i]);


    if (feature(FEATURE_ONESHOT125)) {
        pwmCompleteOneshotMotorUpdate(motorCount);
    }
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;

    // Sends commands to all motors
    for (i = 0; i < motorCount; i++)
        motor[i] = mc;
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(escAndServoConfig->mincommand);

    delay(50); // give the timers and ESCs a chance to react.
}

void StopPwmAllMotors()
{
    pwmShutdownPulsesForAllMotors(motorCount);
}

#ifndef USE_QUAD_MIXER_ONLY
static void airplaneMixer(void)
{
    int16_t flapperons[2] = { 0, 0 };
    int i;

    if (!ARMING_FLAG(ARMED))
        servo[SERVO_THROTTLE] = escAndServoConfig->mincommand; // Kill throttle when disarmed
    else
        servo[SERVO_THROTTLE] = constrain(rcCommand[THROTTLE], escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
    motor[0] = servo[SERVO_THROTTLE];

    if (airplaneConfig->flaps_speed) {
        // configure SERVO3 middle point in GUI to using an AUX channel for FLAPS control
        // use servo min, servo max and servo rate for proper endpoints adjust
        static int16_t slow_LFlaps;
        int16_t lFlap = determineServoMiddleOrForwardFromChannel(SERVO_FLAPS);

        lFlap = constrain(lFlap, servoConf[SERVO_FLAPS].min, servoConf[SERVO_FLAPS].max);
        lFlap = escAndServoConfig->servoCenterPulse - lFlap;
        if (slow_LFlaps < lFlap)
            slow_LFlaps += airplaneConfig->flaps_speed;
        else if (slow_LFlaps > lFlap)
            slow_LFlaps -= airplaneConfig->flaps_speed;

        servo[SERVO_FLAPS] = ((int32_t)servoConf[SERVO_FLAPS].rate * slow_LFlaps) / 100L;
        servo[SERVO_FLAPS] += escAndServoConfig->servoCenterPulse;
    }

    if (FLIGHT_MODE(PASSTHRU_MODE)) {   // Direct passthru from RX
        servo[SERVO_FLAPPERON_1] = rcCommand[ROLL] + flapperons[0];
        servo[SERVO_FLAPPERON_2] = rcCommand[ROLL] + flapperons[1];
        servo[SERVO_RUDDER]      = rcCommand[YAW];
        servo[SERVO_ELEVATOR]    = rcCommand[PITCH];
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        servo[SERVO_FLAPPERON_1] = axisPID[ROLL] + flapperons[0];
        servo[SERVO_FLAPPERON_2] = axisPID[ROLL] + flapperons[1];
        servo[SERVO_RUDDER]      = axisPID[YAW];
        servo[SERVO_ELEVATOR]    = axisPID[PITCH];
    }

    for (i = SERVO_PLANE_INDEX_MIN; i <= SERVO_PLANE_INDEX_MAX; i++) {
        servo[i] = ((int32_t)servoConf[i].rate * servo[i]) / 100L; // servo rates
        servo[i] += determineServoMiddleOrForwardFromChannel(i);
    }
}
#endif

void mixTable(void)
{
    uint32_t i;

    if (motorCount >= 4 && mixerConfig->yaw_jump_prevention_limit < YAW_JUMP_PREVENTION_LIMIT_HIGH) {
        // prevent "yaw jump" during yaw correction (500 is disabled jump protection)
        axisPID[YAW] = constrain(axisPID[YAW], -mixerConfig->yaw_jump_prevention_limit - ABS(rcCommand[YAW]), mixerConfig->yaw_jump_prevention_limit + ABS(rcCommand[YAW]));
    }

    // motors for non-servo mixes
    if (motorCount > 1) {
        for (i = 0; i < motorCount; i++) {
            motor[i] =
                rcCommand[THROTTLE] * currentMixer[i].throttle +
                axisPID[PITCH] * currentMixer[i].pitch +
                axisPID[ROLL] * currentMixer[i].roll +
                -mixerConfig->yaw_direction * axisPID[YAW] * currentMixer[i].yaw;
        }
    }

#if !defined(USE_QUAD_MIXER_ONLY) || defined(USE_SERVOS)
    int8_t yawDirection3D = 1;

    // Reverse yaw servo when inverted in 3D mode
    if (feature(FEATURE_3D) && (rcData[THROTTLE] < rxConfig->midrc)) {
        yawDirection3D = -1;
    }

    // airplane / servo mixes
    switch (currentMixerMode) {
        case MIXER_BI:
            servo[SERVO_BIPLANE_LEFT] = (servoDirection(SERVO_BIPLANE_LEFT, 2) * axisPID[YAW]) + (servoDirection(SERVO_BIPLANE_LEFT, 1) * axisPID[PITCH]) + determineServoMiddleOrForwardFromChannel(SERVO_BIPLANE_LEFT);
            servo[SERVO_BIPLANE_RIGHT] = (servoDirection(SERVO_BIPLANE_RIGHT, 2) * axisPID[YAW]) + (servoDirection(SERVO_BIPLANE_RIGHT, 1) * axisPID[PITCH]) + determineServoMiddleOrForwardFromChannel(SERVO_BIPLANE_RIGHT);
            break;

        case MIXER_TRI:
            servo[SERVO_RUDDER] = (servoDirection(SERVO_RUDDER, 1) * axisPID[YAW] * yawDirection3D) + determineServoMiddleOrForwardFromChannel(SERVO_RUDDER);
            break;

        case MIXER_GIMBAL:
            servo[SERVO_GIMBAL_PITCH] = (((int32_t)servoConf[SERVO_GIMBAL_PITCH].rate * inclination.values.pitchDeciDegrees) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
            servo[SERVO_GIMBAL_ROLL] = (((int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * inclination.values.rollDeciDegrees) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);
            break;

        case MIXER_AIRPLANE:
            airplaneMixer();
            break;

        case MIXER_FLYING_WING:
            if (!ARMING_FLAG(ARMED))
                servo[SERVO_THROTTLE] = escAndServoConfig->mincommand;
            else
                servo[SERVO_THROTTLE] = constrain(rcCommand[THROTTLE], escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);

            motor[0] = servo[SERVO_THROTTLE];

            if (FLIGHT_MODE(PASSTHRU_MODE)) {
                // do not use sensors for correction, simple 2 channel mixing
                servo[SERVO_FLAPPERON_1] = (servoDirection(SERVO_FLAPPERON_1, 1) * rcCommand[PITCH]) + (servoDirection(SERVO_FLAPPERON_1, 2) * rcCommand[ROLL]);
                servo[SERVO_FLAPPERON_2] = (servoDirection(SERVO_FLAPPERON_2, 1) * rcCommand[PITCH]) + (servoDirection(SERVO_FLAPPERON_2, 2) * rcCommand[ROLL]);
            } else {
                // use sensors to correct (gyro only or gyro + acc)
                servo[SERVO_FLAPPERON_1] = (servoDirection(SERVO_FLAPPERON_1, 1) * axisPID[PITCH]) + (servoDirection(SERVO_FLAPPERON_1, 2) * axisPID[ROLL]);
                servo[SERVO_FLAPPERON_2] = (servoDirection(SERVO_FLAPPERON_2, 1) * axisPID[PITCH]) + (servoDirection(SERVO_FLAPPERON_2, 2) * axisPID[ROLL]);
            }
            servo[SERVO_FLAPPERON_1] += determineServoMiddleOrForwardFromChannel(SERVO_FLAPPERON_1);
            servo[SERVO_FLAPPERON_2] += determineServoMiddleOrForwardFromChannel(SERVO_FLAPPERON_2);
            break;

        case MIXER_DUALCOPTER:
            for (i = SERVO_DUALCOPTER_INDEX_MIN; i <= SERVO_DUALCOPTER_INDEX_MAX; i++) {
                servo[i] = axisPID[SERVO_DUALCOPTER_INDEX_MAX - i] * servoDirection(i, 1); // mix and setup direction
                servo[i] += determineServoMiddleOrForwardFromChannel(i);
            }
            break;

        case MIXER_SINGLECOPTER:
            for (i = SERVO_SINGLECOPTER_INDEX_MIN; i <= SERVO_SINGLECOPTER_INDEX_MAX; i++) {
                servo[i] = (axisPID[YAW] * servoDirection(i, 2)) + (axisPID[(SERVO_SINGLECOPTER_INDEX_MAX - i) >> 1] * servoDirection(i, 1)); // mix and setup direction
                servo[i] += determineServoMiddleOrForwardFromChannel(i);
            }
            motor[0] = rcCommand[THROTTLE];
            break;

        default:
            break;
    }

    // do camstab
    if (feature(FEATURE_SERVO_TILT)) {
        // center at fixed position, or vary either pitch or roll by RC channel
        servo[SERVO_GIMBAL_PITCH] = determineServoMiddleOrForwardFromChannel(0);
        servo[SERVO_GIMBAL_ROLL] = determineServoMiddleOrForwardFromChannel(1);

        if (IS_RC_MODE_ACTIVE(BOXCAMSTAB)) {
            if (gimbalConfig->gimbal_flags & GIMBAL_MIXTILT) {
                servo[SERVO_GIMBAL_PITCH] -= (-(int32_t)servoConf[SERVO_GIMBAL_PITCH].rate) * inclination.values.pitchDeciDegrees / 50 - (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * inclination.values.rollDeciDegrees / 50;
                servo[SERVO_GIMBAL_ROLL] += (-(int32_t)servoConf[SERVO_GIMBAL_PITCH].rate) * inclination.values.pitchDeciDegrees / 50 + (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * inclination.values.rollDeciDegrees / 50;
            } else {
                servo[SERVO_GIMBAL_PITCH] += (int32_t)servoConf[SERVO_GIMBAL_PITCH].rate * inclination.values.pitchDeciDegrees / 50;
                servo[SERVO_GIMBAL_ROLL] += (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * inclination.values.rollDeciDegrees  / 50;
            }
        }
    }

    // constrain servos
    if (useServo) {
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servo[i] = constrain(servo[i], servoConf[i].min, servoConf[i].max); // limit the values
        }
    }
    // forward AUX1-4 to servo outputs (not constrained)
    if (gimbalConfig->gimbal_flags & GIMBAL_FORWARDAUX) {
        forwardAuxChannelsToServos();
    }
#endif

    if (ARMING_FLAG(ARMED)) {

        bool isFailsafeActive = failsafeIsActive();

        // Find the maximum motor output.
        int16_t maxMotor = motor[0];
        for (i = 1; i < motorCount; i++) {
            // If one motor is above the maxthrottle threshold, we reduce the value
            // of all motors by the amount of overshoot.  That way, only one motor
            // is at max and the relative power of each motor is preserved.
            if (motor[i] > maxMotor) {
                maxMotor = motor[i];
            }
        }

        for (i = 0; i < motorCount; i++) {
            if (maxMotor > escAndServoConfig->maxthrottle) {
                // this is a way to still have good gyro corrections if at least one motor reaches its max.
                motor[i] -= maxMotor - escAndServoConfig->maxthrottle;
            }

            if (feature(FEATURE_3D)) {
                if ((rcData[THROTTLE]) > rxConfig->midrc) {
                    motor[i] = constrain(motor[i], flight3DConfig->deadband3d_high, escAndServoConfig->maxthrottle);
                } else {
                    motor[i] = constrain(motor[i], escAndServoConfig->mincommand, flight3DConfig->deadband3d_low);
                }
            } else {
                if (isFailsafeActive) {
                    motor[i] = constrain(motor[i], escAndServoConfig->mincommand, escAndServoConfig->maxthrottle);
                } else {
                    // If we're at minimum throttle and FEATURE_MOTOR_STOP enabled,
                    // do not spin the motors.
                    motor[i] = constrain(motor[i], escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
                    if ((rcData[THROTTLE]) < rxConfig->mincheck) {
                        if (feature(FEATURE_MOTOR_STOP)) {
                            motor[i] = escAndServoConfig->mincommand;
                        } else if (mixerConfig->pid_at_min_throttle == 0) {
                            motor[i] = escAndServoConfig->minthrottle;
                        }
                    }
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
bool isMixerUsingServos(void)
{
    return useServo;
}
#endif

void filterServos(void)
{
#ifdef USE_SERVOS
    int16_t servoIdx;

#if defined(MIXER_DEBUG)
    uint32_t startTime = micros();
#endif

    if (mixerConfig->servo_lowpass_enable) {
        for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            servo[servoIdx] = (int16_t)lowpassFixed(&lowpassFilters[servoIdx], servo[servoIdx], mixerConfig->servo_lowpass_freq);

            // Sanity check
            servo[servoIdx] = constrain(servo[servoIdx], servoConf[servoIdx].min, servoConf[servoIdx].max);
        }
    }
#if defined(MIXER_DEBUG)
    debug[0] = (int16_t)(micros() - startTime);
#endif

#endif
}

