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
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/filter.h"

#include "drivers/pwm_output.h"
#include "drivers/system.h"

#include "rx/rx.h"

#include "io/gimbal.h"
#include "io/servos.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "config/feature.h"

extern mixerMode_e currentMixerMode;
extern rxConfig_t *rxConfig;

static servoMixerConfig_t *servoMixerConfig;

static uint8_t servoRuleCount = 0;
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];
static gimbalConfig_t *gimbalConfig;
int16_t servo[MAX_SUPPORTED_SERVOS];
static int useServo;
static servoParam_t *servoConf;


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

static const servoMixer_t servoMixerBI[] = {
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerTri[] = {
    { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerDual[] = {
    { SERVO_DUALCOPTER_LEFT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_DUALCOPTER_RIGHT, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerSingle[] = {
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerGimbal[] = {
    { SERVO_GIMBAL_PITCH, INPUT_GIMBAL_PITCH, 125, 0, 0, 100, 0 },
    { SERVO_GIMBAL_ROLL, INPUT_GIMBAL_ROLL,  125, 0, 0, 100, 0 },
};


const mixerRules_t servoMixers[] = {
    { 0, NULL },                // entry 0
    { COUNT_SERVO_RULES(servoMixerTri), servoMixerTri },       // MULTITYPE_TRI
    { 0, NULL },                // MULTITYPE_QUADP
    { 0, NULL },                // MULTITYPE_QUADX
    { COUNT_SERVO_RULES(servoMixerBI), servoMixerBI },        // MULTITYPE_BI
    { COUNT_SERVO_RULES(servoMixerGimbal), servoMixerGimbal },    // * MULTITYPE_GIMBAL
    { 0, NULL },                // MULTITYPE_Y6
    { 0, NULL },                // MULTITYPE_HEX6
    { COUNT_SERVO_RULES(servoMixerFlyingWing), servoMixerFlyingWing },// * MULTITYPE_FLYING_WING
    { 0, NULL },                // MULTITYPE_Y4
    { 0, NULL },                // MULTITYPE_HEX6X
    { 0, NULL },                // MULTITYPE_OCTOX8
    { 0, NULL },                // MULTITYPE_OCTOFLATP
    { 0, NULL },                // MULTITYPE_OCTOFLATX
    { COUNT_SERVO_RULES(servoMixerAirplane), servoMixerAirplane },  // * MULTITYPE_AIRPLANE
    { 0, NULL },                // * MULTITYPE_HELI_120_CCPM
    { 0, NULL },                // * MULTITYPE_HELI_90_DEG
    { 0, NULL },                // MULTITYPE_VTAIL4
    { 0, NULL },                // MULTITYPE_HEX6H
    { 0, NULL },                // * MULTITYPE_PPM_TO_SERVO
    { COUNT_SERVO_RULES(servoMixerDual), servoMixerDual },      // MULTITYPE_DUALCOPTER
    { COUNT_SERVO_RULES(servoMixerSingle), servoMixerSingle },    // MULTITYPE_SINGLECOPTER
    { 0, NULL },                // MULTITYPE_ATAIL4
    { 0, NULL },                // MULTITYPE_CUSTOM
    { 0, NULL },                // MULTITYPE_CUSTOM_PLANE
    { 0, NULL },                // MULTITYPE_CUSTOM_TRI
    { 0, NULL },
};

static servoMixer_t *customServoMixers;

void servoUseConfigs(servoMixerConfig_t *servoMixerConfigToUse, servoParam_t *servoParamsToUse, struct gimbalConfig_s *gimbalConfigToUse)
{
    servoMixerConfig = servoMixerConfigToUse;
    servoConf = servoParamsToUse;
    gimbalConfig = gimbalConfigToUse;
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

void servoMixerInit(servoMixer_t *initialCustomServoMixers)
{
    customServoMixers = initialCustomServoMixers;

    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo = mixers[currentMixerMode].useServo;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (feature(FEATURE_SERVO_TILT) || feature(FEATURE_CHANNEL_FORWARDING))
        useServo = 1;

    // give all servos a default command
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }
}

void loadCustomServoMixer(void)
{
    // reset settings
    servoRuleCount = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));

    // load custom mixer into currentServoMixer
    for (uint8_t i = 0; i < MAX_SERVO_RULES; i++) {
        // check if done
        if (customServoMixers[i].rate == 0)
            break;

        currentServoMixer[i] = customServoMixers[i];
        servoRuleCount++;
    }
}

void servoConfigureOutput(void)
{
    if (useServo) {
        servoRuleCount = servoMixers[currentMixerMode].servoRuleCount;
        if (servoMixers[currentMixerMode].rule) {
            for (int i = 0; i < servoRuleCount; i++)
                currentServoMixer[i] = servoMixers[currentMixerMode].rule[i];
        }
    }

    // set flag that we're on something with wings
    if (currentMixerMode == MIXER_FLYING_WING ||
        currentMixerMode == MIXER_AIRPLANE ||
        currentMixerMode == MIXER_CUSTOM_AIRPLANE
    ) {
        ENABLE_STATE(FIXED_WING);

        if (currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
            loadCustomServoMixer();
        }
    } else {
        DISABLE_STATE(FIXED_WING);

        if (currentMixerMode == MIXER_CUSTOM_TRI) {
            loadCustomServoMixer();
        }
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

    for (i = 0; i < servoMixers[index].servoRuleCount; i++)
        customServoMixers[i] = servoMixers[index].rule[i];
}

STATIC_UNIT_TESTED void forwardAuxChannelsToServos(uint8_t firstServoIndex)
{
    // start forwarding from this channel
    uint8_t channelOffset = AUX1;

    uint8_t servoOffset;
    for (servoOffset = 0; servoOffset < MAX_AUX_CHANNEL_COUNT && channelOffset < MAX_SUPPORTED_RC_CHANNEL_COUNT; servoOffset++) {
        pwmWriteServo(firstServoIndex + servoOffset, rcData[channelOffset++]);
    }
}

static void updateGimbalServos(uint8_t firstServoIndex)
{
    pwmWriteServo(firstServoIndex + 0, servo[SERVO_GIMBAL_PITCH]);
    pwmWriteServo(firstServoIndex + 1, servo[SERVO_GIMBAL_ROLL]);
}

void writeServos(void)
{
    uint8_t servoIndex = 0;

    switch (currentMixerMode) {
    case MIXER_BICOPTER:
        pwmWriteServo(servoIndex++, servo[SERVO_BICOPTER_LEFT]);
        pwmWriteServo(servoIndex++, servo[SERVO_BICOPTER_RIGHT]);
        break;

    case MIXER_TRI:
    case MIXER_CUSTOM_TRI:
        if (servoMixerConfig->tri_unarmed_servo) {
            // if unarmed flag set, we always move servo
            pwmWriteServo(servoIndex++, servo[SERVO_RUDDER]);
        } else {
            // otherwise, only move servo when copter is armed
            if (ARMING_FLAG(ARMED))
                pwmWriteServo(servoIndex++, servo[SERVO_RUDDER]);
            else
                pwmWriteServo(servoIndex++, 0); // kill servo signal completely.
        }
        break;

    case MIXER_FLYING_WING:
        pwmWriteServo(servoIndex++, servo[SERVO_FLAPPERON_1]);
        pwmWriteServo(servoIndex++, servo[SERVO_FLAPPERON_2]);
        break;

    case MIXER_DUALCOPTER:
        pwmWriteServo(servoIndex++, servo[SERVO_DUALCOPTER_LEFT]);
        pwmWriteServo(servoIndex++, servo[SERVO_DUALCOPTER_RIGHT]);
        break;

    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_AIRPLANE:
        for (int i = SERVO_PLANE_INDEX_MIN; i <= SERVO_PLANE_INDEX_MAX; i++) {
            pwmWriteServo(servoIndex++, servo[i]);
        }
        break;

    case MIXER_SINGLECOPTER:
        for (int i = SERVO_SINGLECOPTER_INDEX_MIN; i <= SERVO_SINGLECOPTER_INDEX_MAX; i++) {
            pwmWriteServo(servoIndex++, servo[i]);
        }
        break;

    default:
        break;
    }

    // Two servos for SERVO_TILT, if enabled
    if (feature(FEATURE_SERVO_TILT) || currentMixerMode == MIXER_GIMBAL) {
        updateGimbalServos(servoIndex);
        servoIndex += 2;
    }

    // forward AUX to remaining servo outputs (not constrained)
    if (feature(FEATURE_CHANNEL_FORWARDING)) {
        forwardAuxChannelsToServos(servoIndex);
        servoIndex += MAX_AUX_CHANNEL_COUNT;
    }
}

STATIC_UNIT_TESTED void servoMixer(void)
{
    int16_t input[INPUT_SOURCE_COUNT]; // Range [-500:+500]
    static int16_t currentOutput[MAX_SERVO_RULES];
    uint8_t i;

    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        // Direct passthru from RX
        input[INPUT_STABILIZED_ROLL] = rcCommand[ROLL];
        input[INPUT_STABILIZED_PITCH] = rcCommand[PITCH];
        input[INPUT_STABILIZED_YAW] = rcCommand[YAW];
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        input[INPUT_STABILIZED_ROLL] = axisPIDf[ROLL] * PID_SERVO_MIXER_SCALING;
        input[INPUT_STABILIZED_PITCH] = axisPIDf[PITCH] * PID_SERVO_MIXER_SCALING;
        input[INPUT_STABILIZED_YAW] = axisPIDf[YAW] * PID_SERVO_MIXER_SCALING;

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

            if (currentServoMixer[i].speed == 0)
                currentOutput[i] = input[from];
            else {
                if (currentOutput[i] < input[from])
                    currentOutput[i] = constrain(currentOutput[i] + currentServoMixer[i].speed, currentOutput[i], input[from]);
                else if (currentOutput[i] > input[from])
                    currentOutput[i] = constrain(currentOutput[i] - currentServoMixer[i].speed, input[from], currentOutput[i]);
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


void servoTable(void)
{
    // airplane / servo mixes
    switch (currentMixerMode) {
    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_FLYING_WING:
    case MIXER_AIRPLANE:
    case MIXER_BICOPTER:
    case MIXER_CUSTOM_TRI:
    case MIXER_TRI:
    case MIXER_DUALCOPTER:
    case MIXER_SINGLECOPTER:
    case MIXER_GIMBAL:
        servoMixer();
        break;

    /*
    case MIXER_GIMBAL:
        servo[SERVO_GIMBAL_PITCH] = (((int32_t)servoConf[SERVO_GIMBAL_PITCH].rate * attitude.values.pitch) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
        servo[SERVO_GIMBAL_ROLL] = (((int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);
        break;
    */

    default:
        break;
    }

    // camera stabilization
    if (feature(FEATURE_SERVO_TILT)) {
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

    // constrain servos
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = constrain(servo[i], servoConf[i].min, servoConf[i].max); // limit the values
    }
}

bool isMixerUsingServos(void)
{
    return useServo;
}

void filterServos(void)
{
    static int16_t servoIdx;
    static bool servoFilterIsSet;
    static biquadFilter_t servoFilter[MAX_SUPPORTED_SERVOS];

#if defined(MIXER_DEBUG)
    uint32_t startTime = micros();
#endif

    if (servoMixerConfig->servo_lowpass_enable) {
        for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            if (!servoFilterIsSet) {
                biquadFilterInitLPF(&servoFilter[servoIdx], servoMixerConfig->servo_lowpass_freq, targetPidLooptime);
                servoFilterIsSet = true;
            }

            servo[servoIdx] = lrintf(biquadFilterApply(&servoFilter[servoIdx], (float)servo[servoIdx]));
            // Sanity check
            servo[servoIdx] = constrain(servo[servoIdx], servoConf[servoIdx].min, servoConf[servoIdx].max);
        }
    }
#if defined(MIXER_DEBUG)
    debug[0] = (int16_t)(micros() - startTime);
#endif
}
#endif
