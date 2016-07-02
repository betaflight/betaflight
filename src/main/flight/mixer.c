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
#include <math.h>

#include <platform.h>
#include "build/debug.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"

#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"

#include "rx/rx.h"

#include "io/gimbal.h"
#include "io/motor_and_servo.h"

#include "fc/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "config/parameter_group_ids.h"
#include "config/feature.h"
#include "config/config_reset.h"

#include "fc/runtime_config.h"
#include "fc/config.h"

//#define MIXER_DEBUG

uint8_t motorCount;

int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

bool motorLimitReached;

motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];

PG_REGISTER_ARR(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(motor3DConfig_t, motor3DConfig, PG_MOTOR_3D_CONFIG, 0);

PG_RESET_TEMPLATE(motor3DConfig_t, motor3DConfig,
    .deadband3d_low = 1406,
    .deadband3d_high = 1514,
    .neutral3d = 1460,
);


#ifdef USE_SERVOS
PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixerMode = MIXER_QUADX,
    .pid_at_min_throttle = 1,
    .yaw_motor_direction = 1,
    .yaw_jump_prevention_limit = 200,

    .tri_unarmed_servo = 1,
    .servo_lowpass_freq = 400.0f,
);
#else
PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixerMode = MIXER_QUADX,
    .pid_at_min_throttle = 1,
    .yaw_motor_direction = 1,
    .yaw_jump_prevention_limit = 200,
);
#endif

/* QuadX
4CW   2CCW
   \ /
    X
   / \
3CCW  1CW
*/
static const motorMixer_t mixerQuadX[] = {
//throttle,  roll, pitch,   yaw
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R  (M1)
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R (M2)
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L  (M3)
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L (M4)
};

#ifndef USE_QUAD_MIXER_ONLY
/* QuadP
    4CW
     |
3CCW-+-2CCW
     |
    1CW
*/
static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

/* Vtail4
4CCW-----2CW
     ||
     ||
3CW\ || /1CCW
    \||/
*/
static const motorMixer_t mixerVtail4[] = {
    { 1.0f, -0.58f,  0.58f,  1.0f },        // REAR_R
    { 1.0f, -0.46f, -0.39f, -0.5f },        // FRONT_R
    { 1.0f,  0.58f,  0.58f, -1.0f },        // REAR_L
    { 1.0f,  0.46f, -0.39f,  0.5f },        // FRONT_L
};

/* Atail4
 4CW----2CCW
     ||
     ||
     /\
3CCW/  \1CW
*/
static const motorMixer_t mixerAtail4[] = {
    { 1.0f, -0.58f,  0.58f, -1.0f },        // REAR_R
    { 1.0f, -0.46f, -0.39f,  0.5f },        // FRONT_R
    { 1.0f,  0.58f,  0.58f,  1.0f },        // REAR_L
    { 1.0f,  0.46f, -0.39f, -0.5f },        // FRONT_L
};

/* Y4
 4CW----2CCW
     ||
     ||
    1CW
    3CCW
*/
static const motorMixer_t mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};

#if (MAX_SUPPORTED_MOTORS >= 6)
static const motorMixer_t mixerY6[] = {
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};

static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },          // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },          // LEFT
};

static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
    { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
    { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
};

static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};
#endif

#if (MAX_SUPPORTED_MOTORS >= 8)
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
    { 1.0f,  0.707107f, -0.707107f,  1.0f },// FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },// FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },// REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },// REAR_L
    { 1.0f,  0.0f,      -1.0f,      -1.0f },// FRONT
    { 1.0f, -1.0f,       0.0f,      -1.0f },// RIGHT
    { 1.0f,  0.0f,       1.0f,      -1.0f },// REAR
    { 1.0f,  1.0f,       0.0f,      -1.0f },// LEFT
};

static const motorMixer_t mixerOctoFlatX[] = {
    { 1.0f,  1.0f,      -0.414178f,  1.0f },// MIDFRONT_L
    { 1.0f, -0.414178f, -1.0f,       1.0f },// FRONT_R
    { 1.0f, -1.0f,       0.414178f,  1.0f },// MIDREAR_R
    { 1.0f,  0.414178f,  1.0f,       1.0f },// REAR_L
    { 1.0f,  0.414178f, -1.0f,      -1.0f },// FRONT_L
    { 1.0f, -1.0f,      -0.414178f, -1.0f },// MIDFRONT_R
    { 1.0f, -0.414178f,  1.0f,      -1.0f },// REAR_R
    { 1.0f,  1.0f,       0.414178f, -1.0f },// MIDREAR_L
};
#endif

static const motorMixer_t mixerSingleProp[] = {
    { 1.0f,  0.0f,  0.0f, 0.0f },
};

static const motorMixer_t mixerDualcopter[] = {
    { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
    { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
};

static const motorMixer_t mixerBicopter[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};

static const motorMixer_t mixerTricopter[] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

// Keep synced with mixerMode_e
const mixer_t mixers[] = {
    // motors, use servo, motor mixer
    { 0, false, NULL },                // entry 0
    { 3, true,  mixerTricopter },      // MIXER_TRI
    { 4, false, mixerQuadP },          // MIXER_QUADP
    { 4, false, mixerQuadX },          // MIXER_QUADX
    { 2, true,  mixerBicopter },       // MIXER_BICOPTER
    { 0, true,  NULL },                // * MIXER_GIMBAL
#if (MAX_SUPPORTED_MOTORS >= 6)
    { 6, false, mixerY6 },             // MIXER_Y6
    { 6, false, mixerHex6P },          // MIXER_HEX6
#else
    { 6, false, NULL },                // MIXER_Y6
    { 6, false, NULL },                // MIXER_HEX6
#endif
    { 1, true,  mixerSingleProp },     // * MIXER_FLYING_WING
    { 4, false, mixerY4 },             // MIXER_Y4
#if (MAX_SUPPORTED_MOTORS >= 6)
    { 6, false, mixerHex6X },          // MIXER_HEX6X
#else
    { 6, false, NULL },                // MIXER_HEX6X
#endif
#if (MAX_SUPPORTED_MOTORS >= 8)
    { 8, false, mixerOctoX8 },         // MIXER_OCTOX8
    { 8, false, mixerOctoFlatP },      // MIXER_OCTOFLATP
    { 8, false, mixerOctoFlatX },      // MIXER_OCTOFLATX
#else
    { 8, false, NULL },                // MIXER_OCTOX8
    { 8, false, NULL },                // MIXER_OCTOFLATP
    { 8, false, NULL },                // MIXER_OCTOFLATX
#endif
    { 1, true,  mixerSingleProp },     // * MIXER_AIRPLANE
    { 0, true,  NULL },                // * MIXER_HELI_120_CCPM
    { 0, true,  NULL },                // * MIXER_HELI_90_DEG
    { 4, false, mixerVtail4 },         // MIXER_VTAIL4
#if (MAX_SUPPORTED_MOTORS >= 6)
    { 6, false, mixerHex6H },          // MIXER_HEX6H
#else
    { 6, false, NULL },                // MIXER_HEX6H
#endif
    { 0, true,  NULL },                // * MIXER_PPM_TO_SERVO
    { 2, true,  mixerDualcopter },     // MIXER_DUALCOPTER
    { 1, true,  NULL },                // MIXER_SINGLECOPTER
    { 4, false, mixerAtail4 },         // MIXER_ATAIL4
    { 0, false, NULL },                // MIXER_CUSTOM
    { 2, true,  NULL },                // MIXER_CUSTOM_AIRPLANE
    { 3, true,  NULL },                // MIXER_CUSTOM_TRI
};
#endif

motorMixer_t *customMixers;

void mixerInit(motorMixer_t *initialCustomMixers)
{
    customMixers = initialCustomMixers;
}

#if !defined(USE_SERVOS) || defined(USE_QUAD_MIXER_ONLY)
void mixerUsePWMIOConfiguration(pwmIOConfiguration_t *pwmIOConfiguration)
{
    UNUSED(pwmIOConfiguration);
    motorCount = 4;
    uint8_t i;
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
        motor_disarmed[i] = feature(FEATURE_3D) ? motor3DConfig()->neutral3d : motorAndServoConfig()->mincommand;
}

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
    writeAllMotors(feature(FEATURE_3D) ? motor3DConfig()->neutral3d : motorAndServoConfig()->mincommand);

    delay(50); // give the timers and ESCs a chance to react.
}

void StopPwmAllMotors()
{
    pwmShutdownPulsesForAllMotors(motorCount);
}

uint16_t mixConstrainMotorForFailsafeCondition(uint8_t motorIndex)
{
    return constrain(motor[motorIndex], motorAndServoConfig()->mincommand, motorAndServoConfig()->maxthrottle);
}

void mixTable(void)
{
    uint32_t i;

    bool isFailsafeActive = failsafeIsActive();

    if (motorCount >= 4 && mixerConfig()->yaw_jump_prevention_limit < YAW_JUMP_PREVENTION_LIMIT_HIGH) {
        // prevent "yaw jump" during yaw correction
        axisPID[FD_YAW] = constrain(axisPID[FD_YAW], -mixerConfig()->yaw_jump_prevention_limit - ABS(rcCommand[YAW]), mixerConfig()->yaw_jump_prevention_limit + ABS(rcCommand[YAW]));
    }

    if (rcModeIsActive(BOXAIRMODE)) {
        // Initial mixer concept by bdoiron74 reused and optimized for Air Mode
        int16_t rollPitchYawMix[MAX_SUPPORTED_MOTORS];
        int16_t rollPitchYawMixMax = 0; // assumption: symetrical about zero.
        int16_t rollPitchYawMixMin = 0;

        // Find roll/pitch/yaw desired output
        for (i = 0; i < motorCount; i++) {
            rollPitchYawMix[i] =
                axisPID[FD_PITCH] * currentMixer[i].pitch +
                axisPID[FD_ROLL] * currentMixer[i].roll +
                -mixerConfig()->yaw_motor_direction * axisPID[FD_YAW] * currentMixer[i].yaw;

            if (rollPitchYawMix[i] > rollPitchYawMixMax) rollPitchYawMixMax = rollPitchYawMix[i];
            if (rollPitchYawMix[i] < rollPitchYawMixMin) rollPitchYawMixMin = rollPitchYawMix[i];
        }

        // Scale roll/pitch/yaw uniformly to fit within throttle range
        int16_t rollPitchYawMixRange = rollPitchYawMixMax - rollPitchYawMixMin;
        int16_t throttleRange, throttle;
        int16_t throttleMin, throttleMax;
        static int16_t throttlePrevious = 0;   // Store the last throttle direction for deadband transitions in 3D.

        // Find min and max throttle based on condition. Use rcData for 3D to prevent loss of power due to min_check
        if (feature(FEATURE_3D)) {
            if (!ARMING_FLAG(ARMED)) throttlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.

            if ((rcData[THROTTLE] <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle))) { // Out of band handling
                throttleMax = motor3DConfig()->deadband3d_low;
                throttleMin = motorAndServoConfig()->minthrottle;
                throttlePrevious = throttle = rcData[THROTTLE];
            } else if (rcData[THROTTLE] >= (rxConfig()->midrc + rcControlsConfig()->deadband3d_throttle)) { // Positive handling
                throttleMax = motorAndServoConfig()->maxthrottle;
                throttleMin = motor3DConfig()->deadband3d_high;
                throttlePrevious = throttle = rcData[THROTTLE];
            } else if ((throttlePrevious <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle)))  { // Deadband handling from negative to positive
                throttle = throttleMax = motor3DConfig()->deadband3d_low;
                throttleMin = motorAndServoConfig()->minthrottle;
            } else {  // Deadband handling from positive to negative
                throttleMax = motorAndServoConfig()->maxthrottle;
                throttle = throttleMin = motor3DConfig()->deadband3d_high;
            }
        } else {
            throttle = rcCommand[THROTTLE];
            throttleMin = motorAndServoConfig()->minthrottle;
            throttleMax = motorAndServoConfig()->maxthrottle;
        }

        throttleRange = throttleMax - throttleMin;

        if (rollPitchYawMixRange > throttleRange) {
            motorLimitReached = true;
            float mixReduction = (float) throttleRange / rollPitchYawMixRange;
            for (i = 0; i < motorCount; i++) {
                rollPitchYawMix[i] =  lrintf((float) rollPitchYawMix[i] * mixReduction);
            }
            // Get the maximum correction by setting throttle offset to center.
            throttleMin = throttleMax = throttleMin + (throttleRange / 2);
        } else {
            motorLimitReached = false;
            throttleMin = throttleMin + (rollPitchYawMixRange / 2);
            throttleMax = throttleMax - (rollPitchYawMixRange / 2);
        }

        // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
        // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
        for (i = 0; i < motorCount; i++) {
            motor[i] = rollPitchYawMix[i] + constrain(throttle * currentMixer[i].throttle, throttleMin, throttleMax);

            if (isFailsafeActive) {
                motor[i] = mixConstrainMotorForFailsafeCondition(i);
            } else if (feature(FEATURE_3D)) {
                if (throttlePrevious <= (rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle)) {
                    motor[i] = constrain(motor[i], motorAndServoConfig()->minthrottle, motor3DConfig()->deadband3d_low);
                } else {
                    motor[i] = constrain(motor[i], motor3DConfig()->deadband3d_high, motorAndServoConfig()->maxthrottle);
                }
            } else {
                motor[i] = constrain(motor[i], motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);
            }
        }
    } else {
        // motors for non-servo mixes
        for (i = 0; i < motorCount; i++) {
            motor[i] =
                rcCommand[THROTTLE] * currentMixer[i].throttle +
                axisPID[FD_PITCH] * currentMixer[i].pitch +
                axisPID[FD_ROLL] * currentMixer[i].roll +
                -mixerConfig()->yaw_motor_direction * axisPID[FD_YAW] * currentMixer[i].yaw;
        }

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

        int16_t maxThrottleDifference = 0;
        if (maxMotor > motorAndServoConfig()->maxthrottle) {
            maxThrottleDifference = maxMotor - motorAndServoConfig()->maxthrottle;
        }

        for (i = 0; i < motorCount; i++) {
            // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motor[i] -= maxThrottleDifference;

            if (feature(FEATURE_3D)) {
                if (mixerConfig()->pid_at_min_throttle
                        || rcData[THROTTLE] <= rxConfig()->midrc - rcControlsConfig()->deadband3d_throttle
                        || rcData[THROTTLE] >= rxConfig()->midrc + rcControlsConfig()->deadband3d_throttle) {
                    if (rcData[THROTTLE] > rxConfig()->midrc) {
                        motor[i] = constrain(motor[i], motor3DConfig()->deadband3d_high, motorAndServoConfig()->maxthrottle);
                    } else {
                        motor[i] = constrain(motor[i], motorAndServoConfig()->mincommand, motor3DConfig()->deadband3d_low);
                    }
                } else {
                    if (rcData[THROTTLE] > rxConfig()->midrc) {
                        motor[i] = motor3DConfig()->deadband3d_high;
                    } else {
                        motor[i] = motor3DConfig()->deadband3d_low;
                    }
                }
            } else {
                if (isFailsafeActive) {
                    motor[i] = mixConstrainMotorForFailsafeCondition(i);
                } else {
                    // If we're at minimum throttle and FEATURE_MOTOR_STOP enabled,
                    // do not spin the motors.
                    motor[i] = constrain(motor[i], motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);
                    if ((rcData[THROTTLE]) < rxConfig()->mincheck) {
                        if (feature(FEATURE_MOTOR_STOP)) {
                            motor[i] = motorAndServoConfig()->mincommand;
                        } else if (mixerConfig()->pid_at_min_throttle == 0) {
                            motor[i] = motorAndServoConfig()->minthrottle;
                        }
                    }
                }
            }
        }
    }


    /* Disarmed for all mixers */
    if (!ARMING_FLAG(ARMED)) {
        for (i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }

    // motor outputs are used as sources for servo mixing, so motors must be calculated before servos.

#if !defined(USE_QUAD_MIXER_ONLY) && defined(USE_SERVOS)
    servoMixTable();
#endif
}

