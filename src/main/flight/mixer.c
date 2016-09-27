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

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

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
#include "sensors/battery.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "fc/runtime_config.h"

#include "config/config.h"
#include "config/feature.h"

uint8_t motorCount;

int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

mixerConfig_t *mixerConfig;
static flight3DConfig_t *flight3DConfig;
static escAndServoConfig_t *escAndServoConfig;
static airplaneConfig_t *airplaneConfig;
rxConfig_t *rxConfig;
static bool syncPwmWithPidLoop = false;

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

static const motorMixer_t mixerBicopter[] = {
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
    { 1.0f,  1.0f, -0.414178f,  1.0f },      // MIDFRONT_L
    { 1.0f, -0.414178f, -1.0f,  1.0f },      // FRONT_R
    { 1.0f, -1.0f,  0.414178f,  1.0f },      // MIDREAR_R
    { 1.0f,  0.414178f,  1.0f,  1.0f },      // REAR_L
    { 1.0f,  0.414178f, -1.0f, -1.0f },      // FRONT_L
    { 1.0f, -1.0f, -0.414178f, -1.0f },      // MIDFRONT_R
    { 1.0f, -0.414178f,  1.0f, -1.0f },      // REAR_R
    { 1.0f,  1.0f,  0.414178f, -1.0f },      // MIDREAR_L
};

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

static const motorMixer_t mixerSingleProp[] = {
    { 1.0f,  0.0f,  0.0f, 0.0f },
};

static const motorMixer_t mixerQuadX1234[] = {
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
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
    { 6, false, mixerY6 },             // MIXER_Y6
    { 6, false, mixerHex6P },          // MIXER_HEX6
    { 1, true,  mixerSingleProp },     // * MIXER_FLYING_WING
    { 4, false, mixerY4 },             // MIXER_Y4
    { 6, false, mixerHex6X },          // MIXER_HEX6X
    { 8, false, mixerOctoX8 },         // MIXER_OCTOX8
    { 8, false, mixerOctoFlatP },      // MIXER_OCTOFLATP
    { 8, false, mixerOctoFlatX },      // MIXER_OCTOFLATX
    { 1, true,  mixerSingleProp },     // * MIXER_AIRPLANE
    { 0, true,  NULL },                // * MIXER_HELI_120_CCPM
    { 0, true,  NULL },                // * MIXER_HELI_90_DEG
    { 4, false, mixerVtail4 },         // MIXER_VTAIL4
    { 6, false, mixerHex6H },          // MIXER_HEX6H
    { 0, true,  NULL },                // * MIXER_PPM_TO_SERVO
    { 2, true,  mixerDualcopter },     // MIXER_DUALCOPTER
    { 1, true,  NULL },                // MIXER_SINGLECOPTER
    { 4, false, mixerAtail4 },         // MIXER_ATAIL4
    { 0, false, NULL },                // MIXER_CUSTOM
    { 2, true,  NULL },                // MIXER_CUSTOM_AIRPLANE
    { 3, true,  NULL },                // MIXER_CUSTOM_TRI
    { 4, false, mixerQuadX1234 },
};
#endif

static motorMixer_t *customMixers;

void mixerUseConfigs(
        flight3DConfig_t *flight3DConfigToUse,
        escAndServoConfig_t *escAndServoConfigToUse,
        mixerConfig_t *mixerConfigToUse,
        airplaneConfig_t *airplaneConfigToUse,
        rxConfig_t *rxConfigToUse)
{
    flight3DConfig = flight3DConfigToUse;
    escAndServoConfig = escAndServoConfigToUse;
    mixerConfig = mixerConfigToUse;
    airplaneConfig = airplaneConfigToUse;
    rxConfig = rxConfigToUse;
}








void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers)
{
    currentMixerMode = mixerMode;

    customMixers = initialCustomMixers;
}

#ifndef USE_QUAD_MIXER_ONLY

void mixerUsePWMOutputConfiguration(bool use_unsyncedPwm)
{
    int i;

    motorCount = 0;

    syncPwmWithPidLoop = !use_unsyncedPwm;

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

void mixerUsePWMOutputConfiguration(bool use_unsyncedPwm)
{
    syncPwmWithPidLoop = !use_unsyncedPwm;

    motorCount = 4;

    uint8_t i;
    for (i = 0; i < motorCount; i++) {
        currentMixer[i] = mixerQuadX[i];
    }

    mixerResetDisarmedMotors();
}
#endif

void mixerResetDisarmedMotors(void)
{
    int i;
    // set disarmed motor values
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motor_disarmed[i] = feature(FEATURE_3D) ? flight3DConfig->neutral3d : escAndServoConfig->mincommand;
}

void writeMotors(void)
{
    uint8_t i;

    for (i = 0; i < motorCount; i++)
        pwmWriteMotor(i, motor[i]);

    if (syncPwmWithPidLoop) {
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
    writeAllMotors(feature(FEATURE_3D) ? flight3DConfig->neutral3d : escAndServoConfig->mincommand);

    delay(50); // give the timers and ESCs a chance to react.
}

void stopPwmAllMotors()
{
    pwmShutdownPulsesForAllMotors(motorCount);
    delayMicroseconds(1500);
}

void mixTable(void *pidProfilePtr)
{
    uint32_t i = 0;
    fix12_t vbatCompensationFactor = 0;
    static fix12_t mixReduction;
    bool use_vbat_compensation = false;
    pidProfile_t *pidProfile = (pidProfile_t *) pidProfilePtr;
    if (batteryConfig && pidProfile->vbatPidCompensation) {
        use_vbat_compensation = true;
        vbatCompensationFactor = calculateVbatPidCompensation();
    }

    bool isFailsafeActive = failsafeIsActive(); // TODO - Find out if failsafe checks are really needed here in mixer code

    // Initial mixer concept by bdoiron74 reused and optimized for Air Mode
    int16_t rollPitchYawMix[MAX_SUPPORTED_MOTORS];
    int16_t rollPitchYawMixMax = 0; // assumption: symetrical about zero.
    int16_t rollPitchYawMixMin = 0;

    if (use_vbat_compensation) rollPitchYawMix[i] = qMultiply(vbatCompensationFactor, rollPitchYawMix[i]);  // Add voltage compensation

    // Find roll/pitch/yaw desired output
    for (i = 0; i < motorCount; i++) {
        rollPitchYawMix[i] =
            axisPID[PITCH] * currentMixer[i].pitch +
            axisPID[ROLL] * currentMixer[i].roll +
            -mixerConfig->yaw_motor_direction * axisPID[YAW] * currentMixer[i].yaw;

        if (use_vbat_compensation) rollPitchYawMix[i] = qMultiply(vbatCompensationFactor, rollPitchYawMix[i]);  // Add voltage compensation

        if (rollPitchYawMix[i] > rollPitchYawMixMax) rollPitchYawMixMax = rollPitchYawMix[i];
        if (rollPitchYawMix[i] < rollPitchYawMixMin) rollPitchYawMixMin = rollPitchYawMix[i];

        if (debugMode == DEBUG_MIXER && i < 5) debug[i] = rollPitchYawMix[i];
    }

    // Scale roll/pitch/yaw uniformly to fit within throttle range
    int16_t rollPitchYawMixRange = rollPitchYawMixMax - rollPitchYawMixMin;
    int16_t throttleRange, throttle;
    int16_t throttleMin, throttleMax;
    static int16_t throttlePrevious = 0;   // Store the last throttle direction for deadband transitions

    // Find min and max throttle based on condition.
    if (feature(FEATURE_3D)) {
        if (!ARMING_FLAG(ARMED)) throttlePrevious = rxConfig->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.

        if ((rcCommand[THROTTLE] <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle))) { // Out of band handling
            throttleMax = flight3DConfig->deadband3d_low;
            throttleMin = escAndServoConfig->minthrottle;
            throttlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] + flight3DConfig->deadband3d_throttle;
        } else if (rcCommand[THROTTLE] >= (rxConfig->midrc + flight3DConfig->deadband3d_throttle)) { // Positive handling
            throttleMax = escAndServoConfig->maxthrottle;
            throttleMin = flight3DConfig->deadband3d_high;
            throttlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] - flight3DConfig->deadband3d_throttle;
        } else if ((throttlePrevious <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle)))  { // Deadband handling from negative to positive
            throttle = throttleMax = flight3DConfig->deadband3d_low;
            throttleMin = escAndServoConfig->minthrottle;
        } else {  // Deadband handling from positive to negative
            throttleMax = escAndServoConfig->maxthrottle;
            throttle = throttleMin = flight3DConfig->deadband3d_high;
        }
    } else {
        throttle = rcCommand[THROTTLE];
        throttleMin = escAndServoConfig->minthrottle;
        throttleMax = escAndServoConfig->maxthrottle;
    }

    throttleRange = throttleMax - throttleMin;

    if (rollPitchYawMixRange > throttleRange) {
        mixReduction = qConstruct(throttleRange, rollPitchYawMixRange);

        for (i = 0; i < motorCount; i++) {
            rollPitchYawMix[i] =  qMultiply(mixReduction,rollPitchYawMix[i]);
        }
        // Get the maximum correction by setting offset to center
        throttleMin = throttleMax = throttleMin + (throttleRange / 2);
    } else {
        throttleMin = throttleMin + (rollPitchYawMixRange / 2);
        throttleMax = throttleMax - (rollPitchYawMixRange / 2);
    }

    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (i = 0; i < motorCount; i++) {
        motor[i] = rollPitchYawMix[i] + constrain(throttle * currentMixer[i].throttle, throttleMin, throttleMax);

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
        if (feature(FEATURE_MOTOR_STOP) && ARMING_FLAG(ARMED) && !feature(FEATURE_3D) && !isAirmodeActive()) {
            if (((rcData[THROTTLE]) < rxConfig->mincheck)) {
                motor[i] = escAndServoConfig->mincommand;
            }
        }
    }

    // Anti Desync feature for ESC's. Limit rapid throttle changes
    if (escAndServoConfig->maxEscThrottleJumpMs) {
        const int16_t maxThrottleStep = constrain(escAndServoConfig->maxEscThrottleJumpMs / (1000 / targetPidLooptime), 2, 10000);

        // Only makes sense when it's within the range
        if (maxThrottleStep < throttleRange) {
            static int16_t motorPrevious[MAX_SUPPORTED_MOTORS];

            motor[i] = constrain(motor[i], escAndServoConfig->minthrottle, motorPrevious[i] + maxThrottleStep);  // Only limit accelerating situation
            motorPrevious[i] = motor[i];
        }
    }

    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

