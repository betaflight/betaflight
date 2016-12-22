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

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/system.h"
#include "drivers/pwm_output.h"

#include "io/motors.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "config/feature.h"
#include "config/config_master.h"

#define EXTERNAL_DSHOT_CONVERSION_FACTOR 2
// (minimum output value(1001) - (minimum input value(48) / conversion factor(2))
#define EXTERNAL_DSHOT_CONVERSION_OFFSET 977
#define EXTERNAL_CONVERSION_MIN_VALUE 1000
#define EXTERNAL_CONVERSION_MAX_VALUE 2000

uint8_t motorCount;

int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

static mixerConfig_t *mixerConfig;
static flight3DConfig_t *flight3DConfig;
static motorConfig_t *motorConfig;
static airplaneConfig_t *airplaneConfig;
rxConfig_t *rxConfig;
static bool syncMotorOutputWithPidLoop = false;

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

static uint16_t disarmMotorOutput, minMotorOutputNormal, maxMotorOutputNormal, deadbandMotor3dHigh, deadbandMotor3dLow;
static float rcCommandThrottleRange;

bool isMotorProtocolDshot(void) {
#ifdef USE_DSHOT
    if (motorConfig->motorPwmProtocol == PWM_TYPE_DSHOT150 || motorConfig->motorPwmProtocol == PWM_TYPE_DSHOT300 || motorConfig->motorPwmProtocol == PWM_TYPE_DSHOT600)
        return true;
    else
#endif
        return false;
}

// Add here scaled ESC outputs for digital protol
void initEscEndpoints(void) {
#ifdef USE_DSHOT
    if (isMotorProtocolDshot()) {
        disarmMotorOutput = DSHOT_DISARM_COMMAND;
        minMotorOutputNormal = DSHOT_MIN_THROTTLE + motorConfig->digitalIdleOffset;
        maxMotorOutputNormal = DSHOT_MAX_THROTTLE;
	    deadbandMotor3dHigh = DSHOT_3D_MIN_NEGATIVE; // TODO - Not working yet !! Mixer requires some throttle rescaling changes
	    deadbandMotor3dLow = DSHOT_3D_MAX_POSITIVE;  // TODO - Not working yet !! Mixer requires some throttle rescaling changes
    } else
#endif
    {
        disarmMotorOutput = (feature(FEATURE_3D)) ? flight3DConfig->neutral3d : motorConfig->mincommand;
        minMotorOutputNormal = motorConfig->minthrottle;
        maxMotorOutputNormal = motorConfig->maxthrottle;
        deadbandMotor3dHigh = flight3DConfig->deadband3d_high;
        deadbandMotor3dLow = flight3DConfig->deadband3d_low;
    }

    rcCommandThrottleRange = (2000 - rxConfig->mincheck);
}

void mixerUseConfigs(
        flight3DConfig_t *flight3DConfigToUse,
        motorConfig_t *motorConfigToUse,
        mixerConfig_t *mixerConfigToUse,
        airplaneConfig_t *airplaneConfigToUse,
        rxConfig_t *rxConfigToUse)
{
    flight3DConfig = flight3DConfigToUse;
    motorConfig = motorConfigToUse;
    mixerConfig = mixerConfigToUse;
    airplaneConfig = airplaneConfigToUse;
    rxConfig = rxConfigToUse;
    initEscEndpoints();
}

void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers)
{
    currentMixerMode = mixerMode;

    customMixers = initialCustomMixers;
}

#ifndef USE_QUAD_MIXER_ONLY

void mixerConfigureOutput(void)
{
    int i;

    motorCount = 0;

    syncMotorOutputWithPidLoop = pwmIsSynced();

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
void mixerConfigureOutput(void)
{
    syncMotorOutputWithPidLoop = pwmIsSynced();
    
    motorCount = QUAD_MOTOR_COUNT;

    for (uint8_t i = 0; i < motorCount; i++) {
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
        motor_disarmed[i] = disarmMotorOutput;
}

void writeMotors(void)
{
	for (uint8_t i = 0; i < motorCount; i++) {
        pwmWriteMotor(i, motor[i]);
	}

    if (syncMotorOutputWithPidLoop) {
        pwmCompleteMotorUpdate(motorCount);
    }
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (uint8_t i = 0; i < motorCount; i++) {
        motor[i] = mc;
    }

    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(disarmMotorOutput);

    delay(50); // give the timers and ESCs a chance to react.
}

void stopPwmAllMotors(void)
{
    pwmShutdownPulsesForAllMotors(motorCount);
    delayMicroseconds(1500);
}

void mixTable(pidProfile_t *pidProfile)
{
    uint32_t i = 0;
    float vbatCompensationFactor = 1;

    // Scale roll/pitch/yaw uniformly to fit within throttle range
    // Initial mixer concept by bdoiron74 reused and optimized for Air Mode
    float motorMix[MAX_SUPPORTED_MOTORS], motorMixMax = 0, motorMixMin = 0;
    float throttleRange = 0, throttle = 0;
    float motorOutputRange, motorMixRange;
    uint16_t motorOutputMin, motorOutputMax;
    static uint16_t throttlePrevious = 0;   // Store the last throttle direction for deadband transitions

    // Find min and max throttle based on condition.
    if (feature(FEATURE_3D)) {
        if (!ARMING_FLAG(ARMED)) throttlePrevious = rxConfig->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.

        if ((rcCommand[THROTTLE] <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle))) { // Out of band handling
            motorOutputMax = deadbandMotor3dLow;
            motorOutputMin = minMotorOutputNormal;
            throttlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] + flight3DConfig->deadband3d_throttle;
        } else if (rcCommand[THROTTLE] >= (rxConfig->midrc + flight3DConfig->deadband3d_throttle)) { // Positive handling
            motorOutputMax = maxMotorOutputNormal;
            motorOutputMin = deadbandMotor3dHigh;
            throttlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] - flight3DConfig->deadband3d_throttle;
        } else if ((throttlePrevious <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle)))  { // Deadband handling from negative to positive
            throttle = deadbandMotor3dLow;
            motorOutputMin = motorOutputMax = minMotorOutputNormal;
        } else {  // Deadband handling from positive to negative
            motorOutputMax = maxMotorOutputNormal;
            throttle = motorOutputMin = deadbandMotor3dHigh;
        }
    } else {
        throttle = rcCommand[THROTTLE];
        motorOutputMin = minMotorOutputNormal;
        motorOutputMax = maxMotorOutputNormal;
    }

    motorOutputRange = motorOutputMax - motorOutputMin;
    throttle = constrainf((throttle - rxConfig->mincheck) / rcCommandThrottleRange, 0.0f, 1.0f);
    throttleRange = motorOutputMax - motorOutputMin;

    // Limit the PIDsum
    for (int axis = 0; axis < 3; axis++)
        axisPIDf[axis] = constrainf(axisPIDf[axis] / PID_MIXER_SCALING, -pidProfile->pidSumLimit, pidProfile->pidSumLimit);

    // Calculate voltage compensation
    if (batteryConfig && pidProfile->vbatPidCompensation) vbatCompensationFactor = calculateVbatPidCompensation();

    // Find roll/pitch/yaw desired output
    for (i = 0; i < motorCount; i++) {
        motorMix[i] =
            axisPIDf[PITCH] * currentMixer[i].pitch +
            axisPIDf[ROLL] * currentMixer[i].roll +
            -mixerConfig->yaw_motor_direction * axisPIDf[YAW] * currentMixer[i].yaw;

        if (vbatCompensationFactor > 1.0f) motorMix[i] *= vbatCompensationFactor;  // Add voltage compensation

        if (motorMix[i] > motorMixMax) motorMixMax = motorMix[i];
        if (motorMix[i] < motorMixMin) motorMixMin = motorMix[i];
    }

    motorMixRange = motorMixMax - motorMixMin;

    if (motorMixRange > 1.0f) {
        for (i = 0; i < motorCount; i++) {
            motorMix[i] /= motorMixRange;
        }
        // Get the maximum correction by setting offset to center
        throttle = 0.5f;
    } else {
        float throttleLimitOffset = motorMixRange / 2.0f;
        throttle = constrainf(throttle, 0.0f + throttleLimitOffset, 1.0f - throttleLimitOffset);
    }

    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (i = 0; i < motorCount; i++) {
        motor[i] = lrintf( motorOutputMin + (motorOutputRange * (motorMix[i] + (throttle * currentMixer[i].throttle))) );

        if (failsafeIsActive()) {
            if (isMotorProtocolDshot())
                motor[i] = (motor[i] < motorOutputMin) ? disarmMotorOutput : motor[i]; // Prevent getting into special reserved range

            motor[i] = constrain(motor[i], disarmMotorOutput, motorOutputMax);
        } else {
            motor[i] = constrain(motor[i], motorOutputMin, motorOutputMax);
        }

        // Motor stop handling
        if (feature(FEATURE_MOTOR_STOP) && ARMING_FLAG(ARMED) && !feature(FEATURE_3D) && !isAirmodeActive()) {
            if (((rcData[THROTTLE]) < rxConfig->mincheck)) {
                motor[i] = disarmMotorOutput;
            }
        }
    }

    // Anti Desync feature for ESC's. Limit rapid throttle changes
    if (motorConfig->maxEscThrottleJumpMs) {
        const int16_t maxThrottleStep = constrain(motorConfig->maxEscThrottleJumpMs / (1000 / targetPidLooptime), 2, 10000);

        // Only makes sense when it's within the range
        if (maxThrottleStep < throttleRange) {
            static int16_t motorPrevious[MAX_SUPPORTED_MOTORS];

            motor[i] = constrain(motor[i], motorOutputMin, motorPrevious[i] + maxThrottleStep);  // Only limit accelerating situation
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

uint16_t convertExternalToMotor(uint16_t externalValue)
{
    uint16_t motorValue = externalValue;
#ifdef USE_DSHOT
    if (isMotorProtocolDshot()) {
        motorValue = externalValue <= EXTERNAL_CONVERSION_MIN_VALUE ? DSHOT_DISARM_COMMAND : constrain((externalValue - EXTERNAL_DSHOT_CONVERSION_OFFSET) * EXTERNAL_DSHOT_CONVERSION_FACTOR, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    }
#endif

    return motorValue;
}

uint16_t convertMotorToExternal(uint16_t motorValue)
{
    uint16_t externalValue = motorValue;
#ifdef USE_DSHOT
    if (isMotorProtocolDshot()) {
        externalValue = motorValue < DSHOT_MIN_THROTTLE ? EXTERNAL_CONVERSION_MIN_VALUE : constrain((motorValue / EXTERNAL_DSHOT_CONVERSION_FACTOR) + EXTERNAL_DSHOT_CONVERSION_OFFSET, EXTERNAL_CONVERSION_MIN_VALUE + 1, EXTERNAL_CONVERSION_MAX_VALUE);
    }
#endif

    return externalValue;
}
