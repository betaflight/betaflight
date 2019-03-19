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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/pwm_output.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "io/motors.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/core.h"
#include "fc/rc.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);

#define DYN_LPF_THROTTLE_STEPS           100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 5000 // minimum of 5ms between updates

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixerMode = DEFAULT_MIXER,
    .yaw_motors_reversed = false,
    .crashflip_motor_percent = 0,
);

PG_REGISTER_WITH_RESET_FN(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);

void pgResetFn_motorConfig(motorConfig_t *motorConfig)
{
#ifdef BRUSHED_MOTORS
    motorConfig->minthrottle = 1000;
    motorConfig->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
    motorConfig->dev.motorPwmProtocol = PWM_TYPE_BRUSHED;
    motorConfig->dev.useUnsyncedPwm = true;
#else
#ifdef USE_BRUSHED_ESC_AUTODETECT
    if (getDetectedMotorType() == MOTOR_BRUSHED) {
        motorConfig->minthrottle = 1000;
        motorConfig->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfig->dev.motorPwmProtocol = PWM_TYPE_BRUSHED;
        motorConfig->dev.useUnsyncedPwm = true;
    } else
#endif
    {
        motorConfig->minthrottle = 1070;
        motorConfig->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
        motorConfig->dev.motorPwmProtocol = PWM_TYPE_ONESHOT125;
    }
#endif
    motorConfig->maxthrottle = 2000;
    motorConfig->mincommand = 1000;
    motorConfig->digitalIdleOffsetValue = 550;
#ifdef USE_DSHOT_DMAR
    motorConfig->dev.useBurstDshot = ENABLE_DSHOT_DMAR;
#endif

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS; motorIndex++) {
        motorConfig->dev.ioTags[motorIndex] = timerioTagGetByUsage(TIM_USE_MOTOR, motorIndex);
    }

    motorConfig->motorPoleCount = 14;   // Most brushes motors that we use are 14 poles
}

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);

#define PWM_RANGE_MID 1500

static FAST_RAM_ZERO_INIT uint8_t motorCount;
static FAST_RAM_ZERO_INIT float motorMixRange;

float FAST_RAM_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];

#ifdef USE_LAUNCH_CONTROL
static motorMixer_t launchControlMixer[MAX_SUPPORTED_MOTORS];
#endif

static FAST_RAM_ZERO_INIT int throttleAngleCorrection;


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

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerBicopter[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};
#else
#define mixerBicopter NULL
#endif

static const motorMixer_t mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};


#if (MAX_SUPPORTED_MOTORS >= 6)
static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },     // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },     // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },     // LEFT
};

static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
    { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
    { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
};
static const motorMixer_t mixerY6[] = {
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};
#else
#define mixerHex6H NULL
#define mixerHex6P NULL
#define mixerY6 NULL
#endif // USE_UNCOMMON_MIXERS
#else
#define mixerHex6X NULL
#endif // MAX_SUPPORTED_MOTORS >= 6

#if defined(USE_UNCOMMON_MIXERS) && (MAX_SUPPORTED_MOTORS >= 8)
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
#else
#define mixerOctoX8 NULL
#define mixerOctoFlatP NULL
#define mixerOctoFlatX NULL
#endif

static const motorMixer_t mixerVtail4[] = {
    { 1.0f,  -0.58f,  0.58f, 1.0f },        // REAR_R
    { 1.0f,  -0.46f, -0.39f, -0.5f },       // FRONT_R
    { 1.0f,  0.58f,  0.58f, -1.0f },        // REAR_L
    { 1.0f,  0.46f, -0.39f, 0.5f },         // FRONT_L
};

static const motorMixer_t mixerAtail4[] = {
    { 1.0f, -0.58f,  0.58f, -1.0f },          // REAR_R
    { 1.0f, -0.46f, -0.39f,  0.5f },          // FRONT_R
    { 1.0f,  0.58f,  0.58f,  1.0f },          // REAR_L
    { 1.0f,  0.46f, -0.39f, -0.5f },          // FRONT_L
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerDualcopter[] = {
    { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
    { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
};
#else
#define mixerDualcopter NULL
#endif

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
// Some of these entries are bogus when servos (USE_SERVOS) are not configured,
// but left untouched to keep ordinals synced with mixerMode_e (and configurator).
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
    { 1, true,  mixerSingleProp },     // * MIXER_HELI_120_CCPM
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
#endif // !USE_QUAD_MIXER_ONLY

FAST_RAM_ZERO_INIT float motorOutputHigh, motorOutputLow;

static FAST_RAM_ZERO_INIT float disarmMotorOutput, deadbandMotor3dHigh, deadbandMotor3dLow;
static FAST_RAM_ZERO_INIT float rcCommandThrottleRange;

uint8_t getMotorCount(void)
{
    return motorCount;
}

float getMotorMixRange(void)
{
    return motorMixRange;
}

bool areMotorsRunning(void)
{
    bool motorsRunning = false;
    if (ARMING_FLAG(ARMED)) {
        motorsRunning = true;
    } else {
        for (int i = 0; i < motorCount; i++) {
            if (motor_disarmed[i] != disarmMotorOutput) {
                motorsRunning = true;

                break;
            }
        }
    }

    return motorsRunning;
}

#ifdef USE_SERVOS
bool mixerIsTricopter(void)
{
    return (currentMixerMode == MIXER_TRI || currentMixerMode == MIXER_CUSTOM_TRI);
}
#endif

// All PWM motor scaling is done to standard PWM range of 1000-2000 for easier tick conversion with legacy code / configurator
// DSHOT scaling is done to the actual dshot range
void initEscEndpoints(void)
{
    float motorOutputLimit = 1.0f;
    if (currentPidProfile->motor_output_limit < 100) {
        motorOutputLimit = currentPidProfile->motor_output_limit / 100.0f;
    }

    // Can't use 'isMotorProtocolDshot()' here since motors haven't been initialised yet
    switch (motorConfig()->dev.motorPwmProtocol) {
#ifdef USE_DSHOT
    case PWM_TYPE_PROSHOT1000:
    case PWM_TYPE_DSHOT1200:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT150:
        {
            float outputLimitOffset = (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) * (1 - motorOutputLimit);
            disarmMotorOutput = DSHOT_CMD_MOTOR_STOP;
            if (featureIsEnabled(FEATURE_3D)) {
                motorOutputLow = DSHOT_MIN_THROTTLE + ((DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - DSHOT_MIN_THROTTLE) / 100.0f) * CONVERT_PARAMETER_TO_PERCENT(motorConfig()->digitalIdleOffsetValue);
                motorOutputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset / 2;
                deadbandMotor3dHigh = DSHOT_3D_FORWARD_MIN_THROTTLE + ((DSHOT_MAX_THROTTLE - DSHOT_3D_FORWARD_MIN_THROTTLE) / 100.0f) * CONVERT_PARAMETER_TO_PERCENT(motorConfig()->digitalIdleOffsetValue);
                deadbandMotor3dLow = DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - outputLimitOffset / 2;
            } else {
                motorOutputLow = DSHOT_MIN_THROTTLE + ((DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) / 100.0f) * CONVERT_PARAMETER_TO_PERCENT(motorConfig()->digitalIdleOffsetValue);
                motorOutputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset;
            }
        }

        break;
#endif
    default:
        if (featureIsEnabled(FEATURE_3D)) {
            float outputLimitOffset = (flight3DConfig()->limit3d_high - flight3DConfig()->limit3d_low) * (1 - motorOutputLimit) / 2;
            disarmMotorOutput = flight3DConfig()->neutral3d;
            motorOutputLow = flight3DConfig()->limit3d_low + outputLimitOffset;
            motorOutputHigh = flight3DConfig()->limit3d_high - outputLimitOffset;
            deadbandMotor3dHigh = flight3DConfig()->deadband3d_high;
            deadbandMotor3dLow = flight3DConfig()->deadband3d_low;
        } else {
            disarmMotorOutput = motorConfig()->mincommand;
            motorOutputLow = motorConfig()->minthrottle;
            motorOutputHigh = motorConfig()->maxthrottle - ((motorConfig()->maxthrottle - motorConfig()->minthrottle) * (1 - motorOutputLimit));
        }
        break;
    }

    rcCommandThrottleRange = PWM_RANGE_MAX - PWM_RANGE_MIN;
}

void mixerInit(mixerMode_e mixerMode)
{
    currentMixerMode = mixerMode;

    initEscEndpoints();
#ifdef USE_SERVOS
    if (mixerIsTricopter()) {
        mixerTricopterInit();
    }
#endif
}

#ifdef USE_LAUNCH_CONTROL
// Create a custom mixer for launch control based on the current settings
// but disable the front motors. We don't care about roll or yaw because they
// are limited in the PID controller.
void loadLaunchControlMixer(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        launchControlMixer[i] = currentMixer[i];
        // limit the front motors to minimum output
        if (launchControlMixer[i].pitch < 0.0f) {
            launchControlMixer[i].pitch = 0.0f;
            launchControlMixer[i].throttle = 0.0f;
        }
    }
}
#endif

#ifndef USE_QUAD_MIXER_ONLY

void mixerConfigureOutput(void)
{
    motorCount = 0;

    if (currentMixerMode == MIXER_CUSTOM || currentMixerMode == MIXER_CUSTOM_TRI || currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
        // load custom mixer into currentMixer
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            // check if done
            if (customMotorMixer(i)->throttle == 0.0f) {
                break;
            }
            currentMixer[i] = *customMotorMixer(i);
            motorCount++;
        }
    } else {
        motorCount = mixers[currentMixerMode].motorCount;
        if (motorCount > MAX_SUPPORTED_MOTORS) {
            motorCount = MAX_SUPPORTED_MOTORS;
        }
        // copy motor-based mixers
        if (mixers[currentMixerMode].motor) {
            for (int i = 0; i < motorCount; i++)
                currentMixer[i] = mixers[currentMixerMode].motor[i];
        }
    }
#ifdef USE_LAUNCH_CONTROL
    loadLaunchControlMixer();
#endif
    mixerResetDisarmedMotors();
}

void mixerLoadMix(int index, motorMixer_t *customMixers)
{
    // we're 1-based
    index++;
    // clear existing
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        customMixers[i].throttle = 0.0f;
    }
    // do we have anything here to begin with?
    if (mixers[index].motor != NULL) {
        for (int i = 0; i < mixers[index].motorCount; i++) {
            customMixers[i] = mixers[index].motor[i];
        }
    }
}
#else
void mixerConfigureOutput(void)
{
    motorCount = QUAD_MOTOR_COUNT;
    for (int i = 0; i < motorCount; i++) {
        currentMixer[i] = mixerQuadX[i];
    }
#ifdef USE_LAUNCH_CONTROL
    loadLaunchControlMixer();
#endif
    mixerResetDisarmedMotors();
}
#endif // USE_QUAD_MIXER_ONLY

void mixerResetDisarmedMotors(void)
{
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = disarmMotorOutput;
    }
}

void writeMotors(void)
{
    if (pwmAreMotorsEnabled()) {
#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
        if (!pwmStartMotorUpdate(motorCount)) {
            return;
        }
#endif
        for (int i = 0; i < motorCount; i++) {
            pwmWriteMotor(i, motor[i]);
        }
        pwmCompleteMotorUpdate(motorCount);
    }
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < motorCount; i++) {
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

static FAST_RAM_ZERO_INIT float throttle = 0;
static FAST_RAM_ZERO_INIT float loggingThrottle = 0;
static FAST_RAM_ZERO_INIT float motorOutputMin;
static FAST_RAM_ZERO_INIT float motorRangeMin;
static FAST_RAM_ZERO_INIT float motorRangeMax;
static FAST_RAM_ZERO_INIT float motorOutputRange;
static FAST_RAM_ZERO_INIT int8_t motorOutputMixSign;

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    float currentThrottleInputRange = 0;

    if (featureIsEnabled(FEATURE_3D)) {
        uint16_t rcCommand3dDeadBandLow;
        uint16_t rcCommand3dDeadBandHigh;

        if (!ARMING_FLAG(ARMED)) {
            rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
        }

        if (IS_RC_MODE_ACTIVE(BOX3D) || flight3DConfig()->switched_mode3d) {
            // The min_check range is halved because the output throttle is scaled to 500us.
            // So by using half of min_check we maintain the same low-throttle deadband
            // stick travel as normal non-3D mode.
            const int mincheckOffset = (rxConfig()->mincheck - PWM_RANGE_MIN) / 2;
            rcCommand3dDeadBandLow = rxConfig()->midrc - mincheckOffset;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + mincheckOffset;
        } else {
            rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;
        }

        const float rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
        const float rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;

        if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow || isFlipOverAfterCrashActive()) {
            // INVERTED
            motorRangeMin = motorOutputLow;
            motorRangeMax = deadbandMotor3dLow;
            if (isMotorProtocolDshot()) {
                motorOutputMin = motorOutputLow;
                motorOutputRange = deadbandMotor3dLow - motorOutputLow;
            } else {
                motorOutputMin = deadbandMotor3dLow;
                motorOutputRange = motorOutputLow - deadbandMotor3dLow;
            }
            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;
            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand3dDeadBandLow - rcCommand[THROTTLE];
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
            // NORMAL
            motorRangeMin = deadbandMotor3dHigh;
            motorRangeMax = motorOutputHigh;
            motorOutputMin = deadbandMotor3dHigh;
            motorOutputRange = motorOutputHigh - deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] - rcCommand3dDeadBandHigh;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
                !flight3DConfigMutable()->switched_mode3d) ||
                isMotorsReversed()) {
            // INVERTED_TO_DEADBAND
            motorRangeMin = motorOutputLow;
            motorRangeMax = deadbandMotor3dLow;
            if (isMotorProtocolDshot()) {
                motorOutputMin = motorOutputLow;
                motorOutputRange = deadbandMotor3dLow - motorOutputLow;
            } else {
                motorOutputMin = deadbandMotor3dLow;
                motorOutputRange = motorOutputLow - deadbandMotor3dLow;
            }
            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;
            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else {
            // NORMAL_TO_DEADBAND
            motorRangeMin = deadbandMotor3dHigh;
            motorRangeMax = motorOutputHigh;
            motorOutputMin = deadbandMotor3dHigh;
            motorOutputRange = motorOutputHigh - deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        }
        if (currentTimeUs - reversalTimeUs < 250000) {
            // keep iterm zero for 250ms after motor reversal
            pidResetIterm();
        }
    } else {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
        currentThrottleInputRange = rcCommandThrottleRange;
        motorRangeMin = motorOutputLow;
        motorRangeMax = motorOutputHigh;
        motorOutputMin = motorOutputLow;
        motorOutputRange = motorOutputHigh - motorOutputLow;
        motorOutputMixSign = 1;
    }

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

#define CRASH_FLIP_DEADBAND 20
#define CRASH_FLIP_STICK_MINF 0.15f

static void applyFlipOverAfterCrashModeToMotors(void)
{
    if (ARMING_FLAG(ARMED)) {
        float stickDeflectionPitchAbs = getRcDeflectionAbs(FD_PITCH);
        float stickDeflectionRollAbs = getRcDeflectionAbs(FD_ROLL);
        float stickDeflectionYawAbs = getRcDeflectionAbs(FD_YAW);
        float signPitch = getRcDeflection(FD_PITCH) < 0 ? 1 : -1;
        float signRoll = getRcDeflection(FD_ROLL) < 0 ? 1 : -1;
        float signYaw = (getRcDeflection(FD_YAW) < 0 ? 1 : -1) * (mixerConfig()->yaw_motors_reversed ? 1 : -1);

        float stickDeflectionLength = sqrtf(stickDeflectionPitchAbs*stickDeflectionPitchAbs + stickDeflectionRollAbs*stickDeflectionRollAbs);

        if (stickDeflectionYawAbs > MAX(stickDeflectionPitchAbs, stickDeflectionRollAbs)) {
            // If yaw is the dominant, disable pitch and roll
            stickDeflectionLength = stickDeflectionYawAbs;
            signRoll = 0;
            signPitch = 0;
        } else {
            // If pitch/roll dominant, disable yaw
            signYaw = 0;
        }

        float cosPhi = (stickDeflectionPitchAbs + stickDeflectionRollAbs) / (sqrtf(2.0f) * stickDeflectionLength);
        const float cosThreshold = sqrtf(3.0f)/2.0f; // cos(PI/6.0f)

        if (cosPhi < cosThreshold) {
            // Enforce either roll or pitch exclusively, if not on diagonal
            if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
                signPitch = 0;
            } else {
                signRoll = 0;
            }
        }

        // Apply a reasonable amount of stick deadband
        const float flipStickRange = 1.0f - CRASH_FLIP_STICK_MINF;
        float flipPower = MAX(0.0f, stickDeflectionLength - CRASH_FLIP_STICK_MINF) / flipStickRange;

        for (int i = 0; i < motorCount; ++i) {
            float motorOutputNormalised =
                signPitch*currentMixer[i].pitch +
                signRoll*currentMixer[i].roll +
                signYaw*currentMixer[i].yaw;
                
            if (motorOutputNormalised < 0) {
                if (mixerConfig()->crashflip_motor_percent > 0) {
                    motorOutputNormalised = -motorOutputNormalised * (float)mixerConfig()->crashflip_motor_percent / 100.0f;
                } else {
                    motorOutputNormalised = 0;
                }
            } 
            motorOutputNormalised = MIN(1.0f, flipPower * motorOutputNormalised);
            float motorOutput = motorOutputMin + motorOutputNormalised * motorOutputRange;

            // Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
            motorOutput = (motorOutput < motorOutputMin + CRASH_FLIP_DEADBAND) ? disarmMotorOutput : (motorOutput - CRASH_FLIP_DEADBAND);

            motor[i] = motorOutput;
        }
    } else {
        // Disarmed mode
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (int i = 0; i < motorCount; i++) {
        float motorOutput = motorOutputMixSign * motorMix[i] + throttle * activeMixer[i].throttle;
#ifdef USE_THRUST_LINEARIZATION
        motorOutput = pidApplyThrustLinearization(motorOutput);
#endif
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;

#ifdef USE_SERVOS
        if (mixerIsTricopter()) {
            motorOutput += mixerTricopterMotorCorrection(i);
        }
#endif
        if (failsafeIsActive()) {
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
            motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrain(motorOutput, motorRangeMin, motorRangeMax);
        }
        motor[i] = motorOutput;
    }

    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

float applyThrottleLimit(float throttle)
{
    if (currentControlRateProfile->throttle_limit_percent < 100) {
        const float throttleLimitFactor = currentControlRateProfile->throttle_limit_percent / 100.0f;
        switch (currentControlRateProfile->throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }

    return throttle;
}

void applyMotorStop(void)
{
    for (int i = 0; i < motorCount; i++) {
        motor[i] = disarmMotorOutput;
    }
}

#ifdef USE_DYN_LPF
void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
            dynLpfGyroUpdate(dynLpfThrottle);
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs, uint8_t vbatPidCompensation)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    if (isFlipOverAfterCrashActive()) {
        applyFlipOverAfterCrashModeToMotors();

        return;
    }

    const bool launchControlActive = isLaunchControlActive();

    motorMixer_t * activeMixer = &currentMixer[0];
#ifdef USE_LAUNCH_CONTROL
    if (launchControlActive && (currentPidProfile->launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY)) {
        activeMixer = &launchControlMixer[0];
    }
#endif
    
    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinDetected = gyroYawSpinDetected();
    if (yawSpinDetected) {
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
#endif // USE_YAW_SPIN_RECOVERY

    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    // Calculate voltage compensation
    const float vbatCompensationFactor = vbatPidCompensation ? calculateVbatPidCompensation() : 1.0f;

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }

    const bool airmodeEnabled = airmodeIsEnabled() || launchControlActive;

#ifdef USE_YAW_SPIN_RECOVERY
    // 50% throttle provides the maximum authority for yaw recovery when airmode is not active.
    // When airmode is active the throttle setting doesn't impact recovery authority.
    if (yawSpinDetected && !airmodeEnabled) {
        throttle = 0.5f;   // 
    }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
    // While launch control is active keep the throttle at minimum.
    // Once the pilot triggers the launch throttle control will be reactivated.
    if (launchControlActive) {
        throttle = 0.0f;
    }
#endif

    // Find roll/pitch/yaw desired output
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < motorCount; i++) {

        float mix =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw;

        mix *= vbatCompensationFactor;  // Add voltage compensation

        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

    pidUpdateAntiGravityThrottleFilter(throttle);

#ifdef USE_DYN_LPF
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif

#ifdef USE_THRUST_LINEARIZATION
    // reestablish old throttle stick feel by counter compensating thrust linearization
    throttle = pidCompensateThrustLinearization(throttle);
#endif

#if defined(USE_THROTTLE_BOOST)
    if (throttleBoost > 0.0f) {
        const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    }
#endif

#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif

#ifdef USE_AIRMODE_LPF
    const float unadjustedThrottle = throttle;
    throttle += pidGetAirmodeThrottleOffset();
    float airmodeThrottleChange = 0;
#endif
    loggingThrottle = throttle;

    motorMixRange = motorMixMax - motorMixMin;
    if (motorMixRange > 1.0f) {
        for (int i = 0; i < motorCount; i++) {
            motorMix[i] /= motorMixRange;
        }
        // Get the maximum correction by setting offset to center when airmode enabled
        if (airmodeEnabled) {
            throttle = 0.5f;
        }
    } else {
        if (airmodeEnabled || throttle > 0.5f) {  // Only automatically adjust throttle when airmode enabled. Airmode logic is always active on high throttle
            throttle = constrainf(throttle, -motorMixMin, 1.0f - motorMixMax);
#ifdef USE_AIRMODE_LPF
            airmodeThrottleChange = constrainf(unadjustedThrottle, -motorMixMin, 1.0f - motorMixMax) - unadjustedThrottle;
#endif
        }
    }

#ifdef USE_AIRMODE_LPF
    pidUpdateAirmodeLpf(airmodeThrottleChange);
#endif

    if (featureIsEnabled(FEATURE_MOTOR_STOP)
        && ARMING_FLAG(ARMED)
        && !featureIsEnabled(FEATURE_3D)
        && !airmodeEnabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable motor_stop while GPS Rescue is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        // motor_stop handling
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix, activeMixer);
    }
}

float convertExternalToMotor(uint16_t externalValue)
{
    uint16_t motorValue;
    switch ((int)isMotorProtocolDshot()) {
#ifdef USE_DSHOT
    case true:
        externalValue = constrain(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX);

        if (featureIsEnabled(FEATURE_3D)) {
            if (externalValue == PWM_RANGE_MID) {
                motorValue = DSHOT_CMD_MOTOR_STOP;
            } else if (externalValue < PWM_RANGE_MID) {
                motorValue = scaleRange(externalValue, PWM_RANGE_MIN, PWM_RANGE_MID - 1, DSHOT_3D_FORWARD_MIN_THROTTLE - 1, DSHOT_MIN_THROTTLE);
            } else {
                motorValue = scaleRange(externalValue, PWM_RANGE_MID + 1, PWM_RANGE_MAX, DSHOT_3D_FORWARD_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
            }
        } else {
            motorValue = (externalValue == PWM_RANGE_MIN) ? DSHOT_CMD_MOTOR_STOP : scaleRange(externalValue, PWM_RANGE_MIN + 1, PWM_RANGE_MAX, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
        }

        break;
    case false:
#endif
    default:
        motorValue = externalValue;
        break;
    }

    return (float)motorValue;
}

uint16_t convertMotorToExternal(float motorValue)
{
    uint16_t externalValue;
    switch ((int)isMotorProtocolDshot()) {
#ifdef USE_DSHOT
    case true:
        if (featureIsEnabled(FEATURE_3D)) {
            if (motorValue == DSHOT_CMD_MOTOR_STOP || motorValue < DSHOT_MIN_THROTTLE) {
                externalValue = PWM_RANGE_MID;
            } else if (motorValue <= DSHOT_3D_FORWARD_MIN_THROTTLE - 1) {
                externalValue = scaleRange(motorValue, DSHOT_MIN_THROTTLE, DSHOT_3D_FORWARD_MIN_THROTTLE - 1, PWM_RANGE_MID - 1, PWM_RANGE_MIN);
            } else {
                externalValue = scaleRange(motorValue, DSHOT_3D_FORWARD_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MID + 1, PWM_RANGE_MAX);
            }
        } else {
            externalValue = (motorValue < DSHOT_MIN_THROTTLE) ? PWM_RANGE_MIN : scaleRange(motorValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIN + 1, PWM_RANGE_MAX);
        }
        break;
    case false:
#endif
    default:
        externalValue = motorValue;
        break;
    }

    return externalValue;
}

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}

float mixerGetLoggingThrottle(void)
{
    return loggingThrottle;
}

#ifdef USE_DSHOT_TELEMETRY
bool isDshotTelemetryActive(void)
{
    for (uint8_t i = 0; i < motorCount; i++) {
        if (!isDshotMotorTelemetryActive(i)) {
            return false;
        }
    }
    return true;
}
#endif
