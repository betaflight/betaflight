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

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"

#include "fc/controlrate_profile.h"
#include "fc/runtime_config.h"

#include "flight/mixer_tricopter.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "mixer_init.h"

PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixerMode = DEFAULT_MIXER,
    .yaw_motors_reversed = false,
    .crashflip_motor_percent = 0,
    .crashflip_expo = 35
);

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);

mixerMode_e currentMixerMode;

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

FAST_DATA_ZERO_INIT mixerRuntime_t mixerRuntime;

uint8_t getMotorCount(void)
{
    return mixerRuntime.motorCount;
}

bool areMotorsRunning(void)
{
    bool motorsRunning = false;
    if (ARMING_FLAG(ARMED)) {
        motorsRunning = true;
    } else {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            if (motor_disarmed[i] != mixerRuntime.disarmMotorOutput) {
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

    motorInitEndpoints(motorConfig(), motorOutputLimit, &mixerRuntime.motorOutputLow, &mixerRuntime.motorOutputHigh, &mixerRuntime.disarmMotorOutput, &mixerRuntime.deadbandMotor3dHigh, &mixerRuntime.deadbandMotor3dLow);
    if (!mixerRuntime.feature3dEnabled && currentPidProfile->idle_min_rpm) {
        mixerRuntime.motorOutputLow = DSHOT_MIN_THROTTLE;
    }
}

// Initialize pidProfile related mixer settings
void mixerInitProfile(void)
{
#ifdef USE_DYN_IDLE
    mixerRuntime.idleMinMotorRps = currentPidProfile->idle_min_rpm * 100.0f / 60.0f;
    mixerRuntime.idleMaxIncrease = currentPidProfile->idle_max_increase * 0.001f;
    mixerRuntime.idleP = currentPidProfile->idle_p * 0.0001f;
    mixerRuntime.oldMinRps = 0;
#endif

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    mixerRuntime.vbatSagCompensationFactor = 0.0f;
    if (currentPidProfile->vbat_sag_compensation > 0) {
        //TODO: Make this voltage user configurable
        mixerRuntime.vbatFull = CELL_VOLTAGE_FULL_CV;
        mixerRuntime.vbatRangeToCompensate = mixerRuntime.vbatFull - batteryConfig()->vbatwarningcellvoltage;
        if (mixerRuntime.vbatRangeToCompensate > 0) {
            mixerRuntime.vbatSagCompensationFactor = ((float)currentPidProfile->vbat_sag_compensation) / 100.0f;
        }
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
        mixerRuntime.launchControlMixer[i] = mixerRuntime.currentMixer[i];
        // limit the front motors to minimum output
        if (mixerRuntime.launchControlMixer[i].pitch < 0.0f) {
            mixerRuntime.launchControlMixer[i].pitch = 0.0f;
            mixerRuntime.launchControlMixer[i].throttle = 0.0f;
        }
    }
}
#endif

#ifndef USE_QUAD_MIXER_ONLY

static void mixerConfigureOutput(void)
{
    mixerRuntime.motorCount = 0;

    if (currentMixerMode == MIXER_CUSTOM || currentMixerMode == MIXER_CUSTOM_TRI || currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
        // load custom mixer into currentMixer
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            // check if done
            if (customMotorMixer(i)->throttle == 0.0f) {
                break;
            }
            mixerRuntime.currentMixer[i] = *customMotorMixer(i);
            mixerRuntime.motorCount++;
        }
    } else {
        mixerRuntime.motorCount = mixers[currentMixerMode].motorCount;
        if (mixerRuntime.motorCount > MAX_SUPPORTED_MOTORS) {
            mixerRuntime.motorCount = MAX_SUPPORTED_MOTORS;
        }
        // copy motor-based mixers
        if (mixers[currentMixerMode].motor) {
            for (int i = 0; i < mixerRuntime.motorCount; i++)
                mixerRuntime.currentMixer[i] = mixers[currentMixerMode].motor[i];
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
static void mixerConfigureOutput(void)
{
    mixerRuntime.motorCount = QUAD_MOTOR_COUNT;
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        mixerRuntime.currentMixer[i] = mixerQuadX[i];
    }
#ifdef USE_LAUNCH_CONTROL
    loadLaunchControlMixer();
#endif
    mixerResetDisarmedMotors();
}
#endif // USE_QUAD_MIXER_ONLY

void mixerInit(mixerMode_e mixerMode)
{
    currentMixerMode = mixerMode;

    mixerRuntime.feature3dEnabled = featureIsEnabled(FEATURE_3D);

    initEscEndpoints();
#ifdef USE_SERVOS
    if (mixerIsTricopter()) {
        mixerTricopterInit();
    }
#endif

#ifdef USE_DYN_IDLE
    mixerRuntime.idleThrottleOffset = getDigitalIdleOffset(motorConfig()) * PWM_RANGE;
#endif

    mixerConfigureOutput();
}

void mixerResetDisarmedMotors(void)
{
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = mixerRuntime.disarmMotorOutput;
    }
}

mixerMode_e getMixerMode(void)
{
    return currentMixerMode;
}

bool mixerModeIsFixedWing(mixerMode_e mixerMode)
{
    switch (mixerMode) {
    case MIXER_FLYING_WING:
    case MIXER_AIRPLANE:
    case MIXER_CUSTOM_AIRPLANE:
        return true;

        break;
    default:
        return false;

        break;
    }
}

bool isFixedWing(void)
{
    return mixerModeIsFixedWing(currentMixerMode);
}

float getMotorOutputLow(void)
{
    return mixerRuntime.motorOutputLow;
}

float getMotorOutputHigh(void)
{
    return mixerRuntime.motorOutputHigh;
}
