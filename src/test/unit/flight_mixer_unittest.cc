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

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "debug.h"

    #include "platform.h"

    #include "common/axis.h"
    #include "common/maths.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/pwm_mapping.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "rx/rx.h"
    #include "flight/pid.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/lowpass.h"

    #include "io/escservo.h"
    #include "io/gimbal.h"
    #include "io/rc_controls.h"

    extern uint8_t servoCount;
    void forwardAuxChannelsToServos(void);

    void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers);
    void mixerUsePWMOutputConfiguration(pwmOutputConfiguration_t *pwmOutputConfiguration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// input
#define TEST_RC_MID 1500

// output
#define TEST_MIN_COMMAND 1000
#define TEST_SERVO_MID 1500

typedef struct motor_s {
    uint16_t value;
} motor_t;

typedef struct servo_s {
    uint16_t value;
} servo_t;

motor_t motors[MAX_SUPPORTED_MOTORS];
servo_t servos[MAX_SUPPORTED_SERVOS];

uint8_t lastOneShotUpdateMotorCount;

uint32_t testFeatureMask = 0;


TEST(FlightMixerTest, TestForwardAuxChannelsToServosWithNoServos)
{
    // given
    memset(&servos, 0, sizeof(servos));
    servoCount = 0;

    rcData[AUX1] = TEST_RC_MID;
    rcData[AUX2] = TEST_RC_MID;
    rcData[AUX3] = TEST_RC_MID;
    rcData[AUX4] = TEST_RC_MID;

    // when
    forwardAuxChannelsToServos();

    // then
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        EXPECT_EQ(0, servos[i].value);
    }
}

TEST(FlightMixerTest, TestForwardAuxChannelsToServosWithMaxServos)
{
    // given
    memset(&servos, 0, sizeof(servos));
    servoCount = MAX_SUPPORTED_SERVOS;

    rcData[AUX1] = 1000;
    rcData[AUX2] = 1250;
    rcData[AUX3] = 1750;
    rcData[AUX4] = 2000;

    // when
    forwardAuxChannelsToServos();

    // then
    uint8_t i;
    for (i = 0; i < MAX_SUPPORTED_SERVOS - 4; i++) {
        EXPECT_EQ(servos[i].value, 0);
    }

    // -1 for zero based offset
    EXPECT_EQ(1000, servos[MAX_SUPPORTED_SERVOS - 3 - 1].value);
    EXPECT_EQ(1250, servos[MAX_SUPPORTED_SERVOS - 2 - 1].value);
    EXPECT_EQ(1750, servos[MAX_SUPPORTED_SERVOS - 1 - 1].value);
    EXPECT_EQ(2000, servos[MAX_SUPPORTED_SERVOS - 0 - 1].value);
}

TEST(FlightMixerTest, TestForwardAuxChannelsToServosWithLessServosThanAuxChannelsToForward)
{
    // given
    memset(&servos, 0, sizeof(servos));
    servoCount = 2;

    rcData[AUX1] = 1000;
    rcData[AUX2] = 1250;
    rcData[AUX3] = 1750;
    rcData[AUX4] = 2000;

    // when
    forwardAuxChannelsToServos();

    // then
    uint8_t i;
    for (i = 2; i < MAX_SUPPORTED_SERVOS; i++) {
        EXPECT_EQ(servos[i].value, 0);
    }

    // -1 for zero based offset
    EXPECT_EQ(1000, servos[0].value);
    EXPECT_EQ(1250, servos[1].value);
}

TEST(FlightMixerTest, TestTricopterServo)
{
    // given
    mixerConfig_t mixerConfig;
    memset(&mixerConfig, 0, sizeof(mixerConfig));

    mixerConfig.tri_unarmed_servo = 1;

    escAndServoConfig_t escAndServoConfig;
    memset(&escAndServoConfig, 0, sizeof(escAndServoConfig));
    escAndServoConfig.mincommand = TEST_MIN_COMMAND;

    servoParam_t servoConf[MAX_SUPPORTED_SERVOS];
    memset(&servoConf, 0, sizeof(servoConf));
    servoConf[5].min = DEFAULT_SERVO_MIN;
    servoConf[5].max = DEFAULT_SERVO_MAX;
    servoConf[5].middle = DEFAULT_SERVO_MIDDLE;
    servoConf[5].rate = 100;
    servoConf[5].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;

    gimbalConfig_t gimbalConfig = {
        .gimbal_flags = 0
    };

    mixerUseConfigs(
        servoConf,
        &gimbalConfig,
        NULL,
        &escAndServoConfig,
        &mixerConfig,
        NULL,
        NULL
    );

    motorMixer_t customMixer[MAX_SUPPORTED_MOTORS];
    memset(&customMixer, 0, sizeof(customMixer));

    mixerInit(MIXER_TRI, customMixer);

    // and
    pwmOutputConfiguration_t pwmOutputConfiguration = {
            .servoCount = 1,
            .motorCount = 3
    };

    mixerUsePWMOutputConfiguration(&pwmOutputConfiguration);

    // and
    memset(rcCommand, 0, sizeof(rcCommand));

    // and
    memset(axisPID, 0, sizeof(axisPID));
    axisPID[YAW] = 0;


    // when
    mixTable();
    writeServos();

    // then
    EXPECT_EQ(TEST_SERVO_MID, servos[0].value);
}

TEST(FlightMixerTest, TestQuadMotors)
{
    // given
    mixerConfig_t mixerConfig;
    memset(&mixerConfig, 0, sizeof(mixerConfig));

    //servoParam_t servoConf[MAX_SUPPORTED_SERVOS];
    //memset(&servoConf, 0, sizeof(servoConf));

    escAndServoConfig_t escAndServoConfig;
    memset(&escAndServoConfig, 0, sizeof(escAndServoConfig));
    escAndServoConfig.mincommand = TEST_MIN_COMMAND;

    gimbalConfig_t gimbalConfig = {
        .gimbal_flags = 0
    };

    mixerUseConfigs(
        NULL,// servoConf,
        &gimbalConfig,
        NULL,
        &escAndServoConfig,
        &mixerConfig,
        NULL,
        NULL
    );

    motorMixer_t customMixer[MAX_SUPPORTED_MOTORS];
    memset(&customMixer, 0, sizeof(customMixer));

    mixerInit(MIXER_QUADX, customMixer);

    // and
    pwmOutputConfiguration_t pwmOutputConfiguration = {
            .servoCount = 0,
            .motorCount = 4
    };

    mixerUsePWMOutputConfiguration(&pwmOutputConfiguration);

    // and
    memset(rcCommand, 0, sizeof(rcCommand));

    // and
    memset(axisPID, 0, sizeof(axisPID));
    axisPID[YAW] = 0;


    // when
    mixTable();
    writeMotors();

    // then
    EXPECT_EQ(TEST_MIN_COMMAND, motors[0].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[1].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[2].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[3].value);
}

// STUBS

extern "C" {
rollAndPitchInclination_t inclination;
rxRuntimeConfig_t rxRuntimeConfig;

int16_t axisPID[XYZ_AXIS_COUNT];
int16_t rcCommand[4];
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

uint32_t rcModeActivationMask;
int16_t debug[DEBUG16_VALUE_COUNT];

uint8_t stateFlags;
uint16_t flightModeFlags;
uint8_t armingFlags;

void delay(uint32_t) {}

bool feature(uint32_t mask) {
    return (mask & testFeatureMask);
}

int32_t lowpassFixed(lowpass_t *, int32_t, int16_t) {
    return 0;
}

void pwmWriteMotor(uint8_t index, uint16_t value) {
    motors[index].value = value;
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    uint8_t index;

    for(index = 0; index < motorCount; index++){
        motors[index].value = 0;
    }
}

void pwmCompleteOneshotMotorUpdate(uint8_t motorCount) {
    lastOneShotUpdateMotorCount = motorCount;
}

void pwmWriteServo(uint8_t index, uint16_t value) {
    servos[index].value = value;
}

bool failsafeIsActive(void) {
    return false;
}

}
