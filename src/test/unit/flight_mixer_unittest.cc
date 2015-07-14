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
    void forwardAuxChannelsToServos(uint8_t firstServoIndex);

    void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers, servoMixer_t *initialCustomServoMixers);
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

int updatedServoCount;
int updatedMotorCount;

class ChannelForwardingTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        memset(&servos, 0, sizeof(servos));
    }
};


TEST_F(ChannelForwardingTest, TestForwardAuxChannelsToServosWithNoServos)
{
    // given
    servoCount = 0;

    rcData[AUX1] = TEST_RC_MID;
    rcData[AUX2] = TEST_RC_MID;
    rcData[AUX3] = TEST_RC_MID;
    rcData[AUX4] = TEST_RC_MID;

    // when
    forwardAuxChannelsToServos(MAX_SUPPORTED_SERVOS);

    // then
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        EXPECT_EQ(0, servos[i].value);
    }
}

TEST_F(ChannelForwardingTest, TestForwardAuxChannelsToServosWithMaxServos)
{
    // given
    servoCount = MAX_SUPPORTED_SERVOS;

    rcData[AUX1] = 1000;
    rcData[AUX2] = 1250;
    rcData[AUX3] = 1750;
    rcData[AUX4] = 2000;

    // when
    forwardAuxChannelsToServos(MAX_SUPPORTED_SERVOS);

    // then
    uint8_t i;
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        EXPECT_EQ(0, servos[i].value);
    }
}

TEST_F(ChannelForwardingTest, TestForwardAuxChannelsToServosWithLessRemainingServosThanAuxChannelsToForward)
{
    // given
    servoCount = MAX_SUPPORTED_SERVOS - 2;

    rcData[AUX1] = 1000;
    rcData[AUX2] = 1250;
    rcData[AUX3] = 1750;
    rcData[AUX4] = 2000;

    // when
    forwardAuxChannelsToServos(MAX_SUPPORTED_SERVOS - 2);

    // then
    uint8_t i;
    for (i = 0; i < MAX_SUPPORTED_SERVOS - 2; i++) {
        EXPECT_EQ(0, servos[i].value);
    }

    // -1 for zero based offset
    EXPECT_EQ(1000, servos[MAX_SUPPORTED_SERVOS - 1 - 1].value);
    EXPECT_EQ(1250, servos[MAX_SUPPORTED_SERVOS - 0 - 1].value);
}

class BasicMixerIntegrationTest : public ::testing::Test {
protected:
    mixerConfig_t mixerConfig;
    rxConfig_t rxConfig;
    escAndServoConfig_t escAndServoConfig;
    servoParam_t servoConf[MAX_SUPPORTED_SERVOS];
    gimbalConfig_t gimbalConfig = {
        .mode = GIMBAL_MODE_NORMAL
    };

    motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];
    servoMixer_t customServoMixer[MAX_SUPPORTED_SERVOS];

    virtual void SetUp() {
        updatedServoCount = 0;
        updatedMotorCount = 0;

        memset(&mixerConfig, 0, sizeof(mixerConfig));
        memset(&rxConfig, 0, sizeof(rxConfig));
        memset(&escAndServoConfig, 0, sizeof(escAndServoConfig));
        memset(&servoConf, 0, sizeof(servoConf));

        memset(rcData, 0, sizeof(rcData));
        memset(rcCommand, 0, sizeof(rcCommand));
        memset(axisPID, 0, sizeof(axisPID));

        memset(&customMotorMixer, 0, sizeof(customMotorMixer));
    }

    virtual void withDefaultEscAndServoConfiguration(void) {
        escAndServoConfig.mincommand = TEST_MIN_COMMAND;
    }

    virtual void withDefaultRxConfig(void) {
        rxConfig.midrc = 1500;
    }

    virtual void configureMixer(void) {
        mixerUseConfigs(
            servoConf,
            &gimbalConfig,
            NULL,
            &escAndServoConfig,
            &mixerConfig,
            NULL,
            &rxConfig
        );
    }
};

TEST_F(BasicMixerIntegrationTest, TestTricopterServo)
{
    // given
    rxConfig.midrc = 1500;

    mixerConfig.tri_unarmed_servo = 1;

    withDefaultEscAndServoConfiguration();
    withDefaultRxConfig();

    servoConf[5].min = DEFAULT_SERVO_MIN;
    servoConf[5].max = DEFAULT_SERVO_MAX;
    servoConf[5].middle = DEFAULT_SERVO_MIDDLE;
    servoConf[5].rate = 100;
    servoConf[5].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;

    configureMixer();

    mixerInit(MIXER_TRI, customMotorMixer, customServoMixer);

    // and
    pwmOutputConfiguration_t pwmOutputConfiguration = {
            .servoCount = 1,
            .motorCount = 3
    };

    mixerUsePWMOutputConfiguration(&pwmOutputConfiguration);

    // and
    axisPID[YAW] = 0;

    // when
    mixTable();
    writeServos();

    // then
    EXPECT_EQ(1, updatedServoCount);
    EXPECT_EQ(TEST_SERVO_MID, servos[0].value);
}

TEST_F(BasicMixerIntegrationTest, TestQuadMotors)
{
    // given
    withDefaultEscAndServoConfiguration();

    configureMixer();

    mixerInit(MIXER_QUADX, customMotorMixer, customServoMixer);

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
    EXPECT_EQ(4, updatedMotorCount);

    EXPECT_EQ(TEST_MIN_COMMAND, motors[0].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[1].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[2].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[3].value);
}


class CustomMixerIntegrationTest : public BasicMixerIntegrationTest {
protected:

    virtual void SetUp() {

        BasicMixerIntegrationTest::SetUp();

        memset(&servoConf, 0, sizeof(servoConf));
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servoConf[i].min = DEFAULT_SERVO_MIN;
            servoConf[i].max = DEFAULT_SERVO_MAX;
            servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
            servoConf[i].rate = 100;
            servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
        }

        withDefaultEscAndServoConfiguration();
        withDefaultRxConfig();

        configureMixer();

        memset(&customMotorMixer, 0, sizeof(customMotorMixer));
        memset(&customServoMixer, 0, sizeof(customServoMixer));
    }
};


TEST_F(CustomMixerIntegrationTest, TestCustomMixer)
{
    // given
    enum {
        EXPECTED_SERVOS_TO_MIX_COUNT = 6,
        EXPECTED_MOTORS_TO_MIX_COUNT = 2
    };

    servoMixer_t testServoMixer[EXPECTED_SERVOS_TO_MIX_COUNT] = {
        { SERVO_FLAPS, INPUT_RC_AUX1,  100, 0, 0, 100, 0 },
        { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
        { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
        { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
        { SERVO_ELEVATOR, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
        { SERVO_THROTTLE, INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
    };
    memcpy(customServoMixer, testServoMixer, sizeof(testServoMixer));

    static const motorMixer_t testMotorMixer[EXPECTED_MOTORS_TO_MIX_COUNT] = {
        { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
        { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
    };
    memcpy(customMotorMixer, testMotorMixer, sizeof(testMotorMixer));

    mixerInit(MIXER_CUSTOM_AIRPLANE, customMotorMixer, customServoMixer);

    pwmOutputConfiguration_t pwmOutputConfiguration = {
            .servoCount = 6,
            .motorCount = 2
    };

    mixerUsePWMOutputConfiguration(&pwmOutputConfiguration);

    // and
    rcCommand[THROTTLE] = 1000;

    // and
    rcData[AUX1] = 2000;

    // and
    memset(axisPID, 0, sizeof(axisPID));
    axisPID[YAW] = 0;


    // when
    mixTable();
    writeMotors();
    writeServos();

    // then
    EXPECT_EQ(EXPECTED_MOTORS_TO_MIX_COUNT, updatedMotorCount);

    EXPECT_EQ(TEST_MIN_COMMAND, motors[0].value);
    EXPECT_EQ(TEST_MIN_COMMAND, motors[1].value);

    EXPECT_EQ(EXPECTED_SERVOS_TO_MIX_COUNT, updatedServoCount);

    EXPECT_EQ(2000, servos[0].value); // Flaps
    EXPECT_EQ(TEST_SERVO_MID, servos[1].value);
    EXPECT_EQ(TEST_SERVO_MID, servos[2].value);
    EXPECT_EQ(TEST_SERVO_MID, servos[3].value);
    EXPECT_EQ(TEST_SERVO_MID, servos[4].value);
    EXPECT_EQ(1000, servos[5].value); // Throttle

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
    updatedMotorCount++;
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
    // FIXME logic in test, mimic's production code.
    // Perhaps the solution is to remove the logic from the production code version and assume that
    // anything calling calling pwmWriteServo always uses a valid index?
    // See MAX_SERVOS in pwm_output (driver) and MAX_SUPPORTED_SERVOS (flight)
    if (index < MAX_SERVOS) {
        servos[index].value = value;
    }
    updatedServoCount++;
}

bool failsafeIsActive(void) {
    return false;
}

}
