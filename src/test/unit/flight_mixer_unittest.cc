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
    #include "platform.h"

    #include "common/axis.h"
    #include "common/maths.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "rx/rx.h"
    #include "flight/pid.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/lowpass.h"

    #include "io/rc_controls.h"

    extern uint8_t servoCount;
    void forwardAuxChannelsToServos(void);

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

typedef struct motor_s {
    uint16_t value;
} motor_t;

typedef struct servo_s {
    uint16_t value;
} servo_t;

motor_t motors[MAX_SUPPORTED_MOTORS];
servo_t servos[MAX_SUPPORTED_SERVOS];

uint8_t lastOneShotUpdateMotorCount;


TEST(FlightMixerTest, TestForwardAuxChannelsToServosWithNoServos)
{
    // given
    memset(&servos, 0, sizeof(servos));
    servoCount = 0;

    rcData[AUX1] = 1500;
    rcData[AUX2] = 1500;
    rcData[AUX3] = 1500;
    rcData[AUX4] = 1500;

    // when
    forwardAuxChannelsToServos();

    // then
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        EXPECT_EQ(servos[i].value, 0);
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
    EXPECT_EQ(servos[MAX_SUPPORTED_SERVOS - 3 - 1].value, 1000);
    EXPECT_EQ(servos[MAX_SUPPORTED_SERVOS - 2 - 1].value, 1250);
    EXPECT_EQ(servos[MAX_SUPPORTED_SERVOS - 1 - 1].value, 1750);
    EXPECT_EQ(servos[MAX_SUPPORTED_SERVOS - 0 - 1].value, 2000);
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
    EXPECT_EQ(servos[0].value, 1000);
    EXPECT_EQ(servos[1].value, 1250);
}

// STUBS

extern "C" {
rollAndPitchInclination_t inclination;
rxRuntimeConfig_t rxRuntimeConfig;

int16_t axisPID[XYZ_AXIS_COUNT];
int16_t rcCommand[4];
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

uint32_t rcModeActivationMask;
int16_t debug[4];

uint8_t stateFlags;
uint16_t flightModeFlags;
uint8_t armingFlags;

void delay(uint32_t) {}

bool feature(uint32_t) {
    return true;
}

int32_t lowpassFixed(lowpass_t *, int32_t, int16_t) {
    return 0;
}

void pwmWriteMotor(uint8_t index, uint16_t value) {
    motors[index].value = value;
}

void pwmCompleteOneshotMotorUpdate(uint8_t motorCount) {
    lastOneShotUpdateMotorCount = motorCount;
}

void pwmWriteServo(uint8_t index, uint16_t value) {
    servos[index].value = value;
}

}
