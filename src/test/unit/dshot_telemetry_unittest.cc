/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>

extern "C" {
#include "platform.h"

#include "build/debug.h"

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"

#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Telemetry packets (12 bit, after GCR decode)
#define PACKET_ERPM        0x0100   // eRPM, type nibble 1
#define PACKET_EDT_ENABLE  0x0E00   // EDT enable acknowledge
#define PACKET_VOLTAGE     0x0440   // EDT voltage 64 * 0.25V

// Stubs

extern "C" {

static timeMs_t nowMs;
static uint8_t currentDshotCommand;

PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 0);

bool useDshotTelemetry;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

timeMs_t millis(void)
{
    return nowMs;
}

uint8_t dshotCommandGetCurrent(unsigned index)
{
    UNUSED(index);
    return currentDshotCommand;
}

bool featureIsEnabled(uint32_t mask)
{
    UNUSED(mask);
    return false;
}

}

// One receive round of the platform code, then the decode in the task
static void receiveRound(uint32_t rawValue)
{
    dshotTelemetryReceived(0, rawValue);
    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
    updateDshotTelemetry();
}

class DshotTelemetryTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&dshotTelemetryState, 0, sizeof(dshotTelemetryState));
        useDshotTelemetry = true;
        dshotMotorCount = 1;
        currentDshotCommand = 0;
        nowMs = 1000;
    }
};

TEST_F(DshotTelemetryTest, ValidPacketIsDecodedWithArrivalTime)
{
    receiveRound(PACKET_ERPM);

    const dshotTelemetryMotorState_t *state = &dshotTelemetryState.motorState[0];
    EXPECT_TRUE(isDshotMotorTelemetryActive(0));
    EXPECT_NE(0, state->telemetryData[DSHOT_TELEMETRY_TYPE_eRPM]);
    EXPECT_EQ(1000u, state->updateMs[DSHOT_TELEMETRY_TYPE_eRPM]);
    EXPECT_FALSE(state->rawValueNew);
}

TEST_F(DshotTelemetryTest, InvalidPacketIsCountedAndNotDecoded)
{
    receiveRound(DSHOT_TELEMETRY_INVALID);

    EXPECT_EQ(1u, dshotTelemetryState.invalidPacketCount);
    EXPECT_FALSE(isDshotMotorTelemetryActive(0));
}

// A failed reception must not decode the previous packet again
TEST_F(DshotTelemetryTest, FailedReceptionDoesNotRepeatPreviousPacket)
{
    receiveRound(PACKET_ERPM);

    nowMs += 100;
    receiveRound(DSHOT_TELEMETRY_INVALID);

    EXPECT_EQ(1000u, dshotTelemetryState.motorState[0].updateMs[DSHOT_TELEMETRY_TYPE_eRPM]);
}

// Regression: a failed reception after an EDT packet kept the EDT value fresh
TEST_F(DshotTelemetryTest, FailedReceptionAfterEdtPacketDoesNotRefreshIt)
{
    currentDshotCommand = DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE;
    receiveRound(PACKET_EDT_ENABLE);
    currentDshotCommand = 0;
    EXPECT_NE(0, dshotTelemetryState.motorState[0].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS));

    receiveRound(PACKET_VOLTAGE);
    const dshotTelemetryMotorState_t *state = &dshotTelemetryState.motorState[0];
    EXPECT_EQ(64, state->telemetryData[DSHOT_TELEMETRY_TYPE_VOLTAGE]);
    EXPECT_EQ(1000u, state->updateMs[DSHOT_TELEMETRY_TYPE_VOLTAGE]);

    for (int i = 0; i < 10; i++) {
        nowMs += 1000;
        receiveRound(DSHOT_TELEMETRY_INVALID);
    }

    EXPECT_EQ(1000u, state->updateMs[DSHOT_TELEMETRY_TYPE_VOLTAGE]);
}

TEST_F(DshotTelemetryTest, EdtEnableAcknowledgeIsNotDecodedAsValue)
{
    currentDshotCommand = DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE;
    receiveRound(PACKET_EDT_ENABLE);

    const dshotTelemetryMotorState_t *state = &dshotTelemetryState.motorState[0];
    EXPECT_EQ(1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS, state->telemetryTypes);
    EXPECT_FALSE(state->rawValueNew);
}
