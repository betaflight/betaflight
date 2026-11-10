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

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

extern "C" {
    #include "platform.h"
    #include "canard.h"
    #include "common/maths.h"
    #include "drivers/motor.h"
    #include "flight/mixer.h"
    #include "io/dronecan/dronecan.h"
    #include "io/dronecan/dronecan_esc.h"
    #include "io/dronecan/dronecan_msg.h"
    #include "pg/dronecan.h"
    #include "pg/motor.h"
    #include "sensors/esc_sensor.h"

    dronecanConfig_t dronecanConfig_System;
    motorConfig_t motorConfig_System;
    static dronecanRxHandler statusHandler;
    static timeUs_t mockMicros;
    static escSensorData_t samples[MAX_SUPPORTED_MOTORS];
    static unsigned publications;
    static uint8_t publishedMotor;
    static unsigned ages[MAX_SUPPORTED_MOTORS];

    bool dronecanRegisterSubscriber(const dronecanSubscriber_t *subscriber)
    {
        if (subscriber->dataTypeId == UAVCAN_ESC_STATUS_ID) {
            statusHandler = subscriber->handler;
        }
        return true;
    }
    timeUs_t micros(void)
    {
        return mockMicros;
    }
    bool isMotorProtocolDronecan(void)
    {
        return false;
    }
    bool dronecanIsInitialised(void)
    {
        return false;
    }
    CanardInstance *dronecanGetInstance(void)
    {
        return nullptr;
    }
    void dronecanFlushTx(void)
    {
    }
    uint8_t getMotorCount(void)
    {
        return 4;
    }
    void dronecanNodesNoteSensor(uint8_t, uint8_t)
    {
    }
    void escSensorExternalInit(void)
    {
        memset(samples, 0, sizeof(samples));
        memset(ages, 0, sizeof(ages));
        publications = 0;
    }
    void escSensorSetExternal(uint8_t index, const escSensorData_t *data)
    {
        samples[index] = *data;
        publishedMotor = index;
        publications++;
    }
    void escSensorExternalAge(uint8_t index)
    {
        ages[index]++;
    }
    float scaleRangef(float x, float from, float to, float outFrom, float outTo)
    {
        return outFrom + (x - from) * (outTo - outFrom) / (to - from);
    }
}

#include "gtest/gtest.h"

class DronecanEscTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&dronecanConfig_System, 0, sizeof(dronecanConfig_System));
        memset(&motorConfig_System, 0, sizeof(motorConfig_System));
        dronecanConfig_System.esc_rate_hz = 100;
        motorConfig_System.motorPoleCount = 14;
        mockMicros = 0;
        dronecanEscInit();
        ASSERT_NE(nullptr, statusHandler);
    }

    static void feed(int32_t rpm, float voltage = 24.5f, float current = 2.5f,
        float temperature = 300.0f, uint8_t index = 0, uint16_t length = 14)
    {
        uint8_t payload[14] = {};
        const uint16_t v = canardConvertNativeFloatToFloat16(voltage);
        const uint16_t a = canardConvertNativeFloatToFloat16(current);
        const uint16_t t = canardConvertNativeFloatToFloat16(temperature);
        canardEncodeScalar(payload, ESC_STATUS_OFFSET_VOLTAGE, 16, &v);
        canardEncodeScalar(payload, ESC_STATUS_OFFSET_CURRENT, 16, &a);
        canardEncodeScalar(payload, ESC_STATUS_OFFSET_TEMPERATURE, 16, &t);
        canardEncodeScalar(payload, ESC_STATUS_OFFSET_RPM, 18, &rpm);
        canardEncodeScalar(payload, ESC_STATUS_OFFSET_ESC_INDEX, 5, &index);
        CanardRxTransfer transfer = {};
        transfer.transfer_type = CanardTransferTypeBroadcast;
        transfer.data_type_id = UAVCAN_ESC_STATUS_ID;
        transfer.source_node_id = 42;
        transfer.payload_head = payload;
        transfer.payload_len = length;
        statusHandler(nullptr, &transfer);
    }
};

TEST_F(DronecanEscTest, ConvertsShaftRpmWithConfiguredPoleCount)
{
    for (const uint8_t poles : {4, 14, 28}) {
        motorConfig_System.motorPoleCount = poles;
        feed(98000);
        EXPECT_EQ(98000U * poles / 200U, samples[0].rpm);
        EXPECT_EQ(2450, samples[0].voltage);
        EXPECT_EQ(250, samples[0].current);
        EXPECT_EQ(26, samples[0].temperature);
    }
    EXPECT_EQ(3U, publications);
}

TEST_F(DronecanEscTest, StoresReverseSpeedMagnitude)
{
    feed(-98765, 24.5f, -2.5f, 300.0f, 2);
    EXPECT_EQ(2, publishedMotor);
    EXPECT_EQ(6913, samples[2].rpm);
    EXPECT_EQ(-250, samples[2].current);
}

TEST_F(DronecanEscTest, SaturatesSpeedAtLegacyStoreLimit)
{
    for (const uint8_t poles : {254, 255}) {
        motorConfig_System.motorPoleCount = poles;
        for (const int32_t rpm : {131071, -131072}) {
            feed(rpm);
            EXPECT_EQ(INT16_MAX, samples[0].rpm);
        }
    }
}

TEST_F(DronecanEscTest, PreservesZeroSpeed)
{
    feed(0);
    EXPECT_EQ(0, samples[0].rpm);
}

TEST_F(DronecanEscTest, HandlesUnknownAndInfiniteFields)
{
    for (const float value : {std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
        feed(1000, value, value, value);
        EXPECT_EQ(0, samples[0].voltage);
        EXPECT_EQ(0, samples[0].current);
        EXPECT_EQ(0, samples[0].temperature);
    }
}

TEST_F(DronecanEscTest, BoundsFiniteHalfFloatFields)
{
    feed(1000, 65504.0f, 65504.0f, 65504.0f);
    EXPECT_EQ(UINT16_MAX, samples[0].voltage);
    EXPECT_EQ(6550400, samples[0].current);
    EXPECT_EQ(UINT8_MAX, samples[0].temperature);
    feed(1000, -65504.0f, -65504.0f, -65504.0f);
    EXPECT_EQ(0, samples[0].voltage);
    EXPECT_EQ(-6550400, samples[0].current);
    EXPECT_EQ(0, samples[0].temperature);
}

TEST_F(DronecanEscTest, RejectsTruncatedStatus)
{
    for (uint16_t length = 0; length < 14; length++) {
        feed(98000, 24.5f, 2.5f, 300.0f, 0, length);
    }
    EXPECT_EQ(0U, publications);
}

TEST_F(DronecanEscTest, RejectsInvalidMotorIndex)
{
    feed(98000, 24.5f, 2.5f, 300.0f, 31);
    EXPECT_EQ(0U, publications);
}

TEST_F(DronecanEscTest, AgesSampleReceivedAtTimerZero)
{
    feed(98000);
    dronecanEscUpdate(99999);
    EXPECT_EQ(0U, ages[0]);
    dronecanEscUpdate(100000);
    EXPECT_EQ(1U, ages[0]);
    dronecanEscUpdate(200000);
    EXPECT_EQ(2U, ages[0]);
    EXPECT_EQ(0U, ages[1]);
}

TEST_F(DronecanEscTest, AgesAcrossWrapAndRefreshesOnNewStatus)
{
    mockMicros = UINT32_MAX - 49999;
    feed(98000);
    dronecanEscUpdate(49999);
    EXPECT_EQ(0U, ages[0]);
    dronecanEscUpdate(50000);
    EXPECT_EQ(1U, ages[0]);
    mockMicros = 60000;
    feed(98000);
    dronecanEscUpdate(159999);
    EXPECT_EQ(1U, ages[0]);
    dronecanEscUpdate(160000);
    EXPECT_EQ(2U, ages[0]);
}

TEST_F(DronecanEscTest, DoesNotAgeUnreceivedOrReinitializedSlots)
{
    dronecanEscUpdate(100000);
    EXPECT_EQ(0U, ages[0]);
    feed(98000);
    dronecanEscInit();
    dronecanEscUpdate(200000);
    EXPECT_EQ(0U, ages[0]);
}
