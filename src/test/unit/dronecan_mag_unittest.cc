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

#include <cstdint>
#include <cstring>

extern "C" {
    #include "platform.h"

    #include "canard.h"

    #include "common/axis.h"

    #include "io/dronecan/dronecan.h"
    #include "io/dronecan/dronecan_mag.h"
    #include "io/dronecan/dronecan_msg.h"

    // dronecanRegisterSubscriber() lives in dronecan.c, which is not linked
    // into this test. Capture the handlers the module registers, keyed by
    // data type id, so the tests can feed hand-built transfers directly.
    static dronecanRxHandler capturedMag2Handler = nullptr;
    static dronecanRxHandler capturedLegacyHandler = nullptr;

    bool dronecanRegisterSubscriber(const dronecanSubscriber_t *subscriber)
    {
        if (subscriber->dataTypeId == UAVCAN_MAG2_ID) {
            capturedMag2Handler = subscriber->handler;
        } else if (subscriber->dataTypeId == UAVCAN_MAG_ID) {
            capturedLegacyHandler = subscriber->handler;
        }
        return true;
    }

    // Controllable clock; the module timestamps each accepted frame with it.
    static timeUs_t mockMicros = 0;

    timeUs_t micros(void)
    {
        return mockMicros;
    }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Assemble a MagneticFieldStrength2 payload: uint8 sensor_id followed by the
// fixed float16[3] field vector. Covariance (the TAO tail) is omitted.
static uint8_t payloadBuf[7];

static uint16_t buildMag2Payload(uint8_t sensorId, float x, float y, float z)
{
    uint16_t fx = canardConvertNativeFloatToFloat16(x);
    uint16_t fy = canardConvertNativeFloatToFloat16(y);
    uint16_t fz = canardConvertNativeFloatToFloat16(z);

    memset(payloadBuf, 0, sizeof(payloadBuf));
    canardEncodeScalar(payloadBuf, 0,  8,  &sensorId);
    canardEncodeScalar(payloadBuf, 8,  16, &fx);
    canardEncodeScalar(payloadBuf, 24, 16, &fy);
    canardEncodeScalar(payloadBuf, 40, 16, &fz);
    return sizeof(payloadBuf);
}

static void feedMag2(uint8_t sensorId, float x, float y, float z)
{
    CanardRxTransfer transfer;
    memset(&transfer, 0, sizeof(transfer));
    transfer.transfer_type = CanardTransferTypeBroadcast;
    transfer.data_type_id = UAVCAN_MAG2_ID;
    transfer.payload_head = payloadBuf;
    transfer.payload_len = buildMag2Payload(sensorId, x, y, z);

    capturedMag2Handler(nullptr, &transfer);
}

// The legacy MagneticFieldStrength payload is the same vector with no
// sensor_id in front.
static void feedMagLegacy(float x, float y, float z)
{
    uint16_t fx = canardConvertNativeFloatToFloat16(x);
    uint16_t fy = canardConvertNativeFloatToFloat16(y);
    uint16_t fz = canardConvertNativeFloatToFloat16(z);

    memset(payloadBuf, 0, sizeof(payloadBuf));
    canardEncodeScalar(payloadBuf, 0,  16, &fx);
    canardEncodeScalar(payloadBuf, 16, 16, &fy);
    canardEncodeScalar(payloadBuf, 32, 16, &fz);

    CanardRxTransfer transfer;
    memset(&transfer, 0, sizeof(transfer));
    transfer.transfer_type = CanardTransferTypeBroadcast;
    transfer.data_type_id = UAVCAN_MAG_ID;
    transfer.payload_head = payloadBuf;
    transfer.payload_len = 6;

    capturedLegacyHandler(nullptr, &transfer);
}

class DronecanMagTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mockMicros = 0;
        capturedMag2Handler = nullptr;
        capturedLegacyHandler = nullptr;
        dronecanMagInit();
        ASSERT_NE(capturedMag2Handler, nullptr);
        ASSERT_NE(capturedLegacyHandler, nullptr);
    }
};

TEST_F(DronecanMagTest, NoFrameYet)
{
    int16_t mag[XYZ_AXIS_COUNT] = {1, 2, 3};
    EXPECT_FALSE(dronecanMagGetLatest(mag));
}

TEST_F(DronecanMagTest, DecodesFieldToMilligauss)
{
    mockMicros = 123456;
    feedMag2(0, 0.4f, -0.2f, 0.1f);

    int16_t mag[XYZ_AXIS_COUNT] = {0};
    ASSERT_TRUE(dronecanMagGetLatest(mag));

    // float16 rounding plus the truncating int cast; allow a couple of counts.
    EXPECT_NEAR(mag[X], 400, 2);
    EXPECT_NEAR(mag[Y], -200, 2);
    EXPECT_NEAR(mag[Z], 100, 2);

    EXPECT_EQ(dronecanMagLastUpdateUs(), (timeUs_t)123456);
}

TEST_F(DronecanMagTest, DecodesLegacyMessage)
{
    mockMicros = 5000;
    feedMagLegacy(0.25f, -0.6f, 0.05f);

    int16_t mag[XYZ_AXIS_COUNT] = {0};
    ASSERT_TRUE(dronecanMagGetLatest(mag));
    EXPECT_NEAR(mag[X], 250, 2);
    EXPECT_NEAR(mag[Y], -600, 2);
    EXPECT_NEAR(mag[Z], 50, 2);
    EXPECT_EQ(dronecanMagLastUpdateUs(), (timeUs_t)5000);
}

TEST_F(DronecanMagTest, LatchesFirstSensorIdAndIgnoresOthers)
{
    mockMicros = 1000;
    feedMag2(2, 0.3f, 0.0f, 0.0f);

    int16_t first[XYZ_AXIS_COUNT] = {0};
    ASSERT_TRUE(dronecanMagGetLatest(first));

    // A frame from a different sensor_id on the same node must be dropped: the
    // cache and its timestamp stay put.
    mockMicros = 2000;
    feedMag2(5, -0.5f, -0.5f, -0.5f);

    int16_t second[XYZ_AXIS_COUNT] = {0};
    ASSERT_TRUE(dronecanMagGetLatest(second));
    EXPECT_EQ(second[X], first[X]);
    EXPECT_EQ(second[Y], first[Y]);
    EXPECT_EQ(second[Z], first[Z]);
    EXPECT_EQ(dronecanMagLastUpdateUs(), (timeUs_t)1000);

    // The originally-latched sensor still updates the cache and timestamp.
    mockMicros = 3000;
    feedMag2(2, 0.1f, 0.1f, 0.1f);

    int16_t third[XYZ_AXIS_COUNT] = {0};
    ASSERT_TRUE(dronecanMagGetLatest(third));
    EXPECT_NEAR(third[X], 100, 2);
    EXPECT_EQ(dronecanMagLastUpdateUs(), (timeUs_t)3000);
}

TEST_F(DronecanMagTest, TimestampTracksAcceptedFrame)
{
    // The accessor reports receipt and the host-clock timestamp; the staleness
    // window itself is applied by the compass read function against these.
    mockMicros = 5000;
    feedMag2(0, 0.2f, 0.2f, 0.2f);
    EXPECT_EQ(dronecanMagLastUpdateUs(), (timeUs_t)5000);

    mockMicros = 5000 + 600000; // 600 ms later
    int16_t mag[XYZ_AXIS_COUNT] = {0};
    EXPECT_TRUE(dronecanMagGetLatest(mag));
    EXPECT_EQ((timeUs_t)(mockMicros - dronecanMagLastUpdateUs()), (timeUs_t)600000);
}
