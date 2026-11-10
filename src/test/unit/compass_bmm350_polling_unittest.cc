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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>

extern "C" {
#include "platform.h"
#include "target.h"
#include "drivers/bus.h"
#include "drivers/compass/compass_bmm350.h"
}

#include "gtest/gtest.h"

namespace {
// Register values follow Bosch SensorAPI v1.10.0, bmm350_defs.h.
struct DeviceModel;
std::map<const busDevice_t *, DeviceModel *> devices;

struct DeviceModel {
    magDev_t mag{};
    busDevice_t bus{};
    bmm350State_t state{};
    uint8_t registers[256]{};
    uint16_t otp[32]{};
    uint8_t payload[12]{};
    uint8_t otpIndex = 0;
    uint8_t otpStatus = 1;
    uint8_t normalFlags = 0;
    int failRead = -1;
    int failWrite = -1;
    int failWriteValue = -1;
    int failResetCount = 0;
    int badReadback = -1;
    unsigned resetAttempts = 0;
    unsigned starts = 0;
    unsigned statusReads = 0;
    unsigned pmuReads = 0;
    bool ignoreLatch = false;
    bool busy = false;
    bool transferError = false;
    bool failStart = false;
    bool pmuBusy = false;

    DeviceModel()
    {
        bus.busType = BUS_TYPE_I2C;
        mag.dev.bus = &bus;
        mag.dev.busType_u.i2c.address = 0x14;
        registers[0x00] = 0x33;
        registers[0x30] = 4;
        devices[&bus] = this;
        raw(10000, -20000, 30000, 30000);
    }

    ~DeviceModel()
    {
        devices.erase(&bus);
    }

    void raw(int32_t x, int32_t y, int32_t z, int32_t temperature)
    {
        const int32_t values[] = {x, y, z, temperature};
        for (unsigned channel = 0; channel < 4; channel++) {
            const uint32_t value = values[channel];
            for (unsigned byte = 0; byte < 3; byte++) {
                payload[channel * 3 + byte] = value >> (8 * byte);
            }
        }
    }

    bool init(bmm350Rate_e rate = BMM350_RATE_400_HZ)
    {
        return bmm350InitForPolling(&mag, &state, rate);
    }
};

DeviceModel &model(const extDevice_t *dev)
{
    return *devices.at(dev->bus);
}

class Bmm350PollingTest : public ::testing::Test {
protected:
    DeviceModel device;
    float magnetic[3] = {1.0f, 2.0f, 3.0f};
    float temperature = 4.0f;

    bmm350ReadResult_e poll()
    {
        return bmm350PollSample(&device.mag, magnetic, &temperature);
    }

    void expectUnchanged()
    {
        EXPECT_FLOAT_EQ(1.0f, magnetic[0]);
        EXPECT_FLOAT_EQ(2.0f, magnetic[1]);
        EXPECT_FLOAT_EQ(3.0f, magnetic[2]);
        EXPECT_FLOAT_EQ(4.0f, temperature);
    }
};
} // namespace

extern "C" {
void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}
void busDeviceRegister(const extDevice_t *) {}

bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    auto &m = model(dev);
    if (m.failRead == reg || length < 3) {
        return false;
    }
    data[0] = 0xa5;
    data[1] = 0x5a;
    for (unsigned index = 2; index < length; index++) {
        data[index] = m.registers[reg + index - 2];
    }
    if (reg == 0x55) {
        data[2] = m.otpStatus;
    } else if (reg == 0x52 && length == 4) {
        data[2] = m.otp[m.otpIndex] >> 8;
        data[3] = m.otp[m.otpIndex];
    } else if (reg == 0x07) {
        m.pmuReads++;
        if (m.pmuBusy) {
            data[2] |= 1;
        }
    } else if (reg == 0x30) {
        m.statusReads++;
    }
    if (m.badReadback == reg) {
        data[2] ^= 1;
    }
    return true;
}

bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t value)
{
    auto &m = model(dev);
    if (reg == 0x7e) {
        m.resetAttempts++;
        if (m.failResetCount > 0) {
            m.failResetCount--;
            return false;
        }
    }
    if (m.failWrite == reg && (m.failWriteValue < 0 || m.failWriteValue == value)) {
        return false;
    }
    m.registers[reg] = value;
    if (reg == 0x50 && value >= 0x20 && value < 0x40) {
        m.otpIndex = value & 0x1f;
    } else if (reg == 0x06) {
        m.registers[0x07] = (value << 5) | (value == 1 ? (8 | m.normalFlags) : 0);
    } else if (reg == 0x2e && m.ignoreLatch) {
        m.registers[reg] &= ~1;
    }
    return true;
}

bool busReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    auto &m = model(dev);
    if (m.failStart || reg != 0x31 || length != 14) {
        return false;
    }
    m.starts++;
    data[0] = 0xa5;
    data[1] = 0x5a;
    memcpy(data + 2, m.payload, sizeof(m.payload));
    m.registers[0x30] &= ~4;
    return true;
}

bool busBusy(const extDevice_t *dev, bool *error)
{
    const auto &m = model(dev);
    if (error) {
        *error = m.transferError;
    }
    return m.busy;
}
} // extern "C"

TEST_F(Bmm350PollingTest, InitializesExactRateAndInterruptProfiles)
{
    DeviceModel second;
    ASSERT_TRUE(device.init(BMM350_RATE_100_HZ));
    ASSERT_TRUE(second.init(BMM350_RATE_400_HZ));
    EXPECT_EQ(0x24, device.registers[0x04]);
    EXPECT_EQ(0x02, second.registers[0x04]);
    EXPECT_EQ(7, device.registers[0x05]);
    EXPECT_EQ(0x87, device.registers[0x2e]);
    EXPECT_EQ(0x87, second.registers[0x2e]);
    EXPECT_EQ(100, device.mag.magOdrHz);
    EXPECT_EQ(400, second.mag.magOdrHz);
    EXPECT_TRUE(device.state.trimValid && device.state.magneticResetComplete && device.state.pollingReady);
}

TEST_F(Bmm350PollingTest, NoDataReadyLeavesOutputsUnchanged)
{
    ASSERT_TRUE(device.init());
    device.registers[0x30] = 0;
    EXPECT_EQ(BMM350_SAMPLE_PENDING, poll());
    EXPECT_EQ(0u, device.starts);
    expectUnchanged();
}

TEST_F(Bmm350PollingTest, CompletesSynchronousBurstOnce)
{
    ASSERT_TRUE(device.init());
    ASSERT_EQ(BMM350_SAMPLE_READY, poll());
    EXPECT_EQ(1u, device.starts);
    EXPECT_FALSE(device.state.readInProgress);
    EXPECT_EQ(BMM350_SAMPLE_PENDING, poll());
    EXPECT_EQ(1u, device.starts);
}

TEST_F(Bmm350PollingTest, PendingTransferDoesNotReadClearedDataReadyAgain)
{
    ASSERT_TRUE(device.init());
    device.busy = true;
    EXPECT_EQ(BMM350_SAMPLE_PENDING, poll());
    expectUnchanged();
    const unsigned statusReads = device.statusReads;
    EXPECT_TRUE(device.state.readInProgress);
    EXPECT_EQ(BMM350_SAMPLE_PENDING, poll());
    device.busy = false;
    EXPECT_EQ(BMM350_SAMPLE_READY, poll());
    EXPECT_EQ(statusReads, device.statusReads);
    EXPECT_EQ(1u, device.starts);
}

TEST_F(Bmm350PollingTest, TransferErrorDiscardsOutputAndAllowsRecovery)
{
    ASSERT_TRUE(device.init());
    device.busy = true;
    ASSERT_EQ(BMM350_SAMPLE_PENDING, poll());
    device.busy = false;
    device.transferError = true;
    EXPECT_EQ(BMM350_SAMPLE_ERROR, poll());
    expectUnchanged();
    EXPECT_FALSE(device.state.readInProgress);
    device.transferError = false;
    device.registers[0x30] = 4;
    EXPECT_EQ(BMM350_SAMPLE_READY, poll());
    EXPECT_EQ(2u, device.starts);
}

TEST_F(Bmm350PollingTest, StatusReadAndBurstStartFailuresAreErrors)
{
    ASSERT_TRUE(device.init());
    device.failRead = 0x30;
    EXPECT_EQ(BMM350_SAMPLE_ERROR, poll());
    expectUnchanged();
    device.failRead = -1;
    device.failStart = true;
    EXPECT_EQ(BMM350_SAMPLE_ERROR, poll());
    EXPECT_FALSE(device.state.readInProgress);
    expectUnchanged();
}

TEST_F(Bmm350PollingTest, InterleavedDevicesRetainTheirOwnOtpAndPendingTransfers)
{
    DeviceModel second;
    device.otp[0x0e] = 123;
    second.otp[0x0e] = 456;
    ASSERT_TRUE(device.init());
    ASSERT_TRUE(second.init(BMM350_RATE_100_HZ));
    device.busy = true;
    ASSERT_EQ(BMM350_SAMPLE_PENDING, poll());
    float other[3], otherTemperature;
    ASSERT_EQ(BMM350_SAMPLE_READY, bmm350PollSample(&second.mag, other, &otherTemperature));
    device.busy = false;
    ASSERT_EQ(BMM350_SAMPLE_READY, poll());
    EXPECT_NEAR(333.0f, other[0] - magnetic[0], 0.001f);
    EXPECT_EQ(1u, device.starts);
    EXPECT_EQ(1u, second.starts);
    EXPECT_NE(device.mag.dev.callbackArg, second.mag.dev.callbackArg);
}

TEST_F(Bmm350PollingTest, RetriesFailedSoftResetWrite)
{
    device.failResetCount = 1;
    ASSERT_TRUE(device.init());
    EXPECT_EQ(2u, device.resetAttempts);
}

TEST_F(Bmm350PollingTest, RejectsExhaustedSoftResetWrites)
{
    device.failResetCount = 3;
    EXPECT_FALSE(device.init());
    EXPECT_EQ(3u, device.resetAttempts);
    EXPECT_FALSE(device.state.pollingReady);
}

TEST_F(Bmm350PollingTest, RejectsOtpErrorTimeoutAndPowerDownWriteFailure)
{
    device.otpStatus = 0xe0;
    EXPECT_FALSE(device.init());
    device.otpStatus = 0;
    EXPECT_FALSE(device.init());
    device.otpStatus = 1;
    device.failWrite = 0x50;
    device.failWriteValue = 0x80;
    EXPECT_FALSE(device.init());
    EXPECT_FALSE(device.state.pollingReady);
}

TEST_F(Bmm350PollingTest, RejectsPmuBusyIllegalAndOverriddenConfiguration)
{
    device.pmuBusy = true;
    EXPECT_FALSE(device.init());
    EXPECT_LT(device.pmuReads, 100u);
    device.pmuBusy = false;
    device.normalFlags = 0x10;
    EXPECT_FALSE(device.init());
    device.normalFlags = 0x02;
    EXPECT_FALSE(device.init());
    EXPECT_FALSE(device.state.pollingReady);
}

TEST_F(Bmm350PollingTest, RejectsRegisterReadAndWriteErrorsDuringInitialization)
{
    for (int reg : {0x04, 0x05, 0x07, 0x2e, 0x52}) {
        device.failRead = reg;
        EXPECT_FALSE(device.init()) << reg;
    }
    device.failRead = -1;
    for (int reg : {0x04, 0x05, 0x06, 0x2e}) {
        device.failWrite = reg;
        EXPECT_FALSE(device.init()) << reg;
    }
}

TEST_F(Bmm350PollingTest, RejectsRateAxisAndLatchedInterruptReadbackMismatch)
{
    device.badReadback = 0x04;
    EXPECT_FALSE(device.init());
    device.badReadback = 0x05;
    EXPECT_FALSE(device.init());
    device.badReadback = -1;
    device.ignoreLatch = true;
    EXPECT_FALSE(device.init());
    EXPECT_FALSE(device.state.pollingReady);
}

TEST_F(Bmm350PollingTest, RejectsReinitializationWithAnOutstandingTransfer)
{
    ASSERT_TRUE(device.init());
    device.busy = true;
    ASSERT_EQ(BMM350_SAMPLE_PENDING, poll());
    const unsigned resets = device.resetAttempts;
    EXPECT_FALSE(device.init());
    EXPECT_EQ(resets, device.resetAttempts);
    EXPECT_TRUE(device.state.readInProgress);
    device.busy = false;
    EXPECT_EQ(BMM350_SAMPLE_READY, poll());
}

TEST_F(Bmm350PollingTest, RejectsMissingContextInvalidRateAndNullOutput)
{
    EXPECT_EQ(BMM350_SAMPLE_ERROR, poll());
    EXPECT_FALSE(device.init(static_cast<bmm350Rate_e>(200)));
    ASSERT_TRUE(device.init());
    EXPECT_EQ(BMM350_SAMPLE_ERROR, bmm350PollSample(nullptr, magnetic, &temperature));
    EXPECT_EQ(BMM350_SAMPLE_ERROR, bmm350PollSample(&device.mag, nullptr, &temperature));
    EXPECT_EQ(BMM350_SAMPLE_ERROR, bmm350PollSample(&device.mag, magnetic, nullptr));
    EXPECT_EQ(0u, device.starts);
}

TEST_F(Bmm350PollingTest, RejectsNonfiniteCompensationWithoutPublishingIt)
{
    ASSERT_TRUE(device.init());
    device.state.comp.crossXY = 1.0f;
    device.state.comp.crossYX = 1.0f;
    EXPECT_EQ(BMM350_SAMPLE_ERROR, poll());
    expectUnchanged();
}

TEST_F(Bmm350PollingTest, RejectsNonI2cBusAndNullLegacyOutput)
{
    device.bus.busType = BUS_TYPE_SPI;
    EXPECT_FALSE(device.init());
    device.bus.busType = BUS_TYPE_I2C;
    ASSERT_TRUE(device.init());
    EXPECT_FALSE(device.mag.read(&device.mag, nullptr));
    EXPECT_EQ(0u, device.starts);
}

TEST_F(Bmm350PollingTest, ClampsLegacyOutputBeforeIntegerConversion)
{
    ASSERT_TRUE(device.init());
    device.state.comp.offsetX = 1e20f;
    device.state.comp.offsetY = -1e20f;
    int16_t output[3] = {1, 2, 3};
    EXPECT_FALSE(device.mag.read(&device.mag, output));
    ASSERT_TRUE(device.mag.read(&device.mag, output));
    EXPECT_EQ(INT16_MAX, output[0]);
    EXPECT_EQ(INT16_MIN, output[1]);
}

TEST_F(Bmm350PollingTest, MatchesIndependentBoschSensorApi110Vectors)
{
    // Generated by unmodified Bosch SensorAPI 3daf377ccaf589319c0d41af192105e9275987a2.
    // Its public compensation API reads these raw vectors and unpacks the same OTP words.
    const uint16_t mixedOtp[32] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0xf420, 0xf123, 0x08ab, 0xe4cd, 0x34c5, 0x10e0, 0xf82a, 0x21c4,
        0xec14, 0x18f0, 0, 0xf600, 0, 0, 0, 0, 0, 0, 0,
    };
    const int32_t raw[][4] = {
        {10000, -20000, 30000, 30000},
        {-54321, 22222, 198765, 40000},
        {1, -1, 0, 0},
        {-8388608, 8388607, -1, -40000},
    };
    const float expected[][4] = {
        {70.6997833f, -141.399567f, 215.248917f, 3.94845557f},
        {-53.7121201f, 37.0053101f, -132.335342f, 19.8387451f},
        {342.65152f, -122.00898f, -1904.84558f, -18.4925785f},
        {-59307.2773f, 59307.2695f, -0.00717496406f, -64.741272f},
    };
    for (unsigned index = 0; index < 4; index++) {
        memset(device.otp, 0, sizeof(device.otp));
        if (index == 1 || index == 2) {
            memcpy(device.otp, mixedOtp, sizeof(mixedOtp));
        }
        ASSERT_TRUE(device.init());
        device.raw(raw[index][0], raw[index][1], raw[index][2], raw[index][3]);
        device.registers[0x30] = 4;
        ASSERT_EQ(BMM350_SAMPLE_READY, poll());
        for (unsigned axis = 0; axis < 3; axis++) {
            const float tolerance = std::max(0.0005f, fabsf(expected[index][axis]) * 1e-6f);
            EXPECT_NEAR(expected[index][axis], magnetic[axis], tolerance) << index << ':' << axis;
        }
        EXPECT_NEAR(expected[index][3], temperature, 0.0001f) << index;
    }
}
