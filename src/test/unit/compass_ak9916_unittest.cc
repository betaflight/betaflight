/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later
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
#include "target.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_ak9916.h"
#include "drivers/bus.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {
constexpr uint8_t AK9916_REG_WIA2 = 0x01;
constexpr uint8_t AK9916_REG_ST1 = 0x10;
constexpr uint8_t AK9916_REG_ST2 = 0x18;
constexpr uint8_t AK9916_REG_CNTL2 = 0x31;
constexpr uint8_t AK9916_REG_CNTL3 = 0x32;
constexpr uint8_t AK9916_DEVICE_ID = 0x09;
constexpr uint8_t AK9916_I2C_ADDRESS = 0x0C;
// An address configured by the user, which detect() must not overwrite
constexpr uint8_t AK9916_CONFIGURED_I2C_ADDRESS = 0x0D;
constexpr uint8_t AK9916_SAMPLE_REGS_LEN = 9;
// Must match AK9916_NO_DATA_RETRIES in the driver
constexpr int AK9916_NO_DATA_RETRIES = 25;
} // namespace

static uint8_t mockRegisters[256];
static bool mock_readRet = true;
static bool mock_storeWrites = true;
static uint8_t lastReadReg = 0;
static uint8_t lastReadLen = 0;
static uint8_t lastReadAddress = 0;
static int readCallCount = 0;
static uint8_t lastWriteReg = 0;
static uint8_t lastWriteData = 0;
static uint8_t firstWriteReg = 0;
static uint8_t firstWriteData = 0;
static int writeCallCount = 0;

static void resetWriteTracking(void)
{
    lastWriteReg = 0;
    lastWriteData = 0;
    firstWriteReg = 0;
    firstWriteData = 0;
    writeCallCount = 0;
}

static void resetMocks(void)
{
    memset(mockRegisters, 0, sizeof(mockRegisters));
    mockRegisters[AK9916_REG_WIA2] = AK9916_DEVICE_ID;
    mock_readRet = true;
    mock_storeWrites = true;
    lastReadReg = 0;
    lastReadLen = 0;
    lastReadAddress = 0;
    readCallCount = 0;
    resetWriteTracking();
}

static void initMagDev(magDev_t *mag, busDevice_t *bus, uint8_t address)
{
    memset(mag, 0, sizeof(*mag));
    memset(bus, 0, sizeof(*bus));
    bus->busType = BUS_TYPE_I2C;
    mag->dev.bus = bus;
    mag->dev.busType_u.i2c.address = address;
}

// A sample with X = 100, Y = -100, Z = 256 counts
static void setSample(uint8_t st1, int16_t x, int16_t y, int16_t z, uint8_t st2)
{
    mockRegisters[AK9916_REG_ST1] = st1;
    mockRegisters[AK9916_REG_ST1 + 1] = x & 0xFF;
    mockRegisters[AK9916_REG_ST1 + 2] = (x >> 8) & 0xFF;
    mockRegisters[AK9916_REG_ST1 + 3] = y & 0xFF;
    mockRegisters[AK9916_REG_ST1 + 4] = (y >> 8) & 0xFF;
    mockRegisters[AK9916_REG_ST1 + 5] = z & 0xFF;
    mockRegisters[AK9916_REG_ST1 + 6] = (z >> 8) & 0xFF;
    mockRegisters[AK9916_REG_ST2] = st2;
}

extern "C" {

void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}
bool busBusy(const extDevice_t *, bool *) { return false; }

bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    readCallCount++;
    lastReadReg = reg;
    lastReadLen = len;
    lastReadAddress = dev->busType_u.i2c.address;

    if (!mock_readRet) {
        return false;
    }

    if (buf && len > 0) {
        memcpy(buf, &mockRegisters[reg], len);
    }

    return true;
}

bool busWriteRegister(const extDevice_t *, uint8_t reg, uint8_t data)
{
    writeCallCount++;
    if (writeCallCount == 1) {
        firstWriteReg = reg;
        firstWriteData = data;
    }
    lastWriteReg = reg;
    lastWriteData = data;
    if (mock_storeWrites) {
        mockRegisters[reg] = data;
    }
    return true;
}

void busDeviceRegister(const extDevice_t *) {}

} // extern "C"

TEST(Ak9916DetectTest, DetectSuccessDefaultAddress)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, 0);

    EXPECT_TRUE(ak9916Detect(&mag));
    EXPECT_NE(nullptr, mag.init);
    EXPECT_NE(nullptr, mag.read);
    EXPECT_EQ(AK9916_I2C_ADDRESS, mag.dev.busType_u.i2c.address);
    EXPECT_EQ(AK9916_REG_WIA2, lastReadReg);
    EXPECT_EQ(1, lastReadLen);
}

TEST(Ak9916DetectTest, DetectFailsOnAk8975CompanyId)
{
    // The AK8975/AK8963 WHO_AM_I value is the AK9916 company id, so it must
    // not be enough to identify the chip as an AK9916.
    resetMocks();
    mockRegisters[AK9916_REG_WIA2] = 0x48;

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, 0);

    EXPECT_FALSE(ak9916Detect(&mag));
    EXPECT_EQ(nullptr, mag.init);
}

TEST(Ak9916DetectTest, DetectFailsOnBusError)
{
    resetMocks();
    mock_readRet = false;

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, 0);

    EXPECT_FALSE(ak9916Detect(&mag));
    EXPECT_EQ(nullptr, mag.init);
    EXPECT_EQ(nullptr, mag.read);
}

TEST(Ak9916DetectTest, DetectPreservesConfiguredAddress)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, AK9916_CONFIGURED_I2C_ADDRESS);

    EXPECT_TRUE(ak9916Detect(&mag));
    EXPECT_EQ(AK9916_CONFIGURED_I2C_ADDRESS, mag.dev.busType_u.i2c.address);
    EXPECT_EQ(AK9916_CONFIGURED_I2C_ADDRESS, lastReadAddress);
    EXPECT_EQ(1, readCallCount);
}

TEST(Ak9916DetectTest, DetectRejectsNonI2cBus)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, 0);
    bus.busType = BUS_TYPE_SPI;

    EXPECT_FALSE(ak9916Detect(&mag));
    EXPECT_EQ(0, readCallCount);
}

TEST(Ak9916InitTest, InitResetsAndSetsContinuousMode)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, AK9916_I2C_ADDRESS);
    ASSERT_TRUE(ak9916Detect(&mag));

    EXPECT_TRUE(mag.init(&mag));
    EXPECT_EQ(AK9916_REG_CNTL3, firstWriteReg);
    EXPECT_EQ(0x01, firstWriteData); // soft reset
    EXPECT_EQ(AK9916_REG_CNTL2, lastWriteReg);
    EXPECT_EQ(0x08, lastWriteData); // continuous mode 4, 100Hz
    EXPECT_EQ(100, mag.magOdrHz);
}

TEST(Ak9916InitTest, InitFailsWhenModeDoesNotStick)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, AK9916_I2C_ADDRESS);
    ASSERT_TRUE(ak9916Detect(&mag));

    // The CNTL2 write is acknowledged, but the register keeps its reset value:
    // e.g. the GPS module is not bridging its magnetometer to the I2C interface.
    mock_storeWrites = false;
    mockRegisters[AK9916_REG_CNTL2] = 0x00;

    EXPECT_FALSE(mag.init(&mag));

    // The failure comes from the mode read-back comparison, not from a write ack
    // or a bus error
    EXPECT_EQ(AK9916_REG_CNTL2, lastWriteReg);
    EXPECT_EQ(0x08, lastWriteData);
    EXPECT_EQ(AK9916_REG_CNTL2, lastReadReg);
    EXPECT_TRUE(mock_readRet);
    EXPECT_EQ(0, mag.magOdrHz);
}

TEST(Ak9916InitTest, InitResetsNoDataCounter)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, AK9916_I2C_ADDRESS);
    ASSERT_TRUE(ak9916Detect(&mag));

    int16_t magData[XYZ_AXIS_COUNT] = { 0, 0, 0 };

    // Accumulate a partial retry count
    setSample(0x00, 0, 0, 0, 0x00); // ST1.DRDY clear
    for (int i = 0; i < 10; i++) {
        EXPECT_FALSE(mag.read(&mag, magData));
    }
    EXPECT_EQ(0, writeCallCount);

    // Re-initialising the driver must clear the counter
    EXPECT_TRUE(mag.init(&mag));
    resetWriteTracking();

    for (int i = 0; i < AK9916_NO_DATA_RETRIES - 1; i++) {
        EXPECT_FALSE(mag.read(&mag, magData));
    }
    EXPECT_EQ(0, writeCallCount); // a stale counter would have re-armed here
}

TEST(Ak9916ReadTest, ReadScalesToMilligauss)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, AK9916_I2C_ADDRESS);
    ASSERT_TRUE(ak9916Detect(&mag));

    // 0.15uT/LSB, so 100 counts = 15uT = 150mG
    setSample(0x01, 100, -100, 256, 0x00);

    int16_t magData[XYZ_AXIS_COUNT] = { 0, 0, 0 };
    EXPECT_TRUE(mag.read(&mag, magData));
    EXPECT_EQ(150, magData[X]);
    EXPECT_EQ(-150, magData[Y]);
    EXPECT_EQ(384, magData[Z]);
    EXPECT_EQ(AK9916_REG_ST1, lastReadReg);
    EXPECT_EQ(AK9916_SAMPLE_REGS_LEN, lastReadLen);
}

TEST(Ak9916ReadTest, ReadRejectsOverflow)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, AK9916_I2C_ADDRESS);
    ASSERT_TRUE(ak9916Detect(&mag));

    setSample(0x01, 100, 100, 100, 0x08); // ST2.HOFL

    int16_t magData[XYZ_AXIS_COUNT] = { 0, 0, 0 };
    EXPECT_FALSE(mag.read(&mag, magData));
}

TEST(Ak9916ReadTest, ReadRejectsMissingDataReady)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, AK9916_I2C_ADDRESS);
    ASSERT_TRUE(ak9916Detect(&mag));

    int16_t magData[XYZ_AXIS_COUNT] = { 0, 0, 0 };

    // Deliver one good sample first, so the driver's internal no-data counter is
    // known to start at zero
    setSample(0x01, 100, 100, 100, 0x00);
    EXPECT_TRUE(mag.read(&mag, magData));
    resetWriteTracking();

    setSample(0x00, 100, 100, 100, 0x00); // ST1.DRDY clear

    // Below the retry threshold the mode is not re-armed
    for (int i = 0; i < AK9916_NO_DATA_RETRIES - 1; i++) {
        EXPECT_FALSE(mag.read(&mag, magData));
    }
    EXPECT_EQ(0, writeCallCount);

    // The threshold read re-arms continuous mode at 100Hz
    EXPECT_FALSE(mag.read(&mag, magData));
    EXPECT_EQ(1, writeCallCount);
    EXPECT_EQ(AK9916_REG_CNTL2, lastWriteReg);
    EXPECT_EQ(0x08, lastWriteData); // continuous mode 4, 100Hz
}

TEST(Ak9916ReadTest, ReadReturnsFalseOnBusError)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, AK9916_I2C_ADDRESS);
    ASSERT_TRUE(ak9916Detect(&mag));

    mock_readRet = false;

    int16_t magData[XYZ_AXIS_COUNT] = { 0, 0, 0 };
    EXPECT_FALSE(mag.read(&mag, magData));
}
