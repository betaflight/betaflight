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
#include "target.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_bmm350.h"
#include "drivers/bus.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {
constexpr uint8_t BMM350_REG_CHIP_ID = 0x00;
constexpr uint8_t BMM350_CHIP_ID = 0x33;
constexpr uint8_t BMM350_I2C_ADDRESS_ADSEL_GND = 0x14;
constexpr uint8_t BMM350_I2C_ADDRESS_ADSEL_VDDIO = 0x15;
constexpr uint8_t BMM350_I2C_DUMMY_BYTES = 2;
} // namespace

static bool mock_busReadRegisterBuffer_ret = true;
static uint8_t mock_chipId = BMM350_CHIP_ID;
static uint8_t mock_failAddress = 0;
static uint8_t last_reg = 0;
static uint8_t last_len = 0;
static uint8_t last_address = 0;
static int busReadRegisterBuffer_callCount = 0;

static void resetMocks(void)
{
    mock_busReadRegisterBuffer_ret = true;
    mock_chipId = BMM350_CHIP_ID;
    mock_failAddress = 0;
    last_reg = 0;
    last_len = 0;
    last_address = 0;
    busReadRegisterBuffer_callCount = 0;
}

static void initMagDev(magDev_t *mag, busDevice_t *bus, uint8_t address)
{
    memset(mag, 0, sizeof(*mag));
    memset(bus, 0, sizeof(*bus));
    bus->busType = BUS_TYPE_I2C;
    mag->dev.bus = bus;
    mag->dev.busType_u.i2c.address = address;
}

extern "C" {

void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}
bool busBusy(const extDevice_t *, bool *) { return false; }

bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    busReadRegisterBuffer_callCount++;
    last_reg = reg;
    last_len = len;
    last_address = dev->busType_u.i2c.address;

    if (mock_failAddress != 0 && last_address == mock_failAddress) {
        return false;
    }

    if (!mock_busReadRegisterBuffer_ret) {
        return false;
    }

    if (buf && len > 0) {
        memset(buf, 0, len);
        // BMM350 prepends two dummy bytes; CHIP_ID is the first payload byte.
        if (len > BMM350_I2C_DUMMY_BYTES) {
            buf[BMM350_I2C_DUMMY_BYTES] = mock_chipId;
        }
    }

    return true;
}

bool busWriteRegister(const extDevice_t *, uint8_t, uint8_t) { return true; }
void busDeviceRegister(const extDevice_t *) {}

} // extern "C"

TEST(Bmm350DetectTest, DetectSuccessDefaultAddress)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, 0);

    EXPECT_TRUE(bmm350Detect(&mag));
    EXPECT_NE(nullptr, mag.init);
    EXPECT_NE(nullptr, mag.read);
    EXPECT_EQ(BMM350_I2C_ADDRESS_ADSEL_GND, mag.dev.busType_u.i2c.address);
    EXPECT_EQ(BMM350_REG_CHIP_ID, last_reg);
    EXPECT_EQ(1 + BMM350_I2C_DUMMY_BYTES, last_len);
}

TEST(Bmm350DetectTest, DetectFailWrongChipId)
{
    resetMocks();
    mock_chipId = 0xFF;

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, 0);

    EXPECT_FALSE(bmm350Detect(&mag));
    EXPECT_EQ(nullptr, mag.init);
    EXPECT_EQ(0, mag.dev.busType_u.i2c.address);
}

TEST(Bmm350DetectTest, DetectFailBusError)
{
    resetMocks();
    mock_busReadRegisterBuffer_ret = false;

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, 0);

    EXPECT_FALSE(bmm350Detect(&mag));
}

TEST(Bmm350DetectTest, DetectFallbackTo0x15)
{
    resetMocks();
    mock_failAddress = BMM350_I2C_ADDRESS_ADSEL_GND;

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, 0);

    EXPECT_TRUE(bmm350Detect(&mag));
    EXPECT_EQ(BMM350_I2C_ADDRESS_ADSEL_VDDIO, mag.dev.busType_u.i2c.address);
    EXPECT_NE(nullptr, mag.init);
    EXPECT_NE(nullptr, mag.read);
}

TEST(Bmm350DetectTest, DetectPreservesExistingAddress)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, BMM350_I2C_ADDRESS_ADSEL_VDDIO);

    EXPECT_TRUE(bmm350Detect(&mag));
    EXPECT_EQ(BMM350_I2C_ADDRESS_ADSEL_VDDIO, mag.dev.busType_u.i2c.address);
    EXPECT_EQ(1, busReadRegisterBuffer_callCount);
    EXPECT_EQ(BMM350_I2C_ADDRESS_ADSEL_VDDIO, last_address);
}

TEST(Bmm350DetectTest, DetectConfiguredAddressDoesNotFallback)
{
    resetMocks();
    mock_chipId = 0x00;

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, BMM350_I2C_ADDRESS_ADSEL_GND);

    EXPECT_FALSE(bmm350Detect(&mag));
    EXPECT_EQ(BMM350_I2C_ADDRESS_ADSEL_GND, mag.dev.busType_u.i2c.address);
    EXPECT_EQ(1, busReadRegisterBuffer_callCount);
}

TEST(Bmm350DetectTest, ChipIdNotAcceptedFromDummyBytes)
{
    resetMocks();

    magDev_t mag;
    busDevice_t bus;
    initMagDev(&mag, &bus, 0);

    // Mock only places CHIP_ID at payload offset 2. A 1-byte read would
    // see 0x00 and fail — DetectSuccess already requires len == 3.
    ASSERT_TRUE(bmm350Detect(&mag));
    EXPECT_GT(last_len, BMM350_I2C_DUMMY_BYTES);
}
