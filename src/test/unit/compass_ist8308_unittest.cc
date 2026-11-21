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
#include "drivers/compass/compass_ist8308.h"
#include "drivers/bus.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Register addresses and values mirrored from compass_ist8308.c
namespace {
constexpr uint8_t IST8308_REG_WAI             = 0x00;
constexpr uint8_t IST8308_WAI_VALID           = 0x08;
constexpr uint8_t IST8308_REG_STAT            = 0x10;
constexpr uint8_t IST8308_STAT_DRDY           = 0x01;
constexpr uint8_t IST8308_REG_CNTL2           = 0x31;
constexpr uint8_t IST8308_CNTL2_CONT_100HZ    = 0x08;
constexpr uint8_t IST8308_REG_CNTL3           = 0x32;
constexpr uint8_t IST8308_CNTL3_SRST          = 0x01;
constexpr uint8_t IST8308_REG_CNTL4           = 0x34;
constexpr uint8_t IST8308_REG_OSRCNTL         = 0x41;
constexpr uint8_t IST8308_I2C_ADDRESS_DEFAULT = 0x0C;
constexpr uint8_t IST8308_READ_LEN            = 7;
} // namespace

// ---------------------------------------------------------------------------
// Mocks
// ---------------------------------------------------------------------------
static bool mock_readReg_ret;
static uint8_t mock_readReg_value;
static uint8_t mock_readReg_lastReg;

static bool mock_readStart_ret;
static uint8_t mock_readStart_reg;
static uint8_t mock_readStart_len;
static uint8_t mock_readStart_buf[8];
static int readStart_callCount;

static bool mock_busBusy_error;

static bool mock_writeReg_ret;
static int writeReg_count;
static uint8_t writeReg_reg[8];
static uint8_t writeReg_val[8];

static void resetMocks(void)
{
    mock_readReg_ret = true;
    mock_readReg_value = IST8308_WAI_VALID;
    mock_readReg_lastReg = 0xFF;
    mock_readStart_ret = true;
    mock_readStart_reg = 0;
    mock_readStart_len = 0;
    memset(mock_readStart_buf, 0, sizeof(mock_readStart_buf));
    readStart_callCount = 0;
    mock_busBusy_error = false;
    mock_writeReg_ret = true;
    writeReg_count = 0;
    memset(writeReg_reg, 0, sizeof(writeReg_reg));
    memset(writeReg_val, 0, sizeof(writeReg_val));
}

extern "C" {

void delay(uint32_t) {}
bool busBusy(const extDevice_t*, bool *error)
{
    if (error) {
        *error = mock_busBusy_error;
    }
    return false;
}

bool busReadRegisterBuffer(const extDevice_t*, uint8_t reg, uint8_t *buf, uint8_t len)
{
    mock_readReg_lastReg = reg;
    if (buf && len > 0) {
        buf[0] = mock_readReg_value;
    }
    return mock_readReg_ret;
}

bool busReadRegisterBufferStart(const extDevice_t*, uint8_t reg, uint8_t *buf, uint8_t len)
{
    readStart_callCount++;
    mock_readStart_reg = reg;
    mock_readStart_len = len;
    if (mock_readStart_ret && buf && len > 0) {
        // Simulate async completion
        memcpy(buf, mock_readStart_buf, len);
    }
    return mock_readStart_ret;
}

bool busWriteRegister(const extDevice_t*, uint8_t reg, uint8_t val)
{
    if (writeReg_count < (int)sizeof(writeReg_reg)) {
        writeReg_reg[writeReg_count] = reg;
        writeReg_val[writeReg_count] = val;
    }
    writeReg_count++;
    return mock_writeReg_ret;
}

bool busWriteRegisterStart(const extDevice_t*, uint8_t, uint8_t) { return true; }

void busDeviceRegister(const extDevice_t*) {}

} // extern "C"

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------
class Ist8308Test : public ::testing::Test {
protected:
    magDev_t mag;
    busDevice_t bus;
    int16_t magData[3];

    void SetUp() override
    {
        resetMocks();
        memset(&mag, 0, sizeof(mag));
        memset(&bus, 0, sizeof(bus));
        bus.busType = BUS_TYPE_I2C;
        mag.dev.bus = &bus;
        memset(magData, 0, sizeof(magData));
    }

    // ist8308Read keeps a static pendingRead flag. Drive it to a known state:
    // after this the next call will start a new transfer.
    void syncReadState()
    {
        ASSERT_TRUE(ist8308Detect(&mag));
        for (int i = 0; i < 4; i++) {
            resetMocks();
            mock_readStart_buf[0] = IST8308_STAT_DRDY;
            if (mag.read(&mag, magData)) {
                resetMocks();
                return;
            }
        }
        FAIL() << "Unable to synchronize IST8308 read state";
    }

    void loadSample(int16_t x, int16_t y, int16_t z, uint8_t stat = IST8308_STAT_DRDY)
    {
        mock_readStart_buf[0] = stat;
        mock_readStart_buf[1] = (uint8_t)(x & 0xFF);
        mock_readStart_buf[2] = (uint8_t)((uint16_t)x >> 8);
        mock_readStart_buf[3] = (uint8_t)(y & 0xFF);
        mock_readStart_buf[4] = (uint8_t)((uint16_t)y >> 8);
        mock_readStart_buf[5] = (uint8_t)(z & 0xFF);
        mock_readStart_buf[6] = (uint8_t)((uint16_t)z >> 8);
    }
};

// ---------------------------------------------------------------------------
// Detect
// ---------------------------------------------------------------------------
TEST_F(Ist8308Test, DetectSuccessSetsDefaultAddress)
{
    EXPECT_TRUE(ist8308Detect(&mag));
    EXPECT_EQ(IST8308_REG_WAI, mock_readReg_lastReg);
    EXPECT_NE(nullptr, mag.init);
    EXPECT_NE(nullptr, mag.read);
    EXPECT_EQ(IST8308_I2C_ADDRESS_DEFAULT, mag.dev.busType_u.i2c.address);
}

TEST_F(Ist8308Test, DetectPreservesConfiguredAddress)
{
    mag.dev.busType_u.i2c.address = 0x0E;
    EXPECT_TRUE(ist8308Detect(&mag));
    EXPECT_EQ(0x0E, mag.dev.busType_u.i2c.address);
}

TEST_F(Ist8308Test, DetectRejectsIst8310Wai)
{
    mock_readReg_value = 0x10; // IST8310 WAI
    EXPECT_FALSE(ist8308Detect(&mag));
    EXPECT_EQ(nullptr, mag.init);
}

TEST_F(Ist8308Test, DetectRejectsAk8963Wai)
{
    mock_readReg_value = 0x48; // AKM WIA, shares address 0x0C
    EXPECT_FALSE(ist8308Detect(&mag));
}

TEST_F(Ist8308Test, DetectFailsOnBusError)
{
    mock_readReg_ret = false;
    EXPECT_FALSE(ist8308Detect(&mag));
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
TEST_F(Ist8308Test, InitSequence)
{
    ASSERT_TRUE(ist8308Detect(&mag));
    resetMocks();

    EXPECT_TRUE(mag.init(&mag));
    ASSERT_EQ(4, writeReg_count);
    EXPECT_EQ(IST8308_REG_CNTL3, writeReg_reg[0]);
    EXPECT_EQ(IST8308_CNTL3_SRST, writeReg_val[0]);
    EXPECT_EQ(IST8308_REG_OSRCNTL, writeReg_reg[1]);
    EXPECT_EQ(0x24, writeReg_val[1]); // OSR 16 on X/Z and Y
    EXPECT_EQ(IST8308_REG_CNTL4, writeReg_reg[2]);
    EXPECT_EQ(0x00, writeReg_val[2]); // +/-500uT
    // Continuous mode must be the last write
    EXPECT_EQ(IST8308_REG_CNTL2, writeReg_reg[3]);
    EXPECT_EQ(IST8308_CNTL2_CONT_100HZ, writeReg_val[3]);
    EXPECT_EQ(100, mag.magOdrHz);
}

TEST_F(Ist8308Test, InitFailsOnWriteError)
{
    ASSERT_TRUE(ist8308Detect(&mag));
    resetMocks();
    mock_writeReg_ret = false;
    EXPECT_FALSE(mag.init(&mag));
}

// ---------------------------------------------------------------------------
// Read
// ---------------------------------------------------------------------------
TEST_F(Ist8308Test, ReadBurstsStatusAndData)
{
    syncReadState();

    loadSample(100, 200, 300);
    EXPECT_FALSE(mag.read(&mag, magData));
    EXPECT_EQ(1, readStart_callCount);
    EXPECT_EQ(IST8308_REG_STAT, mock_readStart_reg);
    EXPECT_EQ(IST8308_READ_LEN, mock_readStart_len);

    resetMocks();
    EXPECT_TRUE(mag.read(&mag, magData));
    EXPECT_EQ(0, readStart_callCount);
    // 1.5 mG/LSB, Y inverted
    EXPECT_EQ(150, magData[0]);
    EXPECT_EQ(-300, magData[1]);
    EXPECT_EQ(450, magData[2]);
}

TEST_F(Ist8308Test, ReadNegativeAndFullScale)
{
    syncReadState();

    // +/-500uT at 0.15uT/LSB -> ~3333 LSB
    loadSample(-3334, -2, 3334);
    mag.read(&mag, magData);
    resetMocks();
    EXPECT_TRUE(mag.read(&mag, magData));
    EXPECT_EQ(-5001, magData[0]);
    EXPECT_EQ(3, magData[1]);
    EXPECT_EQ(5001, magData[2]);
}

TEST_F(Ist8308Test, ReadWithoutDrdyReturnsNoSample)
{
    syncReadState();

    magData[0] = magData[1] = magData[2] = 1234;
    loadSample(100, 200, 300, 0x00);
    EXPECT_FALSE(mag.read(&mag, magData));
    resetMocks();
    EXPECT_FALSE(mag.read(&mag, magData));
    EXPECT_EQ(1234, magData[0]);
    EXPECT_EQ(1234, magData[1]);
    EXPECT_EQ(1234, magData[2]);

    // Next call starts a fresh transfer
    resetMocks();
    EXPECT_FALSE(mag.read(&mag, magData));
    EXPECT_EQ(1, readStart_callCount);
}

TEST_F(Ist8308Test, ReadStartFailureRetries)
{
    syncReadState();

    mock_readStart_ret = false;
    EXPECT_FALSE(mag.read(&mag, magData));
    EXPECT_FALSE(mag.read(&mag, magData));
    EXPECT_EQ(2, readStart_callCount);

    resetMocks();
    loadSample(2, 4, 6);
    EXPECT_FALSE(mag.read(&mag, magData));
    resetMocks();
    EXPECT_TRUE(mag.read(&mag, magData));
    EXPECT_EQ(3, magData[0]);
    EXPECT_EQ(-6, magData[1]);
    EXPECT_EQ(9, magData[2]);
}

TEST_F(Ist8308Test, ReadTransferErrorDoesNotPublishStaleSample)
{
    syncReadState();

    // Good sample first
    loadSample(2, 4, 6);
    mag.read(&mag, magData);
    resetMocks();
    ASSERT_TRUE(mag.read(&mag, magData));

    // Next transfer NACKs: buf still holds the previous sample with DRDY set
    resetMocks();
    mock_readStart_ret = true;
    loadSample(2, 4, 6);
    EXPECT_FALSE(mag.read(&mag, magData));
    resetMocks();
    mock_busBusy_error = true;
    EXPECT_FALSE(mag.read(&mag, magData));

    // Following call starts a fresh transfer
    resetMocks();
    EXPECT_FALSE(mag.read(&mag, magData));
    EXPECT_EQ(1, readStart_callCount);
    EXPECT_EQ(IST8308_REG_STAT, mock_readStart_reg);
}
