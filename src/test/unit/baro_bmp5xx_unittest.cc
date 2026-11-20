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
 * License along with Betaflight.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>
#include <map>
#include <vector>

extern "C" {
#include "platform.h"
#include "target.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp5xx.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"
}

#include "gtest/gtest.h"

namespace {
struct RegisterBus {
    std::map<uint8_t, std::vector<uint8_t>> registers;
    bool spiSelected = true;
    bool resetNack = false;
    bool resetExecuted = true;
    uint8_t resetPor = 0x10;
    uint8_t resetNvm = 0x02;
    uint8_t failedWrite = 0xff;
    uint8_t failedRead = 0xff;
    uint8_t i2cAddress = 0;
    int effectiveOsrOverride = -1;
    unsigned selectReads = 0;
    unsigned resetWrites = 0;
    unsigned sampleReads = 0;
    unsigned ioHiCalls = 0;
    unsigned ioInitCalls = 0;
    unsigned ioConfigCalls = 0;
    unsigned spiSetClkDivisorCalls = 0;
    unsigned ioPreinitCalls = 0;
};
RegisterBus mock;

class Bmp5xxTest : public ::testing::Test {
  protected:
    busDevice_t bus{};
    baroDev_t baro{};
    bmp5xxConfig_t config{};

    void SetUp() override
    {
        mock = RegisterBus{};
        mock.registers[0x01] = {0x50};
        mock.registers[0x28] = {0x02};
        mock.registers[0x27] = {0x10};
        mock.registers[0x1d] = {0, 0, 24, 0, 0xa8, 0x61};
        bus.busType = BUS_TYPE_I2C;
        baro.dev.bus = &bus;
        baro.dev.busType_u.i2c.address = 0x46;
    }
};

// Retain the original SPI bus initialization/deinitialization regressions.
TEST_F(Bmp5xxTest, DetectInitializesSpiBusWhenSupportedSpiDefineIsEnabled)
{
    bus.busType = BUS_TYPE_SPI;
    baro.dev.busType_u.spi.csnPin = (IO_t)1;
    mock.spiSelected = false;
    uint8_t detectedChip = 0;

    EXPECT_TRUE(bmp5xxDetect(&config, &baro, &detectedChip));
    EXPECT_EQ(0x50, detectedChip);
    EXPECT_EQ(1u, mock.ioHiCalls);
    EXPECT_EQ(1u, mock.ioInitCalls);
    EXPECT_EQ(1u, mock.ioConfigCalls);
    EXPECT_EQ(1u, mock.spiSetClkDivisorCalls);
    EXPECT_EQ(0u, mock.ioPreinitCalls);
}

TEST_F(Bmp5xxTest, DetectDeinitializesSpiBusOnUnknownChip)
{
    bus.busType = BUS_TYPE_SPI;
    baro.dev.busType_u.spi.csnPin = (IO_t)1;
    mock.registers[0x01] = {0};

    EXPECT_FALSE(bmp5xxDetect(&config, &baro, nullptr));
    EXPECT_EQ(1u, mock.ioHiCalls);
    EXPECT_EQ(1u, mock.ioInitCalls);
    EXPECT_EQ(1u, mock.ioConfigCalls);
    EXPECT_EQ(1u, mock.spiSetClkDivisorCalls);
    EXPECT_EQ(1u, mock.ioPreinitCalls);
}

TEST_F(Bmp5xxTest, ReadsNativeCompensatedPressureAndTemperature)
{
    uint8_t chip = 0;
    ASSERT_TRUE(bmp5xxDetect(&config, &baro, &chip));
    EXPECT_EQ(0x50, chip);
    mock.registers[0x27] = {1};
    ASSERT_TRUE(baro.read_up(&baro));
    ASSERT_TRUE(baro.get_up(&baro));
    int32_t pressure = 0, temperature = 0;
    baro.calculate(&pressure, &temperature);
    EXPECT_EQ(100000, pressure);
    EXPECT_EQ(2400, temperature);
}

TEST_F(Bmp5xxTest, SelectsSpiAtColdStartAndAfterSoftReset)
{
    bus.busType = BUS_TYPE_SPI;
    mock.spiSelected = false;
    ASSERT_TRUE(bmp5xxDetect(&config, &baro, nullptr));
    EXPECT_EQ(2u, mock.selectReads);
    EXPECT_EQ(1u, mock.resetWrites);
}

TEST_F(Bmp5xxTest, ReadsSpiSampleWithoutIncludingTheDummyByte)
{
    bus.busType = BUS_TYPE_SPI;
    baro.dev.busType_u.spi.csnPin = (IO_t)1;
    mock.spiSelected = false;
    mock.registers[0x01] = {0x51};
    uint8_t detectedChip = 0;
    int32_t pressure = 0;
    int32_t temperature = 0;

    ASSERT_TRUE(bmp5xxDetect(&config, &baro, &detectedChip));
    ASSERT_EQ(0x51, detectedChip);
    mock.registers[0x27] = {1};
    ASSERT_EQ(BMP5XX_READ_READY, bmp5xxReadSample(&baro, &pressure, &temperature));
    EXPECT_EQ(100000, pressure);
    EXPECT_EQ(2400, temperature);
    EXPECT_EQ(1u, mock.sampleReads);
    EXPECT_EQ(BMP5XX_READ_PENDING, bmp5xxReadSample(&baro, &pressure, &temperature));
    EXPECT_EQ(1u, mock.sampleReads);
}

TEST_F(Bmp5xxTest, ReadsSecondaryAsicIdOverI2c)
{
    mock.registers[0x01] = {0x51};
    uint8_t detectedChip = 0;
    int32_t pressure = 0;
    int32_t temperature = 0;

    ASSERT_TRUE(bmp5xxDetect(&config, &baro, &detectedChip));
    ASSERT_EQ(0x51, detectedChip);
    mock.registers[0x27] = {1};
    ASSERT_EQ(BMP5XX_READ_READY, bmp5xxReadSample(&baro, &pressure, &temperature));
    EXPECT_EQ(100000, pressure);
    EXPECT_EQ(2400, temperature);
}

TEST_F(Bmp5xxTest, AcceptsI2cResetNackWithValidPostResetStatus)
{
    mock.resetNack = true;
    ASSERT_TRUE(bmp5xxDetect(&config, &baro, nullptr));
    EXPECT_EQ(1u, mock.resetWrites);
}

TEST_F(Bmp5xxTest, RejectsNvmNotReadyBeforeReset)
{
    mock.registers[0x28] = {0};
    EXPECT_FALSE(bmp5xxDetect(&config, &baro, nullptr));
    EXPECT_EQ(0u, mock.resetWrites);
}

TEST_F(Bmp5xxTest, ProbesSecondaryI2cAddressAfterPrimaryNack)
{
    baro.dev.busType_u.i2c.address = 0;
    mock.i2cAddress = 0x46;
    EXPECT_TRUE(bmp5xxDetect(&config, &baro, nullptr));
    EXPECT_EQ(0x46, baro.dev.busType_u.i2c.address);
}

TEST_F(Bmp5xxTest, RestoresAutomaticAddressAfterDetectionFailure)
{
    baro.dev.busType_u.i2c.address = 0;
    mock.i2cAddress = 0x45;
    EXPECT_FALSE(bmp5xxDetect(&config, &baro, nullptr));
    EXPECT_EQ(0, baro.dev.busType_u.i2c.address);
}

TEST_F(Bmp5xxTest, RejectsNvmError)
{
    mock.registers[0x28] = {0x06};
    EXPECT_FALSE(bmp5xxDetect(&config, &baro, nullptr));
}

TEST_F(Bmp5xxTest, RejectsMissingResetCompletion)
{
    mock.resetPor = 0;
    EXPECT_FALSE(bmp5xxDetect(&config, &baro, nullptr));
}

TEST_F(Bmp5xxTest, RejectsI2cResetFailureWithoutANewPor)
{
    // A power-up POR is already latched. It cannot prove that this new
    // reset command succeeded when the I2C controller reports a failure.
    mock.resetExecuted = false;
    EXPECT_FALSE(bmp5xxDetect(&config, &baro, nullptr));
}

TEST_F(Bmp5xxTest, RejectsNvmErrorAfterReset)
{
    mock.resetNvm = 0x0a;
    EXPECT_FALSE(bmp5xxDetect(&config, &baro, nullptr));
}

TEST_F(Bmp5xxTest, RejectsFailedConfigurationWrite)
{
    mock.failedWrite = 0x36;
    EXPECT_FALSE(bmp5xxDetect(&config, &baro, nullptr));
}

TEST_F(Bmp5xxTest, ProgramsAValid50HzOversamplingProfile)
{
    ASSERT_TRUE(bmp5xxDetect(&config, &baro, nullptr));
    // Datasheet table 7: pressure 16x / temperature 4x supports up to 80 Hz;
    // pressure 128x supports only 10 Hz and silently falls back at 50 Hz.
    EXPECT_EQ((std::vector<uint8_t>{0x62}), mock.registers[0x36]);
    EXPECT_EQ((std::vector<uint8_t>{0x3d}), mock.registers[0x37]);
}

TEST_F(Bmp5xxTest, SelectsFilteredPressureAndTemperatureInTheShadowRegisters)
{
    const bool detected = bmp5xxDetect(&config, &baro, nullptr);

    // Datasheet section 7.27: compensation[1:0], temperature IIR[3], pressure IIR[5].
    EXPECT_EQ((std::vector<uint8_t>{0x2b}), mock.registers[0x30]);
    EXPECT_EQ((std::vector<uint8_t>{0x24}), mock.registers[0x31]);
    EXPECT_TRUE(detected);
}

TEST_F(Bmp5xxTest, RejectsAnInvalidEffectiveOversamplingProfile)
{
    mock.effectiveOsrOverride = 0x09; // Invalid ODR with automatic 2x fallback.
    EXPECT_FALSE(bmp5xxDetect(&config, &baro, nullptr));
}

TEST_F(Bmp5xxTest, WaitsForDataReadyWithoutReadingOrReplayingData)
{
    ASSERT_TRUE(bmp5xxDetect(&config, &baro, nullptr));
    mock.registers[0x27] = {0x10};
    EXPECT_FALSE(baro.read_up(&baro));
    EXPECT_EQ(0u, mock.sampleReads);
    mock.registers[0x27] = {1};
    EXPECT_TRUE(baro.read_up(&baro));
    EXPECT_EQ(1u, mock.sampleReads);
}

TEST_F(Bmp5xxTest, RejectsStatusReadFailure)
{
    ASSERT_TRUE(bmp5xxDetect(&config, &baro, nullptr));
    mock.failedRead = 0x27;
    EXPECT_FALSE(baro.read_up(&baro));
    EXPECT_EQ(0u, mock.sampleReads);
}

TEST_F(Bmp5xxTest, SampleReadPreservesOutputsUntilACompleteFreshMeasurement)
{
    ASSERT_TRUE(bmp5xxDetect(&config, &baro, nullptr));
    int32_t pressure = 123, temperature = 456;
    EXPECT_EQ(BMP5XX_READ_PENDING, bmp5xxReadSample(&baro, &pressure, &temperature));
    EXPECT_EQ(123, pressure);
    EXPECT_EQ(456, temperature);
    mock.failedRead = 0x27;
    EXPECT_EQ(BMP5XX_READ_ERROR, bmp5xxReadSample(&baro, &pressure, &temperature));
    mock.failedRead = 0x1d;
    mock.registers[0x27] = {1};
    EXPECT_EQ(BMP5XX_READ_ERROR, bmp5xxReadSample(&baro, &pressure, &temperature));
    EXPECT_EQ(123, pressure);
    EXPECT_EQ(456, temperature);
    mock.failedRead = 0xff;
    mock.registers[0x27] = {1};
    EXPECT_EQ(BMP5XX_READ_READY, bmp5xxReadSample(&baro, &pressure, &temperature));
    EXPECT_EQ(100000, pressure);
    EXPECT_EQ(2400, temperature);
    EXPECT_EQ(BMP5XX_READ_PENDING, bmp5xxReadSample(&baro, &pressure, &temperature));
}
} // namespace

extern "C" {
void delay(timeMs_t)
{
}
bool busBusy(const extDevice_t *, bool *)
{
    return false;
}
void busDeviceRegister(const extDevice_t *)
{
}

bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (dev->bus->busType == BUS_TYPE_I2C && mock.i2cAddress && dev->busType_u.i2c.address != mock.i2cAddress) {
        return false;
    }
    if (reg == mock.failedRead) {
        return false;
    }
    if (dev->bus->busType == BUS_TYPE_SPI && !mock.spiSelected) {
        mock.spiSelected = true;
        ++mock.selectReads;
        memset(data, 0, length);
        return true;
    }
    if (reg == 0x38) {
        // OSR_CONFIG and OSR_EFF use the same P[5:3] / T[2:0] layout.
        // Derive the readback from the actual wire write, not driver constants
        // or a pre-seeded "correct" response. Validity is independently injectable.
        const auto osr = mock.registers.find(0x36);
        const uint8_t configured = osr == mock.registers.end() ? 0 : osr->second[0];
        const uint8_t effective = mock.effectiveOsrOverride >= 0 ? static_cast<uint8_t>(mock.effectiveOsrOverride)
                                                                 : static_cast<uint8_t>(0x80 | (configured & 0x3f));
        mock.registers[reg] = {effective};
    }
    const auto found = mock.registers.find(reg);
    const unsigned dummy = dev->bus->busType == BUS_TYPE_SPI ? 1 : 0;
    if (found == mock.registers.end() || found->second.size() + dummy != length) {
        return false;
    }
    if (reg == 0x1d) {
        ++mock.sampleReads;
    }
    if (dummy) {
        data[0] = 0;
    }
    memcpy(data + dummy, found->second.data(), found->second.size());
    if (reg == 0x27) {
        mock.registers[reg] = {0}; // Clear-on-read INT_STATUS.
    }
    return true;
}

bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t value)
{
    if (reg == mock.failedWrite) {
        return false;
    }
    if (reg == 0x7e && value == 0xb6) {
        ++mock.resetWrites;
        if (!mock.resetExecuted) {
            return false;
        }
        mock.registers[0x27] = {mock.resetPor};
        mock.registers[0x28] = {mock.resetNvm};
        if (dev->bus->busType == BUS_TYPE_SPI) {
            mock.spiSelected = false;
        }
        if (dev->bus->busType == BUS_TYPE_I2C && mock.resetNack) {
            return false;
        }
    }
    mock.registers[reg] = {value};
    return true;
}

IO_t IOGetByTag(ioTag_t)
{
    return IO_NONE;
}
void IOInit(IO_t, resourceOwner_e, uint8_t)
{
    ++mock.ioInitCalls;
}
void IOHi(IO_t)
{
    ++mock.ioHiCalls;
}
void IOConfigGPIO(IO_t, ioConfig_t)
{
    ++mock.ioConfigCalls;
}
void ioPreinitByIO(const IO_t, uint8_t, ioPreinitPinState_e)
{
    ++mock.ioPreinitCalls;
}
uint16_t spiCalculateDivider(uint32_t)
{
    return 2;
}
void spiSetClkDivisor(const extDevice_t *, uint16_t)
{
    ++mock.spiSetClkDivisorCalls;
}
void EXTIHandlerInit(extiCallbackRec_t *, extiHandlerCallback *)
{
}
void EXTIConfig(IO_t, extiCallbackRec_t *, int, ioConfig_t, extiTrigger_t)
{
}
void EXTIEnable(IO_t)
{
}
}
