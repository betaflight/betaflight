/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstring>

extern "C" {

#include "platform.h"

#include "build/build_config.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_lsm6dsv320x.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "sensors/gyro.h"

PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);

} // extern "C"

#include "gtest/gtest.h"

namespace {

constexpr uint8_t REG_IF_CFG = 0x03;
constexpr uint8_t REG_INT1_CTRL = 0x0D;
constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t REG_CTRL1 = 0x10;
constexpr uint8_t REG_CTRL2 = 0x11;
constexpr uint8_t REG_CTRL3 = 0x12;
constexpr uint8_t REG_CTRL4 = 0x13;
constexpr uint8_t REG_CTRL6 = 0x15;
constexpr uint8_t REG_CTRL7 = 0x16;
constexpr uint8_t REG_CTRL8 = 0x17;
constexpr uint8_t REG_CTRL9 = 0x18;
constexpr uint8_t REG_CTRL1_XL_HG = 0x4E;
constexpr uint8_t REG_HAODR_CFG = 0x62;

uint8_t mockRegisters[256];
uint32_t requestedSpiFrequency;
uint16_t appliedSpiDivider;
bool resetWasWritten;
bool sensorsWereOffBeforeReset;
bool mpuInitWasCalled;

class AccgyroSpiLsm6dsv320xTest : public testing::Test {
protected:
    void SetUp() override
    {
        std::memset(mockRegisters, 0, sizeof(mockRegisters));
        mockRegisters[REG_WHO_AM_I] = LSM6DSV320X_WHO_AM_I_CONST;
        requestedSpiFrequency = 0;
        appliedSpiDivider = 0;
        resetWasWritten = false;
        sensorsWereOffBeforeReset = false;
        mpuInitWasCalled = false;
        gyroConfigMutable()->gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
    }
};

TEST_F(AccgyroSpiLsm6dsv320xTest, DetectsExpectedDeviceId)
{
    extDevice_t dev = {};
    EXPECT_EQ(LSM6DSV320X_SPI, lsm6dsv320xSpiDetect(&dev));
}

TEST_F(AccgyroSpiLsm6dsv320xTest, RejectsUnexpectedDeviceId)
{
    extDevice_t dev = {};
    mockRegisters[REG_WHO_AM_I] = 0x70;
    EXPECT_EQ(MPU_NONE, lsm6dsv320xSpiDetect(&dev));
}

TEST_F(AccgyroSpiLsm6dsv320xTest, ConfiguresDatasheetRegisterValues)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = LSM6DSV320X_SPI;
    // Simulate an MCU-only reboot while all IMU sensing channels remain active.
    mockRegisters[REG_CTRL1] = 0x19;
    mockRegisters[REG_CTRL2] = 0x1C;
    mockRegisters[REG_CTRL1_XL_HG] = 0x38;

    ASSERT_TRUE(lsm6dsv320xSpiGyroDetect(&gyro));
    ASSERT_NE(nullptr, gyro.initFn);
    ASSERT_NE(nullptr, gyro.readFn);
    gyro.initFn(&gyro);

    EXPECT_EQ(10000000u, requestedSpiFrequency);
    EXPECT_EQ(8u, appliedSpiDivider);
    EXPECT_TRUE(resetWasWritten);
    EXPECT_TRUE(sensorsWereOffBeforeReset);
    EXPECT_TRUE(mpuInitWasCalled);

    EXPECT_EQ(0x01, mockRegisters[REG_IF_CFG]);
    EXPECT_EQ(0x44, mockRegisters[REG_CTRL3]);
    EXPECT_EQ(0x01, mockRegisters[REG_HAODR_CFG]);
    EXPECT_EQ(0x03, mockRegisters[REG_CTRL8]);
    // CTRL6 bit 3 must be one; FS_G=100 selects +/-2000 dps.
    EXPECT_EQ(0x0C, mockRegisters[REG_CTRL6]);
    EXPECT_EQ(0x01, mockRegisters[REG_CTRL7]);
    EXPECT_EQ(0x08, mockRegisters[REG_CTRL9]);
    EXPECT_EQ(0x02, mockRegisters[REG_CTRL4]);
    EXPECT_EQ(0x02, mockRegisters[REG_INT1_CTRL]);
    EXPECT_EQ(0x19, mockRegisters[REG_CTRL1]);
    EXPECT_EQ(0x1C, mockRegisters[REG_CTRL2]);
    EXPECT_FLOAT_EQ(0.070f, gyro.scale);
}

TEST_F(AccgyroSpiLsm6dsv320xTest, MapsAllHardwareLpfOptions)
{
    static constexpr struct {
        uint8_t option;
        uint8_t ctrl6;
    } cases[] = {
        { GYRO_HARDWARE_LPF_NORMAL, 0x0C },
        { GYRO_HARDWARE_LPF_OPTION_1, 0x2C },
        { GYRO_HARDWARE_LPF_OPTION_2, 0x1C },
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
        { GYRO_HARDWARE_LPF_EXPERIMENTAL, 0x3C },
#endif
    };

    for (const auto &testCase : cases) {
        gyroDev_t gyro = {};
        gyro.mpuDetectionResult.sensor = LSM6DSV320X_SPI;
        gyroConfigMutable()->gyro_hardware_lpf = testCase.option;

        ASSERT_TRUE(lsm6dsv320xSpiGyroDetect(&gyro));
        gyro.initFn(&gyro);

        EXPECT_EQ(testCase.ctrl6, mockRegisters[REG_CTRL6]);
    }
}

TEST_F(AccgyroSpiLsm6dsv320xTest, UsesLowGAccelerometerScale)
{
    accDev_t acc = {};
    acc.mpuDetectionResult.sensor = LSM6DSV320X_SPI;

    ASSERT_TRUE(lsm6dsv320xSpiAccDetect(&acc));
    ASSERT_NE(nullptr, acc.initFn);
    acc.initFn(&acc);

    EXPECT_EQ(2048u, acc.acc_1G);
}

TEST_F(AccgyroSpiLsm6dsv320xTest, RejectsMismatchedDetectionResults)
{
    gyroDev_t gyro = {};
    accDev_t acc = {};
    gyro.mpuDetectionResult.sensor = LSM6DSV16X_SPI;
    acc.mpuDetectionResult.sensor = LSM6DSV16X_SPI;

    EXPECT_FALSE(lsm6dsv320xSpiGyroDetect(&gyro));
    EXPECT_FALSE(lsm6dsv320xSpiAccDetect(&acc));
}

TEST_F(AccgyroSpiLsm6dsv320xTest, ReadsGyroAndLowGAccelerometerSamples)
{
    alignas(int16_t) uint8_t txBuffer[16] = {};
    alignas(int16_t) uint8_t rxBuffer[16] = {};
    gyroDev_t gyro = {};
    accDev_t acc = {};

    gyro.dev.txBuf = txBuffer;
    gyro.dev.rxBuf = rxBuffer;
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    gyro.mpuDetectionResult.sensor = LSM6DSV320X_SPI;
    ASSERT_TRUE(lsm6dsv320xSpiGyroDetect(&gyro));
    ASSERT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(0x1234, gyro.gyroADCRaw[X]);
    EXPECT_EQ(static_cast<int16_t>(0xFEDC), gyro.gyroADCRaw[Y]);
    EXPECT_EQ(static_cast<int16_t>(0x8001), gyro.gyroADCRaw[Z]);

    acc.gyro = &gyro;
    acc.mpuDetectionResult.sensor = LSM6DSV320X_SPI;
    ASSERT_TRUE(lsm6dsv320xSpiAccDetect(&acc));
    ASSERT_TRUE(acc.readFn(&acc));
    EXPECT_EQ(0x2345, acc.ADCRaw[X]);
    EXPECT_EQ(static_cast<int16_t>(0xEDCB), acc.ADCRaw[Y]);
    EXPECT_EQ(static_cast<int16_t>(0x9002), acc.ADCRaw[Z]);
}

TEST_F(AccgyroSpiLsm6dsv320xTest, ReadsCombinedDmaBurst)
{
    alignas(int16_t) uint8_t rxBuffer[16] = {
        0x00, 0x00,
        0x34, 0x12, 0xDC, 0xFE, 0x01, 0x80,
        0x45, 0x23, 0xCB, 0xED, 0x02, 0x90,
    };
    gyroDev_t gyro = {};
    accDev_t acc = {};

    gyro.dev.rxBuf = rxBuffer;
    gyro.gyroModeSPI = GYRO_EXTI_INT_DMA;
    gyro.mpuDetectionResult.sensor = LSM6DSV320X_SPI;
    ASSERT_TRUE(lsm6dsv320xSpiGyroDetect(&gyro));
    ASSERT_TRUE(gyro.readFn(&gyro));

    acc.gyro = &gyro;
    acc.mpuDetectionResult.sensor = LSM6DSV320X_SPI;
    ASSERT_TRUE(lsm6dsv320xSpiAccDetect(&acc));
    ASSERT_TRUE(acc.readFn(&acc));

    EXPECT_EQ(0x1234, gyro.gyroADCRaw[X]);
    EXPECT_EQ(static_cast<int16_t>(0xFEDC), gyro.gyroADCRaw[Y]);
    EXPECT_EQ(static_cast<int16_t>(0x8001), gyro.gyroADCRaw[Z]);
    EXPECT_EQ(0x2345, acc.ADCRaw[X]);
    EXPECT_EQ(static_cast<int16_t>(0xEDCB), acc.ADCRaw[Y]);
    EXPECT_EQ(static_cast<int16_t>(0x9002), acc.ADCRaw[Z]);
}

} // namespace

extern "C" {

void delay(uint32_t) {}

uint16_t spiCalculateDivider(uint32_t frequency)
{
    requestedSpiFrequency = frequency;
    return 8;
}

void spiSetClkDivisor(const extDevice_t *, uint16_t divider)
{
    appliedSpiDivider = divider;
}

void spiWriteReg(const extDevice_t *, uint8_t reg, uint8_t value)
{
    mockRegisters[reg] = value;
    if (reg == REG_CTRL3 && (value & 0x01)) {
        resetWasWritten = true;
        sensorsWereOffBeforeReset = mockRegisters[REG_CTRL1] == 0
            && mockRegisters[REG_CTRL2] == 0
            && mockRegisters[REG_CTRL1_XL_HG] == 0;
        mockRegisters[reg] &= ~0x01;
    }
}

uint8_t spiReadRegMsk(const extDevice_t *, uint8_t reg)
{
    return mockRegisters[reg];
}

void spiSequence(const extDevice_t *, busSegment_t *segments)
{
    uint8_t *rx = segments[0].u.buffers.rxData;
    const uint8_t command = segments[0].u.buffers.txData[0];

    rx[0] = 0;
    if (command == (0x22 | 0x80)) {
        const uint8_t sample[] = { 0x34, 0x12, 0xDC, 0xFE, 0x01, 0x80 };
        std::memcpy(&rx[1], sample, sizeof(sample));
    } else if (command == (0x28 | 0x80)) {
        const uint8_t sample[] = { 0x45, 0x23, 0xCB, 0xED, 0x02, 0x90 };
        std::memcpy(&rx[1], sample, sizeof(sample));
    }
}

void spiWait(const extDevice_t *) {}

void mpuGyroInit(gyroDev_t *)
{
    mpuInitWasCalled = true;
}

busStatus_e mpuIntCallback(uintptr_t)
{
    return BUS_READY;
}

} // extern "C"
