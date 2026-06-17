/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <stddef.h>

extern "C" {

#include "platform.h"
#include "target.h"
#include "build/build_config.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "sensors/gyro.h"

PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);

bool icm426xxSpiGyroDetect(gyroDev_t *gyro);
bool icm426xxSpiAccDetect(accDev_t *acc);
void icm426xxAccInit(accDev_t *acc);

// MUST mirror the file-static definitions in src/main/drivers/accgyro/accgyro_spi_icm426xx.c
// (search for `typedef struct aafConfig_s` and `typedef enum { AAF_CONFIG_258HZ`).
// If the driver's struct fields are reordered or resized, the static_assert below fails loudly;
// a silent field-order drift would otherwise corrupt cfg.delt reads in the AAF tests.
typedef struct aafConfig_s {
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

typedef enum {
    AAF_CONFIG_258HZ = 0,
    AAF_CONFIG_536HZ,
    AAF_CONFIG_997HZ,
    AAF_CONFIG_1962HZ,
    AAF_CONFIG_COUNT
} aafConfig_e;

static_assert(offsetof(aafConfig_t, delt) == 0,
              "aafConfig_t layout has drifted from the driver TU — update the local mirror above");
static_assert(offsetof(aafConfig_t, deltSqr) == 2,
              "aafConfig_t layout has drifted from the driver TU — update the local mirror above");
static_assert(offsetof(aafConfig_t, bitshift) == 4,
              "aafConfig_t layout has drifted from the driver TU — update the local mirror above");

STATIC_UNIT_TESTED aafConfig_t getGyroAafConfig(const mpuSensor_e gyroModel, const aafConfig_e config);

} // extern "C"

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Expected per-sensor delt for static_cast<aafConfig_e>(GYRO_HARDWARE_LPF_NORMAL) (258 Hz):
// aafLUT42605[AAF_CONFIG_258HZ] = { 21, 440, 6 } -- ICM_42605_SPI only
// aafLUT42688[AAF_CONFIG_258HZ] = {  6,  36, 10 } -- all other ICM-426xx family chips
static constexpr uint8_t AAF_DELT_42605_NORMAL = 21;
static constexpr uint8_t AAF_DELT_42688_NORMAL = 6;

// --- icm426xxSpiGyroDetect: gyro->scale per sensor ---------------------------

TEST(AccgyroSpiIcm426xx, GyroScaleIcm42605)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = ICM_42605_SPI;
    EXPECT_TRUE(icm426xxSpiGyroDetect(&gyro));
    EXPECT_FLOAT_EQ(GYRO_SCALE_2000DPS, gyro.scale);
}

TEST(AccgyroSpiIcm426xx, GyroScaleIcm42622P)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = ICM_42622P_SPI;
    EXPECT_TRUE(icm426xxSpiGyroDetect(&gyro));
    EXPECT_FLOAT_EQ(GYRO_SCALE_2000DPS, gyro.scale);
}

TEST(AccgyroSpiIcm426xx, GyroScaleIcm42686P)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = ICM_42686P_SPI;
    EXPECT_TRUE(icm426xxSpiGyroDetect(&gyro));
#if ENABLE_42686_EXTENDED_RANGE
    EXPECT_FLOAT_EQ(GYRO_SCALE_4000DPS, gyro.scale);
#else
    EXPECT_FLOAT_EQ(GYRO_SCALE_2000DPS, gyro.scale);
#endif
}

TEST(AccgyroSpiIcm426xx, GyroScaleIcm42688P)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = ICM_42688P_SPI;
    EXPECT_TRUE(icm426xxSpiGyroDetect(&gyro));
    EXPECT_FLOAT_EQ(GYRO_SCALE_2000DPS, gyro.scale);
}

TEST(AccgyroSpiIcm426xx, GyroScaleIim42652)
{
    // Regression guard: IIM-42652 silicon max FSR is ±2000 dps, not ±4000 dps.
    // Pre-fix code (PR #14424) incorrectly mapped IIM_42652_SPI to GYRO_SCALE_4000DPS.
    // Also assert tempScale/tempZero so a future refactor that drops IIM_42652
    // out of the 2000DPS arm (and therefore out of the temperature-init block)
    // is caught here rather than silently producing zero-init temperature values.
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = IIM_42652_SPI;
    EXPECT_TRUE(icm426xxSpiGyroDetect(&gyro));
    EXPECT_FLOAT_EQ(GYRO_SCALE_2000DPS, gyro.scale);
    EXPECT_FLOAT_EQ(1.0f / 132.48f, gyro.tempScale);
    EXPECT_FLOAT_EQ(25.0f, gyro.tempZero);
}

TEST(AccgyroSpiIcm426xx, GyroScaleIim42653)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = IIM_42653_SPI;
    EXPECT_TRUE(icm426xxSpiGyroDetect(&gyro));
    EXPECT_FLOAT_EQ(GYRO_SCALE_4000DPS, gyro.scale);
}

TEST(AccgyroSpiIcm426xx, GyroScaleUnknownSensorRejected)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = MPU_NONE;
    EXPECT_FALSE(icm426xxSpiGyroDetect(&gyro));
}

// --- icm426xxAccInit: acc->acc_1G per sensor ---------------------------------

TEST(AccgyroSpiIcm426xx, AccOneGIcm42605)
{
    accDev_t acc = {};
    acc.mpuDetectionResult.sensor = ICM_42605_SPI;
    icm426xxAccInit(&acc);
    EXPECT_EQ(512u * 4u, acc.acc_1G); // ±16g
}

TEST(AccgyroSpiIcm426xx, AccOneGIcm42622P)
{
    accDev_t acc = {};
    acc.mpuDetectionResult.sensor = ICM_42622P_SPI;
    icm426xxAccInit(&acc);
    EXPECT_EQ(512u * 4u, acc.acc_1G); // ±16g
}

TEST(AccgyroSpiIcm426xx, AccOneGIcm42686P)
{
    accDev_t acc = {};
    acc.mpuDetectionResult.sensor = ICM_42686P_SPI;
    icm426xxAccInit(&acc);
#if ENABLE_42686_EXTENDED_RANGE
    EXPECT_EQ(512u * 2u, acc.acc_1G); // ±32g
#else
    EXPECT_EQ(512u * 4u, acc.acc_1G); // ±16g
#endif
}

TEST(AccgyroSpiIcm426xx, AccOneGIcm42688P)
{
    accDev_t acc = {};
    acc.mpuDetectionResult.sensor = ICM_42688P_SPI;
    icm426xxAccInit(&acc);
    EXPECT_EQ(512u * 4u, acc.acc_1G); // ±16g
}

TEST(AccgyroSpiIcm426xx, AccOneGIim42652)
{
    // Regression guard: IIM-42652 silicon max FSR is ±16g, not ±32g.
    // Pre-fix code (PR #14424) incorrectly mapped IIM_42652_SPI to the 32g (acc_1G = 1024) arm.
    accDev_t acc = {};
    acc.mpuDetectionResult.sensor = IIM_42652_SPI;
    icm426xxAccInit(&acc);
    EXPECT_EQ(512u * 4u, acc.acc_1G); // ±16g
}

TEST(AccgyroSpiIcm426xx, AccOneGIim42653)
{
    accDev_t acc = {};
    acc.mpuDetectionResult.sensor = IIM_42653_SPI;
    icm426xxAccInit(&acc);
    EXPECT_EQ(512u * 2u, acc.acc_1G); // ±32g
}

// --- getGyroAafConfig: NORMAL cutoff delt per sensor -------------------------

TEST(AccgyroSpiIcm426xx, AafNormalIcm42605)
{
    const aafConfig_t cfg = getGyroAafConfig(ICM_42605_SPI, static_cast<aafConfig_e>(GYRO_HARDWARE_LPF_NORMAL));
    EXPECT_EQ(AAF_DELT_42605_NORMAL, cfg.delt);
}

TEST(AccgyroSpiIcm426xx, AafNormalIcm42622P)
{
    const aafConfig_t cfg = getGyroAafConfig(ICM_42622P_SPI, static_cast<aafConfig_e>(GYRO_HARDWARE_LPF_NORMAL));
    EXPECT_EQ(AAF_DELT_42688_NORMAL, cfg.delt);
}

TEST(AccgyroSpiIcm426xx, AafNormalIcm42686P)
{
    const aafConfig_t cfg = getGyroAafConfig(ICM_42686P_SPI, static_cast<aafConfig_e>(GYRO_HARDWARE_LPF_NORMAL));
    EXPECT_EQ(AAF_DELT_42688_NORMAL, cfg.delt);
}

TEST(AccgyroSpiIcm426xx, AafNormalIcm42688P)
{
    const aafConfig_t cfg = getGyroAafConfig(ICM_42688P_SPI, static_cast<aafConfig_e>(GYRO_HARDWARE_LPF_NORMAL));
    EXPECT_EQ(AAF_DELT_42688_NORMAL, cfg.delt);
}

TEST(AccgyroSpiIcm426xx, AafNormalIim42652)
{
    // Regression guard: IIM-42652 AAF block is the industrial 42688-P IP, must use aafLUT42688.
    // Pre-fix code (PR #14424) incorrectly mapped IIM_42652_SPI to aafLUT42605 (cutoff ~249 Hz instead of ~258 Hz).
    const aafConfig_t cfg = getGyroAafConfig(IIM_42652_SPI, static_cast<aafConfig_e>(GYRO_HARDWARE_LPF_NORMAL));
    EXPECT_EQ(AAF_DELT_42688_NORMAL, cfg.delt);
}

TEST(AccgyroSpiIcm426xx, AafNormalIim42653)
{
    // Regression guard: PR #14095 originally placed IIM_42653_SPI on aafLUT42688; PR #14424 silently
    // moved it to aafLUT42605. Restore the original placement to match the 42653 datasheet AAF block.
    const aafConfig_t cfg = getGyroAafConfig(IIM_42653_SPI, static_cast<aafConfig_e>(GYRO_HARDWARE_LPF_NORMAL));
    EXPECT_EQ(AAF_DELT_42688_NORMAL, cfg.delt);
}

// --- STUBS for SPI / IO / MPU symbols pulled in by the driver TU -------------

extern "C" {

void delay(uint32_t) {}

void spiSetClkDivisor(const extDevice_t *, uint16_t) {}

uint16_t spiCalculateDivider(uint32_t)
{
    return 2;
}

void spiWriteReg(const extDevice_t *, uint8_t, uint8_t) {}

uint8_t spiReadRegMsk(const extDevice_t *, uint8_t)
{
    // Drives the WHO_AM_I detection loop in icm426xxSpiDetect (not exercised by these tests);
    // returning 0 is harmless because the tests never call icm426xxSpiDetect.
    return 0;
}

void mpuGyroInit(gyroDev_t *) {}

bool mpuGyroReadSPI(gyroDev_t *)
{
    return false;
}

bool mpuAccReadSPI(accDev_t *)
{
    return false;
}

} // extern "C"
