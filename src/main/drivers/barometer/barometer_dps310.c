/*
 * This file is part of Cleanflight, Betaflight and INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * Copyright: INAVFLIGHT OU
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/time.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_dps310.h"
#include "drivers/resource.h"

// 10 MHz max SPI frequency
#define DPS310_MAX_SPI_CLK_HZ 10000000

#if defined(USE_BARO) && defined(USE_BARO_DPS310)

#define DPS310_I2C_ADDR             0x76

#define DPS310_REG_PSR_B2           0x00
#define DPS310_REG_PSR_B1           0x01
#define DPS310_REG_PSR_B0           0x02
#define DPS310_REG_TMP_B2           0x03
#define DPS310_REG_TMP_B1           0x04
#define DPS310_REG_TMP_B0           0x05
#define DPS310_REG_PRS_CFG          0x06
#define DPS310_REG_TMP_CFG          0x07
#define DPS310_REG_MEAS_CFG         0x08
#define DPS310_REG_CFG_REG          0x09

#define DPS310_REG_RESET            0x0C
#define DPS310_REG_ID               0x0D

#define DPS310_REG_COEF             0x10
#define DPS310_REG_COEF_SRCE        0x28


#define DPS310_ID_REV_AND_PROD_ID       (0x10)

#define DPS310_RESET_BIT_SOFT_RST       (0x09)    // 0b1001

#define DPS310_MEAS_CFG_COEF_RDY        (1 << 7)
#define DPS310_MEAS_CFG_SENSOR_RDY      (1 << 6)
#define DPS310_MEAS_CFG_TMP_RDY         (1 << 5)
#define DPS310_MEAS_CFG_PRS_RDY         (1 << 4)
#define DPS310_MEAS_CFG_MEAS_CTRL_CONT  (0x7)

#define DPS310_PRS_CFG_BIT_PM_RATE_32HZ (0x50)      //  101 - 32 measurements pr. sec.
#define DPS310_PRS_CFG_BIT_PM_PRC_16    (0x04)      // 0100 - 16 times (Standard).

#define DPS310_TMP_CFG_BIT_TMP_EXT          (0x80)  //
#define DPS310_TMP_CFG_BIT_TMP_RATE_32HZ    (0x50)  //  101 - 32 measurements pr. sec.
#define DPS310_TMP_CFG_BIT_TMP_PRC_16       (0x04)  // 0100 - 16 times (Standard).

#define DPS310_CFG_REG_BIT_P_SHIFT          (0x04)
#define DPS310_CFG_REG_BIT_T_SHIFT          (0x08)

#define DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE  (0x80)

typedef struct {
    int16_t c0;     // 12bit
    int16_t c1;     // 12bit
    int32_t c00;    // 20bit
    int32_t c10;    // 20bit
    int16_t c01;    // 16bit
    int16_t c11;    // 16bit
    int16_t c20;    // 16bit
    int16_t c21;    // 16bit
    int16_t c30;    // 16bit
} calibrationCoefficients_t;

typedef struct {
    calibrationCoefficients_t   calib;
    float                       pressure;       // Pa
    float                       temperature;    // DegC
} baroState_t;

static baroState_t  baroState;

#define busReadBuf busReadRegisterBuffer
#define busWrite   busWriteRegister

// Helper functions
static uint8_t registerRead(busDevice_t * busDev, uint8_t reg)
{
    return busReadRegister(busDev, reg);
}

static void registerWrite(busDevice_t * busDev, uint8_t reg, uint8_t value)
{
    busWrite(busDev, reg, value);
}

static void registerSetBits(busDevice_t * busDev, uint8_t reg, uint8_t setbits)
{
    uint8_t val = registerRead(busDev, reg);

    if ((val & setbits) != setbits) {
        val |= setbits;
        registerWrite(busDev, reg, val);
    }
}

static int32_t getTwosComplement(uint32_t raw, uint8_t length)
{
    if (raw & ((int)1 << (length - 1))) {
        return ((int32_t)raw) - ((int32_t)1 << length);
    }
    else {
        return raw;
    }
}

static bool deviceConfigure(busDevice_t * busDev)
{
    // Trigger a chip reset
    registerSetBits(busDev, DPS310_REG_RESET, DPS310_RESET_BIT_SOFT_RST);

    // Sleep 40ms
    delay(40);

    uint8_t status = registerRead(busDev, DPS310_REG_MEAS_CFG);

    // Check if coefficients are available
    if ((status & DPS310_MEAS_CFG_COEF_RDY) == 0) {
        return false;
    }

    // Check if sensor initialization is complete
    if ((status & DPS310_MEAS_CFG_SENSOR_RDY) == 0) {
        return false;
    }

    // 1. Read the pressure calibration coefficients (c00, c10, c20, c30, c01, c11, and c21) from the Calibration Coefficient register.
    //   Note: The coefficients read from the coefficient register are 2's complement numbers.
    uint8_t coef[18];
    if (!busReadBuf(busDev, DPS310_REG_COEF, coef, sizeof(coef))) {
        return false;
    }

    // 0x11 c0 [3:0] + 0x10 c0 [11:4]
    baroState.calib.c0 = getTwosComplement(((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F), 12);

    // 0x11 c1 [11:8] + 0x12 c1 [7:0]
    baroState.calib.c1 = getTwosComplement((((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2], 12);

    // 0x13 c00 [19:12] + 0x14 c00 [11:4] + 0x15 c00 [3:0]
    baroState.calib.c00 = getTwosComplement(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F), 20);

    // 0x15 c10 [19:16] + 0x16 c10 [15:8] + 0x17 c10 [7:0]
    baroState.calib.c10 = getTwosComplement((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7], 20);

    // 0x18 c01 [15:8] + 0x19 c01 [7:0]
    baroState.calib.c01 = getTwosComplement(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);

    // 0x1A c11 [15:8] + 0x1B c11 [7:0]
    baroState.calib.c11 = getTwosComplement(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);

    // 0x1C c20 [15:8] + 0x1D c20 [7:0]
    baroState.calib.c20 = getTwosComplement(((uint32_t)coef[12] << 8) | (uint32_t)coef[13], 16);

    // 0x1E c21 [15:8] + 0x1F c21 [7:0]
    baroState.calib.c21 = getTwosComplement(((uint32_t)coef[14] << 8) | (uint32_t)coef[15], 16);

    // 0x20 c30 [15:8] + 0x21 c30 [7:0]
    baroState.calib.c30 = getTwosComplement(((uint32_t)coef[16] << 8) | (uint32_t)coef[17], 16);

    // PRS_CFG: pressure measurement rate (32 Hz) and oversampling (16 time standard)
    registerSetBits(busDev, DPS310_REG_PRS_CFG, DPS310_PRS_CFG_BIT_PM_RATE_32HZ | DPS310_PRS_CFG_BIT_PM_PRC_16);

    // TMP_CFG: temperature measurement rate (32 Hz) and oversampling (16 times)
    const uint8_t TMP_COEF_SRCE = registerRead(busDev, DPS310_REG_COEF_SRCE) & DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE;
    registerSetBits(busDev, DPS310_REG_TMP_CFG, DPS310_TMP_CFG_BIT_TMP_RATE_32HZ | DPS310_TMP_CFG_BIT_TMP_PRC_16 | TMP_COEF_SRCE);

    // CFG_REG: set pressure and temperature result bit-shift (required when the oversampling rate is >8 times)
    registerSetBits(busDev, DPS310_REG_CFG_REG, DPS310_CFG_REG_BIT_T_SHIFT | DPS310_CFG_REG_BIT_P_SHIFT);

    // MEAS_CFG: Continuous pressure and temperature measurement
    registerSetBits(busDev, DPS310_REG_MEAS_CFG, DPS310_MEAS_CFG_MEAS_CTRL_CONT);

    return true;
}

static bool deviceReadMeasurement(baroDev_t *baro)
{
    // 1. Check if pressure is ready
    bool pressure_ready = registerRead(&baro->busdev, DPS310_REG_MEAS_CFG) & DPS310_MEAS_CFG_PRS_RDY;
    if (!pressure_ready) {
        return false;
    }

    // 2. Choose scaling factors kT (for temperature) and kP (for pressure) based on the chosen precision rate.
    // The scaling factors are listed in Table 9.
    static float kT = 253952; // 16 times (Standard)
    static float kP = 253952; // 16 times (Standard)

    // 3. Read the pressure and temperature result from the registers
    // Read PSR_B2, PSR_B1, PSR_B0, TMP_B2, TMP_B1, TMP_B0
    uint8_t buf[6];
    if (!busReadBuf(&baro->busdev, DPS310_REG_PSR_B2, buf, 6)) {
        return false;
    }

    const int32_t Praw = getTwosComplement((buf[0] << 16) + (buf[1] << 8) + buf[2], 24);
    const int32_t Traw = getTwosComplement((buf[3] << 16) + (buf[4] << 8) + buf[5], 24);

    // 4. Calculate scaled measurement results.
    const float Praw_sc = Praw / kP;
    const float Traw_sc = Traw / kT;

    // 5. Calculate compensated measurement results.
    const float c00 = baroState.calib.c00;
    const float c01 = baroState.calib.c01;
    const float c10 = baroState.calib.c10;
    const float c11 = baroState.calib.c11;
    const float c20 = baroState.calib.c20;
    const float c21 = baroState.calib.c21;
    const float c30 = baroState.calib.c30;

    const float c0 = baroState.calib.c0;
    const float c1 = baroState.calib.c1;

    baroState.pressure = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21);
    baroState.temperature = c0 * 0.5f + c1 * Traw_sc;

    return true;
}

static void deviceCalculate(int32_t *pressure, int32_t *temperature)
{
    if (pressure) {
        *pressure = baroState.pressure; 
    }

    if (temperature) {
        *temperature = (baroState.temperature * 100);   // to centidegrees
    }
}



#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(busDevice_t * busDev)
{
    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        uint8_t chipId[1];

        delay(100);

        bool ack = busReadBuf(busDev, DPS310_REG_ID, chipId, 1);

        if (ack && chipId[0] == DPS310_ID_REV_AND_PROD_ID) {
            return true;
        }
    };

    return false;
}

static void dps310StartUT(baroDev_t *baro)
{
    UNUSED(baro);
}

static bool dps310ReadUT(baroDev_t *baro)
{
    UNUSED(baro);

    return true;
}

static bool dps310GetUT(baroDev_t *baro)
{
    UNUSED(baro);

    return true;
}

static void dps310StartUP(baroDev_t *baro)
{
    UNUSED(baro);
}

static bool dps310ReadUP(baroDev_t *baro)
{
    UNUSED(baro);

    return true;
}

static void busDeviceInit(busDevice_t *busdev, resourceOwner_e owner)
{
#ifdef USE_BARO_SPI_DPS310
    if (busdev->bustype == BUSTYPE_SPI) {
        IOHi(busdev->busdev_u.spi.csnPin); // Disable
        IOInit(busdev->busdev_u.spi.csnPin, owner, 0);
        IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_OUT_PP);
#ifdef USE_SPI_TRANSACTION
        spiBusTransactionInit(busdev, SPI_MODE0_POL_LOW_EDGE_1ST, spiCalculateDivider(DPS310_MAX_SPI_CLK_HZ)); // DPS310 supports Mode 0 or 3
#else
        spiBusSetDivisor(busdev, spiCalculateDivider(DPS310_MAX_SPI_CLK_HZ));
#endif
    }
#else
    UNUSED(busdev);
    UNUSED(owner);
#endif
}

static void busDeviceDeInit(busDevice_t *busdev)
{
#ifdef USE_BARO_SPI_DPS310
    if (busdev->bustype == BUSTYPE_SPI) {
        spiPreinitByIO(busdev->busdev_u.spi.csnPin);
    }
#else
    UNUSED(busdev);
#endif
}

bool baroDPS310Detect(baroDev_t *baro)
{
    busDevice_t *busdev = &baro->busdev;
    bool defaultAddressApplied = false;

    busDeviceInit(&baro->busdev, OWNER_BARO_CS);

    if ((busdev->bustype == BUSTYPE_I2C) && (busdev->busdev_u.i2c.address == 0)) {
        // Default address for BMP280
        busdev->busdev_u.i2c.address = DPS310_I2C_ADDR;
        defaultAddressApplied = true;
    }

    if (!deviceDetect(busdev)) {
        busDeviceDeInit(busdev);
        if (defaultAddressApplied) {
            busdev->busdev_u.i2c.address = 0;
        }
        return false;
    }

    if (!deviceConfigure(busdev)) {
        busDeviceDeInit(busdev);
        return false;
    }

    busDeviceRegister(busdev);

    const uint32_t baroDelay = 1000000 / 32 / 2;      // twice the sample rate to capture all new data

    baro->ut_delay = 0;
    baro->start_ut = dps310StartUT;
    baro->read_ut = dps310ReadUT;
    baro->get_ut = dps310GetUT;

    baro->up_delay = baroDelay;
    baro->start_up = dps310StartUP;
    baro->read_up = dps310ReadUP;
    baro->get_up = deviceReadMeasurement;

    baro->calculate = deviceCalculate;

    return true;
}

#endif
