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
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/time.h"
#include "drivers/barometer/barometer.h"
#include "drivers/resource.h"

#include "barometer_2smpb_02b.h"

// 10 MHz max SPI frequency
#define BARO_2SMBP_MAX_SPI_CLK_HZ 10000000

#if defined(USE_BARO) && defined(USE_BARO_2SMBP_02B)

#define BARO_2SMBP_I2C_ADDRESS 0x70

#define BARO_2SMBP_CHIP_ID 0x5C

#define REG_CHIP_ID 0xD1
#define REG_RESET 0xE0

#define REG_COE_PR11 0xA0
#define REG_COE_PR21 0xA3
#define REG_COE_PR31 0xA5
#define REG_COE_TEMP11 0xA7
#define REG_COE_TEMP21 0xA9
#define REG_COE_TEMP31 0xAB
#define REG_COE_PTAT11 0xAD
#define REG_COE_PTAT21 0xB1
#define REG_COE_PTAT31 0xB3

#define REG_IIR_CNT 0xF1
#define REG_DEVICE_STAT 0xF3
#define REG_CTRL_MEAS 0xF4
#define REG_IO_SETUP 0xF5
#define REG_PRESS_TXD2 0xF7

// Value for CTRL_MEAS with 4x temperature averaging, 32x perssure, forced mode
#define REG_CLT_MEAS_VAL_TAVG4X_PAVG32X_FORCED ((0x03 << 5) | (0x05 << 2) | 0x01)

// IIR coefficient setting 8x
#define REG_IIR_CNT_VAL_8X 0x03

typedef struct {
    float aa;
    float ba;
    int32_t ca;
    float ap;
    float bp;
    int32_t cp;
    float at;
    float bt;
    float ct;
} calibrationCoefficients_t;

typedef struct {
    calibrationCoefficients_t   calib;
    float                       pressure;       // Pa
    float                       temperature;    // DegC
} baroState_t;

static baroState_t  baroState;
static uint8_t baroDataBuf[6];


static int32_t readSignedRegister(const extDevice_t *dev, uint8_t reg, uint8_t nBytes)
{
    uint8_t buf[3];
    uint32_t rawValue = 0;

    busReadRegisterBuffer(dev, reg, &buf[0], nBytes);

    for (int i=0; i<nBytes; i++) {
        rawValue += (uint32_t)buf[i] << (8 * (nBytes - i - 1));
    }

    // 2's complement
    if (rawValue & ((int32_t)1 << (8 * nBytes - 1))) {
        // Negative
        return ((int32_t)rawValue) - ((int32_t)1 << (8 * nBytes));
    }
    else {
        return rawValue;
    }
}

static int32_t getSigned24bitValue(uint8_t * pData)
{
    uint32_t raw;

    raw = (((uint32_t)pData[0] << 16) | ((uint32_t)pData[1] << 8) | (uint32_t)pData[2]) - ((uint32_t)1 << 23);

    return raw;
}

static bool deviceConfigure(const extDevice_t *dev)
{
    /** Note: Chip reset causes I2C error due missing ACK. This causes interrupt based read (busReadRegisterBufferStart)
        to not work (read stops due to error flags). It works fine without chip reset. **/

    //busWriteRegister(busDev, REG_RESET, 0xE6);

    // No need to write IO_SETUP register: default values are fine

    // Read calibration coefficients and scale them
    baroState.calib.aa = (4.2e-4f  * readSignedRegister(dev, REG_COE_PTAT31, 2)) / 32767;
    baroState.calib.ba = (8.0e0f  * readSignedRegister(dev, REG_COE_PTAT21, 2)) / 32767 - 1.6e2f;
    baroState.calib.ca = readSignedRegister(dev, REG_COE_PTAT11, 3);
    baroState.calib.ap = (3.0e-5f  * readSignedRegister(dev, REG_COE_PR31, 2)) / 32767;
    baroState.calib.bp = (10 * readSignedRegister(dev, REG_COE_PR21, 2)) / 32767 + 3.0e1f;
    baroState.calib.cp = readSignedRegister(dev, REG_COE_PR11, 3);
    baroState.calib.at = (8.0e-11f  * readSignedRegister(dev, REG_COE_TEMP31, 2)) / 32767;
    baroState.calib.bt = (1.6e-6f  * readSignedRegister(dev, REG_COE_TEMP21, 2)) / 32767 - 6.6e-6f;
    baroState.calib.ct = (8.5e-3f  * readSignedRegister(dev, REG_COE_TEMP11, 2)) / 32767 + 4.0e-2f;

    // Configure IIR filter
    busWriteRegister(dev, REG_IIR_CNT, REG_IIR_CNT_VAL_8X);

    return true;
}

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(const extDevice_t *dev)
{
    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        uint8_t chipId;

        chipId = busReadRegister(dev, REG_CHIP_ID);

        if (chipId == BARO_2SMBP_CHIP_ID) {
            return true;
        }

        delay(50);
    };

    return false;
}

static void deviceInit(const extDevice_t *dev, resourceOwner_e owner)
{
#ifdef USE_BARO_SPI_2SMBP_02B
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, owner, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(BARO_2SMBP_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
    UNUSED(owner);
#endif
}

static void busDeviceDeInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_2SMBP_02B
    if (dev->bus->busType == BUS_TYPE_SPI) {
        spiPreinitByIO(dev->busType_u.spi.csnPin);
    }
#else
    UNUSED(dev);
#endif
}

static void b2smpbStartUP(baroDev_t *baro)
{
    // start a forced measurement
    busWriteRegister(&baro->dev, REG_CTRL_MEAS, REG_CLT_MEAS_VAL_TAVG4X_PAVG32X_FORCED);
}

static bool b2smpbReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // Start reading temperature and pressure data
    busReadRegisterBufferStart(&baro->dev, REG_PRESS_TXD2, &baroDataBuf[0], 6);

    return true;
}

static bool b2smpbGetUP(baroDev_t *baro)
{
    int32_t dtp;
    float tr, pl, tmp;

    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // Calculate compensated temperature
    dtp = getSigned24bitValue(&baroDataBuf[3]);

    tmp = baroState.calib.ba * baroState.calib.ba;
    tr = (-1 * baroState.calib.ba - sqrtf(tmp - 4 * baroState.calib.aa * (baroState.calib.ca - dtp))) / (2 * baroState.calib.aa);
    baroState.temperature = tr / 256;

    // Calculate raw pressure
    dtp = getSigned24bitValue(&baroDataBuf[0]);

    tmp = baroState.calib.bp * baroState.calib.bp;
    pl = (sqrtf(tmp - 4 * baroState.calib.ap * (baroState.calib.cp - dtp)) - baroState.calib.bp) / (2 * baroState.calib.ap);

    // Calculate temperature compensated pressure
    tmp = tr * tr;
    baroState.pressure = pl / (baroState.calib.at * tmp + baroState.calib.bt * tr + baroState.calib.ct + 1);

    return true;
}

static void b2smpbStartUT(baroDev_t *baro)
{
    UNUSED(baro);
}

static bool b2smpbReadUT(baroDev_t *baro)
{
    UNUSED(baro);

    return true;
}

static bool b2smpbGetUT(baroDev_t *baro)
{
    UNUSED(baro);

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

bool baro2SMPB02BDetect(baroDev_t *baro)
{
    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    deviceInit(&baro->dev, OWNER_BARO_CS);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        dev->busType_u.i2c.address = BARO_2SMBP_I2C_ADDRESS;
        defaultAddressApplied = true;
    }

    if (!deviceDetect(dev)) {
        busDeviceDeInit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    if (!deviceConfigure(dev)) {
        busDeviceDeInit(dev);
        return false;
    }

    baro->up_delay = 35000; // measurement takes 33.7 ms with 4x / 32x averaging
    baro->start_up = b2smpbStartUP;
    baro->read_up = b2smpbReadUP;
    baro->get_up = b2smpbGetUP;

    // these are dummies, temperature is read with pressure
    baro->ut_delay = 0;
    baro->start_ut = b2smpbStartUT;
    baro->read_ut = b2smpbReadUT;
    baro->get_ut = b2smpbGetUT;

    baro->calculate = deviceCalculate;

    return true;
}

#endif
