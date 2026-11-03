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
 *
 * AKM AK09916 magnetometer driver (I2C only).
 *
 * Note: the part number is AK09916; the driver, symbols and settings follow the
 * Betaflight convention of dropping the leading zero (AK09916 -> AK9916), as
 * done for the other AKxxxx drivers.
 *
 * The AK09916 is a 16 bit magnetometer with a fixed sensitivity of 0.15 uT/LSB
 * and (unlike the AK8963/AK8975) has no fuse ROM sensitivity adjustment values,
 * so magGain[] is deliberately left untouched.
 *
 * It is typically the magnetometer of an ICM-20948, e.g. inside a Here2/Here3
 * GPS where the module's own MCU bridges the aux bus to its I2C interface, so
 * the chip appears as an ordinary I2C device at 0x0C.
 *
 * Note that register 0x00 (WIA1, company ID) is 0x48, i.e. the same byte the
 * AK8975/AK8963 drivers accept as their WHO_AM_I, and all three live at address
 * 0x0C. Detection therefore uses WIA2 (0x01, device id 0x09) and the probe order
 * in sensors/compass.c places this driver before the AK8975/AK8963 cases.
 *
 * Reference:
 *   https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Compass/AP_Compass_AK09916.cpp
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_AK9916

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_ak9916.h"

#define AK9916_MAG_I2C_ADDRESS         0x0C

// Registers
#define AK9916_REG_WIA1                0x00 // company ID, 0x48
#define AK9916_REG_WIA2                0x01 // device ID, 0x09 on the AK9916
#define AK9916_REG_ST1                 0x10 // bit0 DRDY, bit1 DOR
#define AK9916_REG_HXL                 0x11 // X/Y/Z, little endian 16 bit, 0x11..0x16
#define AK9916_REG_ST2                 0x18 // bit3 HOFL (magnetic overflow)
#define AK9916_REG_CNTL2               0x31 // measurement mode
#define AK9916_REG_CNTL3               0x32 // bit0 SRST (soft reset)

#define AK9916_DEVICE_ID               0x09

#define AK9916_ST1_DATA_READY          0x01
#define AK9916_ST1_DATA_OVERRUN        0x02

#define AK9916_ST2_MAG_SENSOR_OVERFLOW 0x08

#define AK9916_CNTL3_SOFT_RESET        0x01

// CNTL2 measurement modes
#define AK9916_MODE_POWER_DOWN         0x00
#define AK9916_MODE_CONT_10_HZ         0x02
#define AK9916_MODE_CONT_20_HZ         0x04
#define AK9916_MODE_CONT_50_HZ         0x06
#define AK9916_MODE_CONT_100_HZ        0x08

#define AK9916_ODR_HZ                  100

// 0.15 uT/LSB, 1uT = 10mG, so raw * 3 / 2 gives milligauss
#define AK9916_MAG_MG_SCALE_NUM        3
#define AK9916_MAG_MG_SCALE_DEN        2

// ST1(0x10) .. ST2(0x18): status, X/Y/Z, temperature, status
#define AK9916_SAMPLE_REGS_LEN         9

// Consecutive reads without new data before the mode is re-armed, to recover
// from a magnetometer that lost its configuration (e.g. after its host MCU
// reset it). Cleared whenever the driver is initialised.
#define AK9916_NO_DATA_RETRIES         25

static uint8_t noDataCount;

static bool ak9916ReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return busReadRegisterBuffer(dev, reg, buf, len);
}

static bool ak9916WriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    return busWriteRegister(dev, reg, data);
}

static bool ak9916SetupMode(const extDevice_t *dev)
{
    uint8_t mode = 0;

    if (!ak9916WriteRegister(dev, AK9916_REG_CNTL2, AK9916_MODE_CONT_100_HZ)) {
        return false;
    }

    // Read back the mode so a magnetometer that is not answering (e.g. a GPS
    // module that never enabled the I2C bridge to its magnetometer) is reported
    // as a failure instead of silently returning zeros.
    if (!ak9916ReadRegisterBuffer(dev, AK9916_REG_CNTL2, &mode, 1)) {
        return false;
    }

    return (mode & 0x1F) == AK9916_MODE_CONT_100_HZ;
}

static bool ak9916Init(magDev_t *mag)
{
    extDevice_t *dev = &mag->dev;

    noDataCount = 0;

    busDeviceRegister(dev);

    if (!ak9916WriteRegister(dev, AK9916_REG_CNTL3, AK9916_CNTL3_SOFT_RESET)) {
        return false;
    }
    delay(1);

    if (!ak9916SetupMode(dev)) {
        return false;
    }

    // Continuous measurement at a fixed 100Hz ODR, which lets the compass task
    // schedule its reads accordingly.
    mag->magOdrHz = AK9916_ODR_HZ;

    return true;
}

static bool ak9916Read(magDev_t *mag, int16_t *magData)
{
    uint8_t buf[AK9916_SAMPLE_REGS_LEN];

    extDevice_t *dev = &mag->dev;

    // Reading the whole ST1..ST2 block in one transaction guarantees a
    // consistent sample, and reading ST2 releases the data registers for the
    // next measurement.
    if (!ak9916ReadRegisterBuffer(dev, AK9916_REG_ST1, buf, sizeof(buf))) {
        return false;
    }

    if ((buf[0] & AK9916_ST1_DATA_READY) == 0) {
        if (++noDataCount >= AK9916_NO_DATA_RETRIES) {
            noDataCount = 0;
            ak9916SetupMode(dev);
        }
        return false;
    }
    noDataCount = 0;

    // Overflown measurements are discarded, see the AK9916 datasheet
    if (buf[AK9916_SAMPLE_REGS_LEN - 1] & AK9916_ST2_MAG_SENSOR_OVERFLOW) {
        return false;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        const int32_t raw = (int16_t)(buf[1 + axis * 2 + 1] << 8 | buf[1 + axis * 2]);
        magData[axis] = (int16_t)constrain((raw * AK9916_MAG_MG_SCALE_NUM) / AK9916_MAG_MG_SCALE_DEN, INT16_MIN, INT16_MAX);
    }

    return true;
}

bool ak9916Detect(magDev_t *mag)
{
    uint8_t deviceId = 0;

    extDevice_t *dev = &mag->dev;

    if (dev->bus->busType != BUS_TYPE_I2C) {
        return false;
    }

    if (dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = AK9916_MAG_I2C_ADDRESS;
    }

    // WIA2, not WIA1: the AK9916 company ID is 0x48, which is also what the
    // AK8975/AK8963 drivers accept as their WHO_AM_I on the same address.
    if (!ak9916ReadRegisterBuffer(dev, AK9916_REG_WIA2, &deviceId, 1)) {
        return false;
    }

    if (deviceId != AK9916_DEVICE_ID) {
        return false;
    }

    mag->init = ak9916Init;
    mag->read = ak9916Read;

    return true;
}
#endif // USE_MAG_AK9916
