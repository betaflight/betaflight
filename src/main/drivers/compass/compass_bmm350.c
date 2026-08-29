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
 *
 * BMM350 magnetometer driver (I2C).
 *
 * Compensation and bring-up follow Bosch BMM350 SensorAPI /
 * ArduPilot AP_Compass_BMM350. Raw register values are not usable
 * without OTP trim.
 *
 * References:
 *   https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm350/
 *   https://github.com/boschsensortec/BMM350_SensorAPI
 *   https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Compass/AP_Compass_BMM350.cpp
 */

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MAG_BMM350

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_bmm350.h"

#define BMM350_I2C_ADDRESS_ADSEL_GND    0x14
#define BMM350_I2C_ADDRESS_ADSEL_VDDIO  0x15

#define BMM350_REG_CHIP_ID              0x00
#define BMM350_REG_PMU_CMD_AGGR_SET     0x04
#define BMM350_REG_PMU_CMD_AXIS_EN      0x05
#define BMM350_REG_PMU_CMD              0x06
#define BMM350_REG_PMU_CMD_STATUS_0     0x07
#define BMM350_REG_INT_CTRL             0x2E
#define BMM350_REG_MAG_X_XLSB           0x31
#define BMM350_REG_OTP_CMD              0x50
#define BMM350_REG_OTP_DATA_MSB         0x52
#define BMM350_REG_OTP_STATUS           0x55
#define BMM350_REG_CMD                  0x7E

#define BMM350_CHIP_ID                  0x33
#define BMM350_CMD_SOFTRESET            0xB6

#define BMM350_OTP_CMD_DIR_READ         (0x01 << 5)
#define BMM350_OTP_CMD_PWR_OFF_OTP      (0x04 << 5)
#define BMM350_OTP_STATUS_ERROR_MASK    0xE0
#define BMM350_OTP_STATUS_CMD_DONE      0x01
#define BMM350_OTP_DATA_LENGTH          32

#define BMM350_OTP_TEMP_OFF_SENS        0x0D
#define BMM350_OTP_MAG_OFFSET_X         0x0E
#define BMM350_OTP_MAG_OFFSET_Y         0x0F
#define BMM350_OTP_MAG_OFFSET_Z         0x10
#define BMM350_OTP_MAG_SENS_X           0x10
#define BMM350_OTP_MAG_SENS_Y           0x11
#define BMM350_OTP_MAG_SENS_Z           0x11
#define BMM350_OTP_MAG_TCO_X            0x12
#define BMM350_OTP_MAG_TCO_Y            0x13
#define BMM350_OTP_MAG_TCO_Z            0x14
#define BMM350_OTP_MAG_TCS_X            0x12
#define BMM350_OTP_MAG_TCS_Y            0x13
#define BMM350_OTP_MAG_TCS_Z            0x14
#define BMM350_OTP_CROSS_X_Y            0x15
#define BMM350_OTP_CROSS_Y_X            0x15
#define BMM350_OTP_CROSS_Z_X            0x16
#define BMM350_OTP_CROSS_Z_Y            0x16
#define BMM350_OTP_MAG_DUT_T_0          0x18

#define BMM350_SENS_CORR_Y              0.01f
#define BMM350_TCS_CORR_Z               0.0001f

#define BMM350_PMU_CMD_SUSPEND          0x00
#define BMM350_PMU_CMD_NORMAL           0x01
#define BMM350_PMU_CMD_UPD_OAE          0x02
#define BMM350_PMU_CMD_FGR              0x05
#define BMM350_PMU_CMD_BR               0x07

#define BMM350_PMU_STATUS_BUSY          0x01
#define BMM350_PMU_STATUS_NORMAL        0x08
#define BMM350_PMU_STATUS_CMD_MASK      0xE0
#define BMM350_PMU_STATUS_CMD_SHIFT     5

#define BMM350_ODR_100HZ                0x04
#define BMM350_AVERAGING_4              (0x02 << 4)
#define BMM350_AXIS_EN_XYZ              0x07

#define BMM350_INT_MODE_PULSED          (0 << 0)
#define BMM350_INT_POL_ACTIVE_HIGH      (1 << 1)
#define BMM350_INT_OD_PUSHPULL          (1 << 2)
#define BMM350_INT_OUTPUT_DISABLE       (0 << 3)
#define BMM350_INT_DRDY_EN              (1 << 7)

#define BMM350_I2C_DUMMY_BYTES          2
#define BMM350_MEAS_BYTES               12
#define BMM350_BOOT_RETRIES             3
#define BMM350_OTP_POLL_TRIES           10

#define BMM350_XY_SENSITIVE             14.55f
#define BMM350_Z_SENSITIVE              9.0f
#define BMM350_TEMP_SENSITIVE           0.00204f
#define BMM350_XY_INA_GAIN              19.46f
#define BMM350_Z_INA_GAIN               31.0f
#define BMM350_ADC_GAIN                 (1.0f / 1.5f)
#define BMM350_LUT_GAIN                 0.714607238769531f
#define BMM350_POWER                    (1000000.0f / 1048576.0f)

#define BMM350_XY_SCALE                 (BMM350_POWER / (BMM350_XY_SENSITIVE * BMM350_XY_INA_GAIN * BMM350_ADC_GAIN * BMM350_LUT_GAIN))
#define BMM350_Z_SCALE                  (BMM350_POWER / (BMM350_Z_SENSITIVE * BMM350_Z_INA_GAIN * BMM350_ADC_GAIN * BMM350_LUT_GAIN))
#define BMM350_TEMP_SCALE               (1.0f / (BMM350_TEMP_SENSITIVE * BMM350_ADC_GAIN * BMM350_LUT_GAIN * 1048576.0f))

#define BMM350_UT_TO_MILLIGAUSS         10.0f

static const uint8_t bmm350I2cAddresses[] = {
    BMM350_I2C_ADDRESS_ADSEL_GND,
    BMM350_I2C_ADDRESS_ADSEL_VDDIO,
};

// Per-unit OTP trim. Zeroed if OTP read fails (uncompensated field still usable).
typedef struct bmm350Comp_s {
    float offsetX;
    float offsetY;
    float offsetZ;
    float offsetTemp;
    float sensitX;
    float sensitY;
    float sensitZ;
    float sensitTemp;
    float tcoX;
    float tcoY;
    float tcoZ;
    float tcsX;
    float tcsY;
    float tcsZ;
    float t0;
    float crossXY;
    float crossYX;
    float crossZX;
    float crossZY;
} bmm350Comp_t;

static bmm350Comp_t bmm350Comp;

/**
 * @brief Read registers, discarding the two dummy bytes the BMM350 prepends.
 */
static bool bmm350ReadBytes(const extDevice_t *dev, uint8_t reg, uint8_t *out, uint8_t len)
{
    uint8_t buf[BMM350_MEAS_BYTES + BMM350_I2C_DUMMY_BYTES];

    if (len > BMM350_MEAS_BYTES) {
        return false;
    }

    if (!busReadRegisterBuffer(dev, reg, buf, len + BMM350_I2C_DUMMY_BYTES)) {
        return false;
    }

    memcpy(out, &buf[BMM350_I2C_DUMMY_BYTES], len);
    return true;
}

/**
 * @brief Poll PMU_CMD_STATUS_0 until the given command completes or timeoutMs elapses.
 */
static bool bmm350WaitPmuCmd(const extDevice_t *dev, uint8_t cmd, uint32_t timeoutMs)
{
    for (uint32_t remaining = timeoutMs; remaining > 0; remaining--) {
        delay(1);

        uint8_t status = 0;
        if (!bmm350ReadBytes(dev, BMM350_REG_PMU_CMD_STATUS_0, &status, 1)) {
            return false;
        }

        const uint8_t lastCmd = (status & BMM350_PMU_STATUS_CMD_MASK) >> BMM350_PMU_STATUS_CMD_SHIFT;
        if ((status & BMM350_PMU_STATUS_BUSY) == 0 && lastCmd == cmd) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Switch PMU power mode, suspending first if currently in normal/UPD_OAE.
 */
static bool bmm350SetPowerMode(const extDevice_t *dev, uint8_t mode)
{
    uint8_t pmuCmd = 0;

    if (!bmm350ReadBytes(dev, BMM350_REG_PMU_CMD, &pmuCmd, 1)) {
        return false;
    }

    if (pmuCmd == BMM350_PMU_CMD_NORMAL || pmuCmd == BMM350_PMU_CMD_UPD_OAE) {
        if (!busWriteRegister(dev, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_SUSPEND)) {
            return false;
        }
        bmm350WaitPmuCmd(dev, BMM350_PMU_CMD_SUSPEND, 6);
    }

    if (!busWriteRegister(dev, BMM350_REG_PMU_CMD, mode)) {
        return false;
    }

    return bmm350WaitPmuCmd(dev, mode, 38);
}

/**
 * @brief Read 32 OTP words used for offset, sensitivity, TCO/TCS and cross-axis trim.
 */
static bool bmm350ReadOtp(const extDevice_t *dev, uint16_t *otpData)
{
    for (uint8_t index = 0; index < BMM350_OTP_DATA_LENGTH; index++) {
        if (!busWriteRegister(dev, BMM350_REG_OTP_CMD, BMM350_OTP_CMD_DIR_READ | index)) {
            return false;
        }

        bool done = false;
        for (int poll = 0; poll < BMM350_OTP_POLL_TRIES; poll++) {
            delayMicroseconds(300);

            uint8_t status = 0;
            if (!bmm350ReadBytes(dev, BMM350_REG_OTP_STATUS, &status, 1)
                || (status & BMM350_OTP_STATUS_ERROR_MASK) != 0) {
                return false;
            }

            if (status & BMM350_OTP_STATUS_CMD_DONE) {
                done = true;
                break;
            }
        }

        if (!done) {
            return false;
        }

        uint8_t otpBytes[2];
        if (!bmm350ReadBytes(dev, BMM350_REG_OTP_DATA_MSB, otpBytes, 2)) {
            return false;
        }

        otpData[index] = ((uint16_t)otpBytes[0] << 8) | otpBytes[1];
    }

    return true;
}

/**
 * @brief Sign-extend a 12-bit field packed in a 16-bit OTP word.
 */
static float bmm350SignExtend12(uint16_t value)
{
    return (float)((int16_t)(value << 4) >> 4);
}

/**
 * @brief Unpack OTP words into bmm350Comp (Bosch SensorAPI / ArduPilot layout).
 */
static void bmm350ParseOtp(const uint16_t *otpData)
{
    bmm350Comp.offsetX = bmm350SignExtend12(otpData[BMM350_OTP_MAG_OFFSET_X] & 0x0FFF);
    bmm350Comp.offsetY = bmm350SignExtend12(((otpData[BMM350_OTP_MAG_OFFSET_X] & 0xF000) >> 4)
        + (otpData[BMM350_OTP_MAG_OFFSET_Y] & 0x00FF));
    bmm350Comp.offsetZ = bmm350SignExtend12((otpData[BMM350_OTP_MAG_OFFSET_Y] & 0x0F00)
        + (otpData[BMM350_OTP_MAG_OFFSET_Z] & 0x00FF));
    bmm350Comp.offsetTemp = (float)((int8_t)otpData[BMM350_OTP_TEMP_OFF_SENS]) * (1.0f / 5.0f);

    bmm350Comp.sensitX = (float)((int8_t)((otpData[BMM350_OTP_MAG_SENS_X] & 0xFF00) >> 8)) * (1.0f / 256.0f);
    bmm350Comp.sensitY = (float)((int8_t)otpData[BMM350_OTP_MAG_SENS_Y]) * (1.0f / 256.0f) + BMM350_SENS_CORR_Y;
    bmm350Comp.sensitZ = (float)((int8_t)((otpData[BMM350_OTP_MAG_SENS_Z] & 0xFF00) >> 8)) * (1.0f / 256.0f);
    bmm350Comp.sensitTemp = (float)((int8_t)((otpData[BMM350_OTP_TEMP_OFF_SENS] & 0xFF00) >> 8)) * (1.0f / 512.0f);

    bmm350Comp.tcoX = (float)((int8_t)otpData[BMM350_OTP_MAG_TCO_X]) * (1.0f / 32.0f);
    bmm350Comp.tcoY = (float)((int8_t)otpData[BMM350_OTP_MAG_TCO_Y]) * (1.0f / 32.0f);
    bmm350Comp.tcoZ = (float)((int8_t)otpData[BMM350_OTP_MAG_TCO_Z]) * (1.0f / 32.0f);

    bmm350Comp.tcsX = (float)((int8_t)((otpData[BMM350_OTP_MAG_TCS_X] & 0xFF00) >> 8)) * (1.0f / 16384.0f);
    bmm350Comp.tcsY = (float)((int8_t)((otpData[BMM350_OTP_MAG_TCS_Y] & 0xFF00) >> 8)) * (1.0f / 16384.0f);
    bmm350Comp.tcsZ = (float)((int8_t)((otpData[BMM350_OTP_MAG_TCS_Z] & 0xFF00) >> 8)) * (1.0f / 16384.0f) - BMM350_TCS_CORR_Z;

    bmm350Comp.t0 = (float)((int16_t)otpData[BMM350_OTP_MAG_DUT_T_0]) * (1.0f / 512.0f) + 23.0f;

    bmm350Comp.crossXY = (float)((int8_t)otpData[BMM350_OTP_CROSS_X_Y]) * (1.0f / 800.0f);
    bmm350Comp.crossYX = (float)((int8_t)((otpData[BMM350_OTP_CROSS_Y_X] & 0xFF00) >> 8)) * (1.0f / 800.0f);
    bmm350Comp.crossZX = (float)((int8_t)otpData[BMM350_OTP_CROSS_Z_X]) * (1.0f / 800.0f);
    bmm350Comp.crossZY = (float)((int8_t)((otpData[BMM350_OTP_CROSS_Z_Y] & 0xFF00) >> 8)) * (1.0f / 800.0f);
}

/**
 * @brief Bit reset then flux-guide reset. Needs 2.2 uF on CRST.
 */
static bool bmm350MagReset(const extDevice_t *dev)
{
    uint8_t status = 0;

    if (!bmm350ReadBytes(dev, BMM350_REG_PMU_CMD_STATUS_0, &status, 1)) {
        return false;
    }

    if (status & BMM350_PMU_STATUS_NORMAL) {
        if (!busWriteRegister(dev, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_SUSPEND)) {
            return false;
        }
        bmm350WaitPmuCmd(dev, BMM350_PMU_CMD_SUSPEND, 6);
    }

    if (!busWriteRegister(dev, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR)
        || !bmm350WaitPmuCmd(dev, BMM350_PMU_CMD_BR, 19)) {
        return false;
    }

    if (!busWriteRegister(dev, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR)
        || !bmm350WaitPmuCmd(dev, BMM350_PMU_CMD_FGR, 23)) {
        return false;
    }

    return bmm350SetPowerMode(dev, BMM350_PMU_CMD_NORMAL);
}

/**
 * @brief Assemble 3 register bytes into a sign-extended 21-bit integer.
 */
static int32_t bmm350Assemble21(const uint8_t *bytes)
{
    const uint32_t raw = ((uint32_t)bytes[0] << 8) | ((uint32_t)bytes[1] << 16) | ((uint32_t)bytes[2] << 24);

    return (int32_t)raw >> 8;
}

/**
 * @brief Convert 21-bit raw samples to compensated milligauss.
 */
static void bmm350Compensate(int32_t rawX, int32_t rawY, int32_t rawZ, int32_t rawTemp, int16_t *magData)
{
    float magx = (float)rawX * BMM350_XY_SCALE;
    float magy = (float)rawY * BMM350_XY_SCALE;
    float magz = (float)rawZ * BMM350_Z_SCALE;
    float temp = ((float)rawTemp * BMM350_TEMP_SCALE) - 25.49f;

    temp = ((1.0f + bmm350Comp.sensitTemp) * temp) + bmm350Comp.offsetTemp;

    magx = ((1.0f + bmm350Comp.sensitX) * magx) + bmm350Comp.offsetX + (bmm350Comp.tcoX * (temp - bmm350Comp.t0));
    magx /= 1.0f + bmm350Comp.tcsX * (temp - bmm350Comp.t0);

    magy = ((1.0f + bmm350Comp.sensitY) * magy) + bmm350Comp.offsetY + (bmm350Comp.tcoY * (temp - bmm350Comp.t0));
    magy /= 1.0f + bmm350Comp.tcsY * (temp - bmm350Comp.t0);

    magz = ((1.0f + bmm350Comp.sensitZ) * magz) + bmm350Comp.offsetZ + (bmm350Comp.tcoZ * (temp - bmm350Comp.t0));
    magz /= 1.0f + bmm350Comp.tcsZ * (temp - bmm350Comp.t0);

    const float denom = 1.0f - bmm350Comp.crossYX * bmm350Comp.crossXY;
    const float crX = (magx - bmm350Comp.crossXY * magy) / denom;
    const float crY = (magy - bmm350Comp.crossYX * magx) / denom;
    const float crZ = magz + (magx * (bmm350Comp.crossYX * bmm350Comp.crossZY - bmm350Comp.crossZX)
        - magy * (bmm350Comp.crossZY - bmm350Comp.crossXY * bmm350Comp.crossZX)) / denom;

    magData[X] = (int16_t)constrain(lrintf(crX * BMM350_UT_TO_MILLIGAUSS), INT16_MIN, INT16_MAX);
    magData[Y] = (int16_t)constrain(lrintf(crY * BMM350_UT_TO_MILLIGAUSS), INT16_MIN, INT16_MAX);
    magData[Z] = (int16_t)constrain(lrintf(crZ * BMM350_UT_TO_MILLIGAUSS), INT16_MIN, INT16_MAX);
}

/**
 * @brief Read CHIP_ID; accept 0x33.
 */
static bool bmm350Probe(magDev_t *magDev)
{
    uint8_t chipId = 0;

    if (!bmm350ReadBytes(&magDev->dev, BMM350_REG_CHIP_ID, &chipId, 1)) {
        return false;
    }

    return chipId == BMM350_CHIP_ID;
}

/**
 * @brief Soft-reset, load OTP, magnetic reset, 100 Hz normal mode.
 */
static bool bmm350Init(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;
    bool identified = false;

    busDeviceRegister(dev);

    for (int attempt = 0; attempt < BMM350_BOOT_RETRIES; attempt++) {
        if (!busWriteRegister(dev, BMM350_REG_CMD, BMM350_CMD_SOFTRESET)) {
            continue;
        }
        delay(24);

        uint8_t chipId = 0;
        if (bmm350ReadBytes(dev, BMM350_REG_CHIP_ID, &chipId, 1) && chipId == BMM350_CHIP_ID) {
            identified = true;
            break;
        }
    }

    if (!identified) {
        return false;
    }

    uint16_t otpData[BMM350_OTP_DATA_LENGTH];
    memset(&bmm350Comp, 0, sizeof(bmm350Comp));
    if (bmm350ReadOtp(dev, otpData)) {
        bmm350ParseOtp(otpData);
    }
    busWriteRegister(dev, BMM350_REG_OTP_CMD, BMM350_OTP_CMD_PWR_OFF_OTP);

    // BR/FGR needs the 2.2 uF CRST capacitor. Still enter normal mode if reset times out.
    bmm350MagReset(dev);

    const uint8_t intCtrl = BMM350_INT_MODE_PULSED | BMM350_INT_POL_ACTIVE_HIGH
        | BMM350_INT_OD_PUSHPULL | BMM350_INT_OUTPUT_DISABLE | BMM350_INT_DRDY_EN;
    if (!busWriteRegister(dev, BMM350_REG_INT_CTRL, intCtrl)) {
        return false;
    }

    if (!busWriteRegister(dev, BMM350_REG_PMU_CMD_AGGR_SET, BMM350_AVERAGING_4 | BMM350_ODR_100HZ)) {
        return false;
    }
    if (!busWriteRegister(dev, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE)) {
        return false;
    }
    bmm350WaitPmuCmd(dev, BMM350_PMU_CMD_UPD_OAE, 20);

    if (!busWriteRegister(dev, BMM350_REG_PMU_CMD_AXIS_EN, BMM350_AXIS_EN_XYZ)) {
        return false;
    }

    if (!bmm350SetPowerMode(dev, BMM350_PMU_CMD_NORMAL)) {
        return false;
    }

    magDev->magOdrHz = 100;
    return true;
}

/**
 * @brief Burst-read XYZ and temperature, then compensate into milligauss.
 *
 * Does not wait for INT_STATUS DRDY; that bit is unreliable on some silicon
 * while the data registers are still valid.
 */
static bool bmm350Read(magDev_t *magDev, int16_t *magData)
{
    extDevice_t *dev = &magDev->dev;
    uint8_t buf[BMM350_MEAS_BYTES];

    if (!bmm350ReadBytes(dev, BMM350_REG_MAG_X_XLSB, buf, BMM350_MEAS_BYTES)) {
        return false;
    }

    const int32_t rawX = bmm350Assemble21(&buf[0]);
    const int32_t rawY = bmm350Assemble21(&buf[3]);
    const int32_t rawZ = bmm350Assemble21(&buf[6]);
    const int32_t rawTemp = bmm350Assemble21(&buf[9]);

    bmm350Compensate(rawX, rawY, rawZ, rawTemp, magData);

    return true;
}

/**
 * @brief Detect a BMM350 on I2C and install init/read callbacks.
 */
bool bmm350Detect(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    delay(3);

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address != 0) {
        if (bmm350Probe(magDev)) {
            magDev->init = bmm350Init;
            magDev->read = bmm350Read;
            return true;
        }
        return false;
    }

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        for (unsigned i = 0; i < ARRAYLEN(bmm350I2cAddresses); i++) {
            dev->busType_u.i2c.address = bmm350I2cAddresses[i];
            if (bmm350Probe(magDev)) {
                magDev->init = bmm350Init;
                magDev->read = bmm350Read;
                return true;
            }
        }
        dev->busType_u.i2c.address = 0;
    }

    return false;
}

#endif // USE_MAG_BMM350
