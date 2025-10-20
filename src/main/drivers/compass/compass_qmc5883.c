/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_QMC5883

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_qmc5883.h"

// QMC5883L I2C Address
#define QMC5883L_MAG_I2C_ADDRESS        0x0D

// Registers
#define QMC5883L_REG_RESET              0x0B
#define QMC5883L_REG_CONF1              0x09
#define QMC5883L_REG_CONF2              0x0A

// data output rates for 5883L
#define QMC5883L_ODR_10HZ               (0x00 << 2)
#define QMC5883L_ODR_50HZ               (0x01 << 2)
#define QMC5883L_ODR_100HZ              (0x02 << 2)
#define QMC5883L_ODR_200HZ              (0x03 << 2)

// Sensor operation modes
#define QMC5883L_MODE_STANDBY           0x00
#define QMC5883L_MODE_CONTINUOUS        0x01

#define QMC5883L_RNG_2G                 (0x00 << 4)
#define QMC5883L_RNG_8G                 (0x01 << 4)

#define QMC5883L_OSR_512                (0x00 << 6)
#define QMC5883L_OSR_256                (0x01 << 6)
#define QMC5883L_OSR_128                (0x02 << 6)
#define QMC5883L_OSR_64                 (0x03 << 6)

#define QMC5883L_RST                     0x80

#define QMC5883L_REG_DATA_OUTPUT_X      0x00
#define QMC5883L_REG_DATA_UNLOCK        0x05
#define QMC5883L_REG_STATUS             0x06
#define QMC5883L_REG_STATUS_DRDY        0x01
#define QMC5883L_REG_STATUS_OVL         0x02
#define QMC5883L_REG_STATUS_DOR         0x04

#define QMC5883L_REG_ID                 0x0D
#define QMC5883L_ID_VAL                 0xFF

// QMC5883P I2C Address
#define QMC5883P_I2C_ADDRESS            0x2C

// QMC5883P Register Addresses
#define QMC5883P_REG_ID                 0x00
#define QMC5883P_REG_DATA_OUTPUT_X      0x01
#define QMC5883P_REG_DATA_OUTPUT_X_MSB  0x02
#define QMC5883P_REG_DATA_OUTPUT_Y      0x03
#define QMC5883P_REG_DATA_OUTPUT_Y_MSB  0x04
#define QMC5883P_REG_DATA_OUTPUT_Z      0x05
#define QMC5883P_REG_DATA_OUTPUT_Z_MSB  0x06
#define QMC5883P_REG_STATUS             0x09
#define QMC5883P_REG_CONF1              0x0A
#define QMC5883P_REG_CONF2              0x0B

// QMC5883P Chip ID
#define QMC5883P_ID_VAL                 0x80

// QMC5883P Configuration Values
// Mode settings for CONF1 register
#define QMC5883P_MODE_STANDBY           0x00
#define QMC5883P_MODE_CONTINUOUS        0x03

// Output Data Rate (ODR) settings for CONF1 register
#define QMC5883P_ODR_10HZ              (0x00 << 2)
#define QMC5883P_ODR_50HZ              (0x01 << 2)
#define QMC5883P_ODR_100HZ             (0x02 << 2)
#define QMC5883P_ODR_200HZ             (0x03 << 2)

// Range settings for CONF1 register
#define QMC5883P_RNG_2G                (0x00 << 4)
#define QMC5883P_RNG_8G                (0x01 << 4)

// Oversampling Ratio 1 (OSR1) settings for CONF1 register
#define QMC5883P_OSR1_8                (0x00 << 6)
#define QMC5883P_OSR1_4                (0x01 << 6)
#define QMC5883P_OSR1_2                (0x02 << 6)
#define QMC5883P_OSR1_1                (0x03 << 6)

// Oversampling Ratio 2 (OSR2) settings for CONF2 register
#define QMC5883P_OSR2_8                0x08
#define QMC5883P_OSR2_4                0x04
#define QMC5883P_OSR2_2                0x02
#define QMC5883P_OSR2_1                0x01

// Status register bits
#define QMC5883P_STATUS_DATA_READY     0x01
#define QMC5883P_STATUS_DATA_OVERRUN   0x02

// Special configuration registers and values
#define QMC5883P_REG_XYZ_UNLOCK        0x29
#define QMC5883P_XYZ_SIGN_CONFIG       0x06

// Unlock register for data overrun recovery (uses Z_MSB register)
#define QMC5883P_REG_DATA_UNLOCK       QMC5883P_REG_DATA_OUTPUT_Z_MSB

// Default configuration for Betaflight
#define QMC5883P_DEFAULT_CONF1         (QMC5883P_MODE_CONTINUOUS | QMC5883P_ODR_100HZ | QMC5883P_RNG_8G | QMC5883P_OSR1_8)
#define QMC5883P_DEFAULT_CONF2         QMC5883P_OSR2_8

// Unified descriptor for QMC5883 variants
typedef enum {
    QMC_VARIANT_L,
    QMC_VARIANT_P,
} qmc_variant_e;

typedef struct {
    qmc_variant_e variant;
    uint8_t i2c_address;
    uint8_t reg_status;
    uint8_t reg_data;
    uint8_t reg_data_unlock;
    uint8_t status_data_ready_mask;
    uint8_t status_data_overrun_mask;
    uint16_t default_odr_hz;
} qmc_descriptor_t;

static const qmc_descriptor_t qmc5883l_desc = {
    .variant = QMC_VARIANT_L,
    .i2c_address = QMC5883L_MAG_I2C_ADDRESS,
    .reg_status = QMC5883L_REG_STATUS,
    .reg_data = QMC5883L_REG_DATA_OUTPUT_X,
    .reg_data_unlock = QMC5883L_REG_DATA_UNLOCK,
    .status_data_ready_mask = QMC5883L_REG_STATUS_DRDY,
    .status_data_overrun_mask = QMC5883L_REG_STATUS_DOR,
    .default_odr_hz = 200,
};

static const qmc_descriptor_t qmc5883p_desc = {
    .variant = QMC_VARIANT_P,
    .i2c_address = QMC5883P_I2C_ADDRESS,
    .reg_status = QMC5883P_REG_STATUS,
    .reg_data = QMC5883P_REG_DATA_OUTPUT_X,
    .reg_data_unlock = QMC5883P_REG_DATA_UNLOCK,
    .status_data_ready_mask = QMC5883P_STATUS_DATA_READY,
    .status_data_overrun_mask = QMC5883P_STATUS_DATA_OVERRUN,
    .default_odr_hz = 100,
};

static bool qmc5883Init(magDev_t *magDev)
{
    bool ack = true;
    extDevice_t *dev = &magDev->dev;
    const qmc_descriptor_t *desc = (const qmc_descriptor_t *)dev->callbackArg;

    if (!desc) {
        return false;
    }

    busDeviceRegister(dev);

    // For L variant, apply reset sequence and configure CONF1
    if (desc->variant == QMC_VARIANT_L) {
        ack = ack && busWriteRegister(dev, QMC5883L_REG_RESET, 0x01);
        ack = ack && busWriteRegister(dev, QMC5883L_REG_CONF1, QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_200HZ | QMC5883L_OSR_512 | QMC5883L_RNG_8G);
    } else {
        // P variant default confs are expected to be written by detect path, but ensure magOdrHz is set
        // No further writes here to avoid repeating detect-specific unlocks
    }

    if (!ack) {
        return false;
    }

    magDev->magOdrHz = desc->default_odr_hz;
    return true;
}

static bool qmc5883Read(magDev_t *magDev, int16_t *magData)
{
    static uint8_t buf[6];
    static uint8_t status = 0; // request status on first read
    static enum {
        STATE_WAIT_DRDY,
        STATE_READ,
    } state = STATE_WAIT_DRDY;

    extDevice_t *dev = &magDev->dev;
    const qmc_descriptor_t *desc = (const qmc_descriptor_t *)dev->callbackArg;

    if (!desc) {
        return false;
    }

    switch (state) {
        default:
        case STATE_WAIT_DRDY:
            if (status & desc->status_data_ready_mask) {
                if (busReadRegisterBufferStart(dev, desc->reg_data, buf, sizeof(buf))) {
                    state = STATE_READ;
                }
            } else if (status & desc->status_data_overrun_mask) {
                if (busReadRegisterBufferStart(dev, desc->reg_data_unlock, buf + sizeof(buf) - 1, 1)) {
                    status = 0; // force status read next
                }
            } else {
                busReadRegisterBufferStart(dev, desc->reg_status, &status, sizeof(status));
            }
            return false;

        case STATE_READ:
        {
            // Data format is LSB, MSB pairs for both variants
            int16_t rawX = (int16_t)(buf[1] << 8 | buf[0]);
            int16_t rawY = (int16_t)(buf[3] << 8 | buf[2]);
            int16_t rawZ = (int16_t)(buf[5] << 8 | buf[4]);

            if (desc->variant == QMC_VARIANT_P) {
                // Extra validation logic originally in P driver
                if ((rawX == 0 && rawY == 0 && rawZ == 0) || (rawX == -1 && rawY == -1 && rawZ == -1)) {
                    state = STATE_WAIT_DRDY;
                    return false;
                }

                static int16_t lastX = 0, lastY = 0, lastZ = 0;
                static uint8_t stuckCount = 0;
                if (rawX == lastX && rawY == lastY && rawZ == lastZ) {
                    stuckCount++;
                    if (stuckCount > 10) {
                        state = STATE_WAIT_DRDY;
                        stuckCount = 0;
                        return false;
                    }
                } else {
                    stuckCount = 0;
                }
                lastX = rawX;
                lastY = rawY;
                lastZ = rawZ;

                magData[X] = rawX;
                magData[Y] = rawY;
                magData[Z] = rawZ;
            } else {
                // QMC5883L: original implementation simply read raw values and returned
                magData[X] = rawX;
                magData[Y] = rawY;
                magData[Z] = rawZ;
            }

            state = STATE_WAIT_DRDY;
            status = 0;
            return true;
        }
    }

    return false;
}

bool qmc5883lDetect(magDev_t *magDev)
{

    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = QMC5883L_MAG_I2C_ADDRESS;
    }

    // Must write reset first  - don't care about the result
    busWriteRegister(dev, QMC5883L_REG_CONF2, QMC5883L_RST);
    delay(20);

    uint8_t sig = 0;
    bool ack = busReadRegisterBuffer(dev, QMC5883L_REG_ID, &sig, 1);
    if (ack && sig == QMC5883_ID_VAL) {
        // Should be in standby mode after soft reset and sensor is really present
        // Reading ChipID of 0xFF alone is not sufficient to be sure the QMC is present
        ack = busReadRegisterBuffer(dev, QMC5883L_REG_CONF1, &sig, 1);
        if (ack && sig != QMC5883L_MODE_STANDBY) {
            return false;
        }

        // Attach descriptor and functions
        dev->callbackArg = (uintptr_t)&qmc5883l_desc;
        magDev->init = qmc5883Init;
        magDev->read = qmc5883Read;
        return true;
    }

    return false;
}

bool qmc5883pDetect(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    // Set I2C address to 0x2C if not already configured
    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = QMC5883P_I2C_ADDRESS;
    }

    // Read chip ID from register 0x00 and verify it's 0x80
    uint8_t chipId = 0;
    bool ack = busReadRegisterBuffer(dev, QMC5883P_REG_ID, &chipId, 1);
    
    if (!ack) {
        // I2C communication failed during detection
        return false;
    }
    
    if (chipId != QMC5883P_ID_VAL) {
        // Wrong chip ID - not a QMC5883P or communication error
        return false;
    }

    // For P variant we perform the special unlock/config writes here (retaining original behavior)
    ack = busWriteRegister(dev, QMC5883P_REG_XYZ_UNLOCK, QMC5883P_XYZ_SIGN_CONFIG) && ack;
    ack = busWriteRegister(dev, QMC5883P_REG_CONF1, QMC5883P_DEFAULT_CONF1) && ack;
    ack = busWriteRegister(dev, QMC5883P_REG_CONF2, QMC5883P_DEFAULT_CONF2) && ack;
    if (!ack) {
        return false;
    }

    // Attach descriptor and functions
    dev->callbackArg = (uintptr_t)&qmc5883p_desc;
    magDev->init = qmc5883Init;
    magDev->read = qmc5883Read;
    return true;
}

bool qmc5883Detect(magDev_t *magDev)
{
    if (qmc5883lDetect(magDev)) {
        return true;
    }
    if (qmc5883pDetect(magDev)) {
        return true;
    }
    return false;
}

#endif
