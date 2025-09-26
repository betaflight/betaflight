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

#ifdef USE_MAG_QMC5883P

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_qmc5883p.h"

// Forward declarations for static functions
static bool qmc5883pInit(magDev_t *magDev);
static bool qmc5883pRead(magDev_t *magDev, int16_t *magData);

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

    // Chip ID verification successful, register function pointers
    magDev->init = qmc5883pInit;
    magDev->read = qmc5883pRead;
    return true;
}

static bool qmc5883pInit(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;
    bool ack = true;

    // Step 1: Write special XYZ sign configuration value to register 0x29
    ack = ack && busWriteRegister(dev, QMC5883P_REG_XYZ_UNLOCK, QMC5883P_XYZ_SIGN_CONFIG);
    if (!ack) {
        // I2C write failed during XYZ sign configuration
        return false;
    }

    // Step 2: Configure CONF1 register with continuous mode, 100Hz ODR, 8G range, and OSR1=8
    ack = ack && busWriteRegister(dev, QMC5883P_REG_CONF1, QMC5883P_DEFAULT_CONF1);
    if (!ack) {
        // I2C write failed during CONF1 configuration
        return false;
    }

    // Step 3: Configure CONF2 register with OSR2=8 oversampling settings
    ack = ack && busWriteRegister(dev, QMC5883P_REG_CONF2, QMC5883P_DEFAULT_CONF2);
    if (!ack) {
        // I2C write failed during CONF2 configuration
        return false;
    }

    // Step 4: Set magOdrHz to 100 for proper timing integration with Betaflight
    magDev->magOdrHz = 100;

    return true;
}

static bool qmc5883pRead(magDev_t *magDev, int16_t *magData)
{
    static uint8_t buf[6];
    static uint8_t status = 0;
    static enum {
        STATE_WAIT_DRDY,
        STATE_STATUS_READ,
        STATE_DATA_READ,
        STATE_UNLOCK_READ,
    } state = STATE_WAIT_DRDY;

    extDevice_t *dev = &magDev->dev;

    // Do not start a new transaction if the bus is still busy
    if (busBusy(dev)) {
        return false;
    }

    switch (state)
    {
        default:
        case STATE_WAIT_DRDY:
            // Start async read of STATUS
            if (!busReadRegisterBufferStart(dev, QMC5883P_REG_STATUS, &status, sizeof(status))) {
                return false;
            }
            state = STATE_STATUS_READ;
            return false;

        case STATE_STATUS_READ:
            // Wait for STATUS to complete, then decide next action
            if (!busReadRegisterBufferFinish(dev)) {
                return false;
            }
            if (status & QMC5883P_STATUS_DATA_READY) {
                // Start async read of 6 data bytes (X/Y/Z LSB/MSB)
                if (!busReadRegisterBufferStart(dev, QMC5883P_REG_DATA_OUTPUT_X, buf, sizeof(buf))) {
                    state = STATE_WAIT_DRDY;
                    return false;
                }
                state = STATE_DATA_READ;
                return false;
            } else if (status & QMC5883P_STATUS_DATA_OVERRUN) {
                // Data overrun detected: read the hidden unlock register (0x29)
                // This clears/unsticks the data path on QMC5883P
                if (!busReadRegisterBufferStart(dev, QMC5883P_REG_XYZ_UNLOCK, buf, 1)) {
                    state = STATE_WAIT_DRDY;
                    return false;
                }
                state = STATE_UNLOCK_READ;
                return false;
            } else {
                // Neither DRDY nor OVERRUN set; poll again next cycle
                state = STATE_WAIT_DRDY;
                return false;
            }

        case STATE_DATA_READ:
            // Wait for data read to complete, then parse
            if (!busReadRegisterBufferFinish(dev)) {
                return false;
            }
            // Process 6 bytes of magnetic data (X, Y, Z as 16-bit little-endian values)
            // QMC5883P data format: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
            int16_t rawX = (int16_t)(buf[1] << 8 | buf[0]);  // X-axis: bytes 0-1
            int16_t rawY = (int16_t)(buf[3] << 8 | buf[2]);  // Y-axis: bytes 2-3
            int16_t rawZ = (int16_t)(buf[5] << 8 | buf[4]);  // Z-axis: bytes 4-5

            // Enhanced data validation - check for obvious data corruption
            // QMC5883P in 8G range should produce values roughly in range ±32767
            // All zeros or all 0xFFFF typically indicates communication issues
            if ((rawX == 0 && rawY == 0 && rawZ == 0) || 
                (rawX == -1 && rawY == -1 && rawZ == -1)) {
                // Likely data corruption, reset state and retry next cycle
                state = STATE_WAIT_DRDY;
                return false;
            }

            // Additional validation: check for stuck data (same values repeatedly)
            static int16_t lastX = 0, lastY = 0, lastZ = 0;
            static uint8_t stuckCount = 0;
            
            if (rawX == lastX && rawY == lastY && rawZ == lastZ) {
                stuckCount++;
                if (stuckCount > 10) {
                    // Data appears stuck, likely sensor or communication issue
                    // Reset state and force status read to recover
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

            // QMC5883P coordinate system matches standard right-hand rule
            // No axis inversions needed (unlike QMC5883L which requires X and Z negation)
            // Store processed values in magData array following Betaflight conventions
            // Note: Coordinate system transformations for external mounting are handled
            // by the higher-level magnetometer subsystem in Betaflight, not at the driver level
            magData[X] = rawX;
            magData[Y] = rawY;
            magData[Z] = rawZ;

            state = STATE_WAIT_DRDY;
            return true;

        case STATE_UNLOCK_READ:
            // Finish the 0x29 read used to clear overrun
            if (!busReadRegisterBufferFinish(dev)) {
                return false;
            }
            state = STATE_WAIT_DRDY;
            return false;
    }

    return false;
}
#endif