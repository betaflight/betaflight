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

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"

bool busRawWriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
#ifdef USE_SPI
    if (busdev->bustype ==  BUSTYPE_SPI) {
#ifdef USE_SPI_TRANSACTION
        spiBusTransactionSetup(busdev);
#endif
        return spiBusWriteRegister(busdev, reg, data);
    } else
#endif
    {
        return busWriteRegister(busdev, reg, data);
    }
}

bool busWriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
    case BUSTYPE_SPI:
#ifdef USE_SPI_TRANSACTION
        // XXX Watch out fastpath users, if any
        return spiBusTransactionWriteRegister(busdev, reg & 0x7f, data);
#else
        return spiBusWriteRegister(busdev, reg & 0x7f, data);
#endif
#endif
#ifdef USE_I2C
    case BUSTYPE_I2C:
        return i2cBusWriteRegister(busdev, reg, data);
#endif
    default:
        return false;
    }
}

bool busRawWriteRegisterStart(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
#ifdef USE_SPI
    if (busdev->bustype ==  BUSTYPE_SPI) {
#ifdef USE_SPI_TRANSACTION
        spiBusTransactionSetup(busdev);
#endif
        return spiBusWriteRegister(busdev, reg, data);
    } else
#endif
    {
        return busWriteRegisterStart(busdev, reg, data);
    }
}

bool busWriteRegisterStart(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
    case BUSTYPE_SPI:
#ifdef USE_SPI_TRANSACTION
        // XXX Watch out fastpath users, if any
        return spiBusTransactionWriteRegister(busdev, reg & 0x7f, data);
#else
        return spiBusWriteRegister(busdev, reg & 0x7f, data);
#endif
#endif
#ifdef USE_I2C
    case BUSTYPE_I2C:
        return i2cBusWriteRegisterStart(busdev, reg, data);
#endif
    default:
        return false;
    }
}

bool busRawReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
#ifdef USE_SPI
    if (busdev->bustype ==  BUSTYPE_SPI) {
#ifdef USE_SPI_TRANSACTION
        spiBusTransactionSetup(busdev);
#endif
        return spiBusRawReadRegisterBuffer(busdev, reg, data, length);
    } else
#endif
    {
        return busReadRegisterBuffer(busdev, reg, data, length);
    }
}

bool busReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
    case BUSTYPE_SPI:
#ifdef USE_SPI_TRANSACTION
        // XXX Watch out fastpath users, if any
        return spiBusTransactionReadRegisterBuffer(busdev, reg | 0x80, data, length);
#else
        return spiBusReadRegisterBuffer(busdev, reg | 0x80, data, length);
#endif
#endif
#ifdef USE_I2C
    case BUSTYPE_I2C:
        return i2cBusReadRegisterBuffer(busdev, reg, data, length);
#endif
    default:
        return false;
    }
}

bool busRawReadRegisterBufferStart(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
#ifdef USE_SPI
    if (busdev->bustype ==  BUSTYPE_SPI) {
#ifdef USE_SPI_TRANSACTION
        spiBusTransactionSetup(busdev);
#endif
        return spiBusRawReadRegisterBuffer(busdev, reg, data, length);
    } else
#endif
    {
        return busReadRegisterBufferStart(busdev, reg, data, length);
    }
}

// Start the I2C read, but do not wait for completion
bool busReadRegisterBufferStart(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
    case BUSTYPE_SPI:
        // For SPI allow the transaction to complete
#ifdef USE_SPI_TRANSACTION
        // XXX Watch out fastpath users, if any
        return spiBusTransactionReadRegisterBuffer(busdev, reg | 0x80, data, length);
#else
        return spiBusReadRegisterBuffer(busdev, reg | 0x80, data, length);
#endif
#endif
#ifdef USE_I2C
    case BUSTYPE_I2C:
        // Initiate the read access
        return i2cBusReadRegisterBufferStart(busdev, reg, data, length);
#endif
    default:
        return false;
    }
}

// Returns true if bus is still busy
bool busBusy(const busDevice_t *busdev, bool *error)
{
#if !defined(USE_I2C)
    UNUSED(error);
#endif
    switch (busdev->bustype) {
#ifdef USE_SPI
    case BUSTYPE_SPI:
        // No waiting on SPI
        return false;
#endif

#ifdef USE_I2C
    case BUSTYPE_I2C:
        return i2cBusBusy(busdev, error);
#endif

    default:
        return false;
    }
}

uint8_t busReadRegister(const busDevice_t *busdev, uint8_t reg)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(busdev);
    UNUSED(reg);
    return false;
#else
    uint8_t data;
    busReadRegisterBuffer(busdev, reg, &data, 1);
    return data;
#endif
}

void busDeviceRegister(const busDevice_t *busdev)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(busdev);
#endif

    switch (busdev->bustype) {
#if defined(USE_SPI)
    case BUSTYPE_SPI:
        spiBusDeviceRegister(busdev);

        break;
#endif
#if defined(USE_I2C)
    case BUSTYPE_I2C:
        i2cBusDeviceRegister(busdev);

        break;
#endif
    default:
        break;
    }
}
