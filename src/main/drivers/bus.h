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

#pragma once

#include "platform.h"

#include "drivers/bus_i2c.h"
#include "drivers/io_types.h"

typedef enum {
    BUSTYPE_NONE = 0,
    BUSTYPE_I2C,
    BUSTYPE_SPI,
    BUSTYPE_MPU_SLAVE, // Slave I2C on SPI master
    BUSTYPE_GYRO_AUTO,  // Only used by acc/gyro bus auto detection code
} busType_e;

struct spiDevice_s;

typedef struct busDevice_s {
    busType_e bustype;
    union {
        struct deviceSpi_s {
            SPI_TypeDef *instance;
#ifdef USE_SPI_TRANSACTION
            struct SPIDevice_s *device;    // Back ptr to controller for this device.
            // Cached SPI_CR1 for spiBusTransactionXXX
            uint16_t modeCache;        // XXX cr1Value may be a better name?
#endif
#if defined(USE_HAL_DRIVER)
            SPI_HandleTypeDef* handle; // cached here for efficiency
#endif
            IO_t csnPin;
        } spi;
        struct deviceI2C_s {
            I2CDevice device;
            uint8_t address;
        } i2c;
        struct deviceMpuSlave_s {
            const struct busDevice_s *master;
            uint8_t address;
        } mpuSlave;
    } busdev_u;
} busDevice_t;

#ifdef TARGET_BUS_INIT
void targetBusInit(void);
#endif

bool busRawWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool busWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool busRawWriteRegisterStart(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool busWriteRegisterStart(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool busRawReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
bool busReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t busReadRegister(const busDevice_t *bus, uint8_t reg);
bool busRawReadRegisterBufferStart(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length);
bool busReadRegisterBufferStart(const busDevice_t *busdev, uint8_t reg, uint8_t *data, uint8_t length);
bool busBusy(const busDevice_t *busdev, bool *error);
void busDeviceRegister(const busDevice_t *busdev);
