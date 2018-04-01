/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#include "drivers/bus_i2c.h"
#include "drivers/io_types.h"

typedef enum {
    BUSTYPE_NONE = 0,
    BUSTYPE_I2C,
    BUSTYPE_SPI,
    BUSTYPE_MPU_SLAVE // Slave I2C on SPI master
} busType_e;

typedef struct busDevice_s {
    busType_e bustype;
    union {
        struct deviceSpi_s {
            SPI_TypeDef *instance;
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

bool busWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool busReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t busReadRegister(const busDevice_t *bus, uint8_t reg);
