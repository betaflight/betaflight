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
#include "drivers/io_types.h"

#ifdef TARGET_BUS_INIT
void targetBusInit(void);
#endif

typedef enum {
    BUSTYPE_NONE = 0,
    BUSTYPE_I2C,
    BUSTYPE_SPI,
} busType_e;

#define BUSTYPE_NONE_STR "NONE"
#define BUSTYPE_I2C_STR  "I2C"
#define BUSTYPE_SPI_STR  "SPI"

typedef struct busDeviceConfig_s {
    busType_e busType;
    uint8_t   busNum;
    uint8_t   i2cAddr;
    ioTag_t   spiCsPin;
    ioTag_t   drdyPin;
} busDeviceConfig_t;
