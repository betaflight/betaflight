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

#include <stdbool.h>
#include <stdint.h> 
#include <stdlib.h>
    
#include <platform.h>

#include "io.h"
#include "system.h"

#include "config/config_master.h"

#include "bus_i2c.h"
#include "nvic.h"
#include "io_impl.h"
#include "rcc.h"

extern i2cDevice_t i2cHardwareMap[I2CDEV_MAX];
extern i2cDevice_t i2cHardwareConfig[];

static I2CDevice i2cConfigFindMap(ioTag_t scl, ioTag_t sda)
{
    for (unsigned int map = 0 ; map < ARRAYLEN(i2cHardwareMap) ; map++) {
        if ((scl == i2cHardwareMap[map].scl) && (sda == i2cHardwareMap[map].sda))
            return (I2CDevice)map;
    }

    return I2CINVALID;
}

void i2cPinConfigSet(int bus, ioTag_t scl, ioTag_t sda)
{
    if (i2cConfigFindMap(scl, sda) == I2CINVALID) {
        // XXX Log or notify error
        return;
    }

    i2cPinConfig()->ioTagSCL[bus] = scl;
    i2cPinConfig()->ioTagSDA[bus] = sda;
}

void i2cConfig(void)
{
    ioTag_t tagSCL;
    ioTag_t tagSDA;
    I2CDevice map;

    for (I2CDevice bus = 0 ; bus < I2CDEV_MAX ; bus++) {
        i2cHardwareConfig[bus].configured = false;

        // Find valid mapping for this bus
        tagSCL = i2cPinConfig()->ioTagSCL[bus];
        tagSDA = i2cPinConfig()->ioTagSDA[bus];

        map = i2cConfigFindMap(tagSCL, tagSDA);

        if (map == I2CINVALID) {
            // XXX Should log config error (except NONE case)?
            continue;
        }

        i2cHardwareConfig[bus] = i2cHardwareMap[map];
        i2cHardwareConfig[bus].configured = true;
    }
}

void i2cInitAll(void)
{

    i2cConfig(); // Setup running configuration

    for (I2CDevice bus = I2CDEV_1 ; bus < I2CDEV_MAX ; bus++) {
        if (i2cHardwareConfig[bus].configured) {
            i2cInit(bus);
        }
    }
}
