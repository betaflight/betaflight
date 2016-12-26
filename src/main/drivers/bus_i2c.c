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

#ifdef USE_I2C

#include "io.h"
#include "system.h"

#include "bus_i2c.h"
#include "nvic.h"
#include "io_impl.h"
#include "rcc.h"

/*
 * XXX To pursue future (further) per-MCU config scheme:
 * https://github.com/betaflight/betaflight/pull/1867#discussion_r93647427
 * (@ledvinap) This way to find I2C device can't handle case when multiple
 * driver instances share identical pins. It is (probably) not problem for I2C,
 * but for example F3 ADC can connect single pin to multiple ADCs.
 * It would be nice to use common mechanism to specify pins for all drivers ...
 */

static i2cDevice_t *i2cConfigFindMap(ioTag_t scl, ioTag_t sda)
{
    for (size_t map = 0 ; map < i2cPinMapSize() ; map++) {
        if ((scl == i2cPinMap[map].scl) && (sda == i2cPinMap[map].sda))
            return &i2cPinMap[map];
    }

    return NULL;
}

static void i2cPinConfigSet(i2cPinConfig_t *i2cPinConfig, const i2cTargetConfig_t *pTargetConfig)
{
    if (!i2cConfigFindMap(pTargetConfig->scl, pTargetConfig->sda)) {
        // XXX Should log or notify error
        return;
    }

    i2cPinConfig->ioTagSCL[pTargetConfig->bus] = pTargetConfig->scl;
    i2cPinConfig->ioTagSDA[pTargetConfig->bus] = pTargetConfig->sda;
}

void i2cTargetConfigInit(i2cPinConfig_t *i2cPinConfig)
{
    for (size_t i = 0 ; i < i2cTargetConfigSize() ; i++) {
        i2cPinConfigSet(i2cPinConfig, &i2cTargetConfig[i]);
    }
}

static void i2cConfig(const i2cPinConfig_t *i2cPinConfig)
{

    for (I2CDevice bus = 0 ; bus < I2CDEV_COUNT ; bus++) {
        i2cHardwareConfig[bus].configured = false;

        // See if this bus is configured.

        ioTag_t tagSCL = i2cPinConfig->ioTagSCL[bus];
        ioTag_t tagSDA = i2cPinConfig->ioTagSDA[bus];

        if (!tagSCL || !tagSDA)
            continue;

        // Find valid mapping for this bus.

        i2cDevice_t *map = i2cConfigFindMap(tagSCL, tagSDA);

        if (!map) {
            // XXX Should log config error (except NONE case)?
            continue;
        }

        i2cHardwareConfig[bus] = *map;
        i2cHardwareConfig[bus].configured = true;
    }
}

void i2cInitAll(const i2cPinConfig_t *i2cPinConfig)
{

    i2cConfig(i2cPinConfig); // Setup running configuration

    for (I2CDevice bus = I2CDEV_1 ; bus < I2CDEV_COUNT ; bus++) {
        if (i2cHardwareConfig[bus].configured) {
            i2cInitBus(bus);
        }
    }
}
#endif // USE_I2C
