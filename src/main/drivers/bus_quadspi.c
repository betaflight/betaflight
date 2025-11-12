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
 *
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_QUADSPI

#include "drivers/bus_quadspi.h"
#include "drivers/bus_quadspi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"

#if PLATFORM_TRAIT_RCC
#include "platform/rcc.h"
#endif

#include "pg/bus_quadspi.h"

quadSpiDevice_t quadSpiDevice[QUADSPIDEV_COUNT] = { 0 };

quadSpiDevice_e quadSpiDeviceByInstance(QUADSPI_TypeDef *instance)
{
    if (!instance) {
        return QUADSPIINVALID;
    }
    for (size_t i = 0; i < QUADSPIDEV_COUNT; i++) {
        if (quadSpiHardware[i].reg == instance) {
            return quadSpiHardware[i].device;
        }
    }
    return QUADSPIINVALID;
}

QUADSPI_TypeDef *quadSpiInstanceByDevice(quadSpiDevice_e device)
{
    if (device == QUADSPIINVALID || device >= QUADSPIDEV_COUNT) {
        return NULL;
    }

    return quadSpiDevice[device].dev;
}

bool quadSpiInit(quadSpiDevice_e device)
{
    switch (device) {
    case QUADSPIINVALID:
        return false;
    case QUADSPIDEV_1:
#ifdef USE_QUADSPI_DEVICE_1
        quadSpiInitDevice(device);
        return true;
#else
        break;
#endif
    }
    return false;
}

uint32_t quadSpiTimeoutUserCallback(QUADSPI_TypeDef *instance)
{
    quadSpiDevice_e device = quadSpiDeviceByInstance(instance);
    if (device == QUADSPIINVALID) {
        return -1;
    }
    quadSpiDevice[device].errorCount++;
    return quadSpiDevice[device].errorCount;
}

uint16_t quadSpiGetErrorCounter(QUADSPI_TypeDef *instance)
{
    quadSpiDevice_e device = quadSpiDeviceByInstance(instance);
    if (device == QUADSPIINVALID) {
        return 0;
    }
    return quadSpiDevice[device].errorCount;
}

void quadSpiResetErrorCounter(QUADSPI_TypeDef *instance)
{
    quadSpiDevice_e device = quadSpiDeviceByInstance(instance);
    if (device != QUADSPIINVALID) {
        quadSpiDevice[device].errorCount = 0;
    }
}
#endif
