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

/*
 * OctoSPI support.
 *
 * Some STM32H7 MCUs support 2 OCTOSPI peripherals, each with up to 2 flash chips, using 2/4/8 IO lines each. Small MCU packages only have one instance.
 * Additionally there is an OCTOSPIM peripheral which maps OCTOSPI1/2 peripherals to IO pins and can perform arbitration.
 *
 * Initial implementation is focused on supporting memory-mapped flash chips connected to an OCTOSPI peripheral that is already initialised by a bootloader.
 *
 * As such the following things are NOT supported:
 * * Configuration of IO pins.
 * * Clock configuration.
 * * User-configuration.
 * * OCTOSPIM configuration.
 * * OCTOSPI2.
 *
 * Should the firmware need to know about the pins used by OCTOSPI then code can be written to determine this from the registers of the OCTOSPIM and OCTOSPI1/2 peripherals.
 *
 * Implementation notes:
 * It's not possible to use the HAL libraries without modifying them and maintaining the internal state of the HAL structures.
 * The HAL libraries were not designed to support the use-case of a bootloader configuring the flash in memory mapped mode and then
 * having a second piece of software (this firmware) also use the flash.  Furthermore many HAL methods were not designed to run with
 * interrupts disabled which is necessary as other ISRs in this firmware will be running from external flash and must be disabled.
 * See HAL_OSPI_Abort, OSPI_WaitFlagStateUntilTimeout, etc.
 * All code that is executed when memory mapped mode is disabled needs to run from RAM, this would also involve modification of the HAL
 * libraries.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OCTOSPI

#include "bus_octospi.h"
#include "bus_octospi_impl.h"

octoSpiDevice_t octoSpiDevice[OCTOSPIDEV_COUNT] = { 0 };

MMFLASH_CODE_NOINLINE octoSpiDevice_e octoSpiDeviceByInstance(OCTOSPI_TypeDef *instance)
{
#ifdef USE_OCTOSPI_DEVICE_1
    if (instance == OCTOSPI1) {
        return OCTOSPIDEV_1;
    }
#endif

    return OCTOSPIINVALID;
}

OCTOSPI_TypeDef *octoSpiInstanceByDevice(octoSpiDevice_e device)
{
    if (device == OCTOSPIINVALID || device >= OCTOSPIDEV_COUNT) {
        return NULL;
    }

    return octoSpiDevice[device].dev;
}

const octoSpiHardware_t octoSpiHardware[] = {
#if defined(STM32H730xx) || defined(STM32H723xx) || defined(STM32H735xx)
    {
        .device = OCTOSPIDEV_1,
        .reg = OCTOSPI1,
    }
#else
#error MCU not supported.
#endif
};

bool octoSpiInit(octoSpiDevice_e device)
{
    for (size_t hwindex = 0; hwindex < ARRAYLEN(octoSpiHardware); hwindex++) {
        const octoSpiHardware_t *hw = &octoSpiHardware[hwindex];

        const octoSpiDevice_e hwDevice = hw->device;
        octoSpiDevice_t *pDev = &octoSpiDevice[hwDevice];

        pDev->dev = hw->reg;
    }

    switch (device) {
    case OCTOSPIINVALID:
        return false;
    case OCTOSPIDEV_1:
#ifdef USE_OCTOSPI_DEVICE_1
        octoSpiInitDevice(OCTOSPIDEV_1);
        return true;
#else
        break;
#endif
    }
    return false;
}

#endif
