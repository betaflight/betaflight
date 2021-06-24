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

#ifdef USE_OCTOSPI

#include "system.h"

#include "bus_octospi.h"
#include "bus_octospi_impl.h"

void octoSpiInitDevice(OCTOSPIDevice device)
{
    octoSpiDevice_t *octoSpi = &(octoSpiDevice[device]);

    octoSpi->hoctoSpi.Instance = octoSpi->dev;

#if defined(STM32H730xx)
    if (isMemoryMappedModeEnabled()) {
        // NOTE: Specifically no HAL_OSPI_Init()
        // The hardware is already initialised by the bootloader, so just update the HAL state to match the hardware's current configuration
        octoSpi->hoctoSpi.State = HAL_OSPI_STATE_BUSY_MEM_MAPPED;
        octoSpi->hoctoSpi.ErrorCode = HAL_OSPI_ERROR_NONE;
    } else {
        failureMode(FAILURE_DEVELOPER); // trying to use this implementation when memory mapped mode is not already enabled by a bootloader

        // Here is where we would configure the peripheral and call HAL_OSPI_Init() and HAL_OSPIM_Config()
    }
#else
#error MCU not supported.
#endif

}

#endif
