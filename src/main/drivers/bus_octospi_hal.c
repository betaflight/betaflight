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

static void Error_Handler(void) { while (1) { } }

uint32_t mmEnableCounter = 0; // XXX
uint32_t mmDisableCounter = 0; // XXX


RAM_CODE NOINLINE static void octoSpiAbort(octoSpiDevice_t *octoSpi)
{
    OSPI_HandleTypeDef *handle = &octoSpi->hoctoSpi;

    SET_BIT(handle->Instance->CR, OCTOSPI_CR_ABORT);
}

RAM_CODE NOINLINE static void octoSpiWaitStatusFlags(octoSpiDevice_t *octoSpi, uint32_t mask, int polarity)
{
    OSPI_HandleTypeDef *handle = &octoSpi->hoctoSpi;

    uint32_t regval;

    if (polarity) {
        while (!((regval = READ_REG(handle->Instance->SR)) & mask))
            {}
    } else {
        while (((regval = READ_REG(handle->Instance->SR)) & mask))
            {}
    }
}
/*
 * Disable memory mapped mode.
 *
 * @See octoSpiEnableMemoryMappedMode
 * @See RAM_CODE
 *
 * Once this is called any code or data in the memory mapped region cannot be accessed.
 * Thus, this function itself must be in RAM, and the caller's code and data should all be in RAM
 * and this requirement continues until octoSpiEnableMemoryMappedMode is called.
 * This applies to ISR code that runs from the memory mapped region, so likely the caller should
 * also disable IRQs before calling this.
 */
RAM_CODE NOINLINE void octoSpiDisableMemoryMappedMode(octoSpiDevice_t *octoSpi)
{
    OSPI_HandleTypeDef *handle = &octoSpi->hoctoSpi;

    octoSpiAbort(octoSpi);

    MODIFY_REG(handle->Instance->CR, OCTOSPI_CR_FMODE, 0x0); // b00 = indirect write, see OCTOSPI->CR->FMODE

    mmDisableCounter++;
}

/*
 * Enable memory mapped mode.
 *
 * @See octoSpiEnableMemoryMappedMode
 * @See RAM_CODE
 */

RAM_CODE NOINLINE void octoSpiEnableMemoryMappedMode(octoSpiDevice_t *octoSpi)
{
    mmEnableCounter++;

    OSPI_HandleTypeDef *handle = &octoSpi->hoctoSpi;

    octoSpiAbort(octoSpi);
    octoSpiWaitStatusFlags(octoSpi, OCTOSPI_SR_BUSY, 0);

    MODIFY_REG(handle->Instance->CR, OCTOSPI_CR_FMODE, OCTOSPI_CR_FMODE);

    // Note: The OCTOSPI peripheral's registers for memory mapped mode will have been configured by the bootloader
    // They are flash chip specific, e.g. the amount of address/data lines to use, the command for reading, etc.

    // Bypass the HAL requirement for configuring the commands.  See implementation of HAL_OSPI_MemoryMapped for the specific MCU.
#if defined(STM32H730xx)
    octoSpi->hoctoSpi.State = HAL_OSPI_STATE_BUSY_MEM_MAPPED;
#else
#error MCU not supported.
#endif

    //__HAL_OSPI_ENABLE(handle);
}

RAM_CODE NOINLINE void octoSpiTestEnableDisableMemoryMappedMode(octoSpiDevice_t *octoSpi)
{
    __disable_irq();
    octoSpiDisableMemoryMappedMode(octoSpi);
    octoSpiEnableMemoryMappedMode(octoSpi);
    __enable_irq();
}

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

        octoSpiTestEnableDisableMemoryMappedMode(octoSpi);
    } else {
        failureMode(FAILURE_DEVELOPER); // trying to use this implementation when memory mapped mode is not already enabled by a bootloader

        // Here is where we would configure the peripheral and call HAL_OSPI_Init() and HAL_OSPIM_Config()
    }
#else
#error MCU not supported.
#endif

}

#endif
