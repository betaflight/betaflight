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
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OCTOSPI

#include "system.h"

#include "bus_octospi.h"
#include "bus_octospi_impl.h"

#if !defined(STM32H730xx)
// Implementation only tested on STM32H730.
#error MCU not supported.
#endif

RAM_CODE NOINLINE static void Error_Handler(void) {
    while (1)
    {
        NOOP;
    }
}


#define __OSPI_GET_FLAG(__INSTANCE__, __FLAG__)           ((READ_BIT((__INSTANCE__)->SR, (__FLAG__)) != 0U) ? SET : RESET)
#define __OSPI_ENABLE(__INSTANCE__)                       SET_BIT((__INSTANCE__)->CR, OCTOSPI_CR_EN)
#define __OSPI_DISABLE(__INSTANCE__)                      CLEAR_BIT((__INSTANCE__)->CR, OCTOSPI_CR_EN)
#define __OSPI_IS_ENABLED(__INSTANCE__)                       (READ_BIT((__INSTANCE__)->CR, OCTOSPI_CR_EN) != 0U)

RAM_CODE NOINLINE static void octoSpiAbort(OCTOSPI_TypeDef *instance)
{
    SET_BIT(instance->CR, OCTOSPI_CR_ABORT);
}

RAM_CODE NOINLINE static void octoSpiWaitStatusFlags(OCTOSPI_TypeDef *instance, uint32_t mask, int polarity)
{
    uint32_t regval;

    if (polarity) {
        while (!((regval = READ_REG(instance->SR)) & mask))
            {}
    } else {
        while (((regval = READ_REG(instance->SR)) & mask))
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

uint32_t busyCount = 0;

RAM_CODE NOINLINE void octoSpiDisableMemoryMappedMode(OCTOSPI_TypeDef *instance)
{
    octoSpiAbort(instance);
    if (__OSPI_GET_FLAG(instance, OCTOSPI_SR_BUSY) == SET) {
        busyCount++;
        __OSPI_DISABLE(instance);
        octoSpiAbort(instance);
    }
    octoSpiWaitStatusFlags(instance, OCTOSPI_SR_BUSY, 0);

    uint32_t fmode = 0x0;  // b00 = indirect write, see OCTOSPI->CR->FMODE
    MODIFY_REG(instance->CR, OCTOSPI_CR_FMODE, fmode);

    uint32_t regval = READ_REG(instance->CR);
    if ((regval & OCTOSPI_CR_FMODE) != fmode) {
        Error_Handler();
    }

    if (!__OSPI_IS_ENABLED(instance)) {
        __OSPI_ENABLE(instance);
    }
}

/*
 * Enable memory mapped mode.
 *
 * @See octoSpiEnableMemoryMappedMode
 * @See RAM_CODE
 */

RAM_CODE NOINLINE void octoSpiEnableMemoryMappedMode(OCTOSPI_TypeDef *instance)
{
    octoSpiAbort(instance);
    octoSpiWaitStatusFlags(instance, OCTOSPI_SR_BUSY, 0);

    MODIFY_REG(instance->CR, OCTOSPI_CR_FMODE, OCTOSPI_CR_FMODE);

    // Note: The OCTOSPI peripheral's registers for memory mapped mode will have been configured by the bootloader
    // They are flash chip specific, e.g. the amount of address/data lines to use, the command for reading, etc.
}

RAM_CODE NOINLINE void octoSpiTestEnableDisableMemoryMappedMode(octoSpiDevice_t *octoSpi)
{
    OCTOSPI_TypeDef *instance = octoSpi->dev;

    __disable_irq();
    octoSpiDisableMemoryMappedMode(instance);
    octoSpiEnableMemoryMappedMode(instance);
    __enable_irq();
}

RAM_CODE NOINLINE bool octoSpiTransmit1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
    // TODO
    return false;
}

RAM_CODE NOINLINE bool octoSpiReceive1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    // TODO
    return false;
}


RAM_CODE NOINLINE bool octoSpiReceive4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    // TODO
    return false;
}

RAM_CODE NOINLINE bool octoSpiReceiveWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    // TODO
    return false;
}

RAM_CODE NOINLINE bool octoSpiReceiveWithAddress4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    // TODO
    return false;
}

RAM_CODE NOINLINE bool octoSpiTransmitWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    // TODO
    return false;
}

RAM_CODE NOINLINE bool octoSpiTransmitWithAddress4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    // TODO
    return false;
}

RAM_CODE NOINLINE bool octoSpiInstructionWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize)
{
    // TODO
    return false;
}


void octoSpiInitDevice(OCTOSPIDevice device)
{
    octoSpiDevice_t *octoSpi = &(octoSpiDevice[device]);

#if defined(STM32H730xx)
    if (isMemoryMappedModeEnabled()) {
        // Bootloader has already configured the IO, clocks and peripherals.
        octoSpiTestEnableDisableMemoryMappedMode(octoSpi);
    } else {
        failureMode(FAILURE_DEVELOPER); // trying to use this implementation when memory mapped mode is not already enabled by a bootloader

        // Here is where we would configure the OCTOSPI1/2 and OCTOSPIM peripherals for the non-memory-mapped use case.
    }
#else
#error MCU not supported.
#endif

}

#endif
