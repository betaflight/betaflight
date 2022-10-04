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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MCO

#include "drivers/io.h"
#include "drivers/mco.h"
#include "pg/mco.h"

#ifdef STM32G4

// Notes
// - MCO output stability
// MCO output should not be too high.
// For example
// it is required to use DIV4 for SYSCLK = 254MHz (derives 62.5MHz)
// it is required to use DIV2 for SYSCLK = 170MHz (derives 85MHz)
//
// - MCO frequency can be more flexible if PLLR is made configurable.

const uint32_t mcoSources[MCO_SOURCE_COUNT] = {
    RCC_MCO1SOURCE_NOCLOCK,
    RCC_MCO1SOURCE_SYSCLK,
    RCC_MCO1SOURCE_HSI,
    RCC_MCO1SOURCE_HSE,
    RCC_MCO1SOURCE_PLLCLK, // PLLR on G4
    RCC_MCO1SOURCE_LSI,
    RCC_MCO1SOURCE_LSE,
    RCC_MCO1SOURCE_HSI48,
};

const uint32_t mcoDividers[MCO_DIVIDER_COUNT] = {
    RCC_MCO_DIV1,
    RCC_MCO_DIV2,
    RCC_MCO_DIV4,
    RCC_MCO_DIV8,
    RCC_MCO_DIV16,
};
#endif

void mcoConfigure(MCODevice_e device, const mcoConfig_t *config)
{
    if (!config->enabled) {
        return;
    }

    IO_t io;

#if defined(STM32F4) || defined(STM32F7)
    // Only configure MCO2 with PLLI2SCLK as source for now.
    // Other MCO1 and other sources can easily be added.

    switch(device) {
    case MCODEV_1: // MCO1 on PA8
        return; // Not supported (yet)

    case MCODEV_2: // MCO2 on PC9
        io = IOGetByTag(DEFIO_TAG_E(PC9));
        IOInit(io, OWNER_MCO, 2);
#if defined(STM32F7)
        HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLI2SCLK, RCC_MCODIV_4);
        IOConfigGPIOAF(io, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH,  GPIO_NOPULL), GPIO_AF0_MCO);
#else
        // All F4s
        RCC_MCO2Config(RCC_MCO2Source_PLLI2SCLK, RCC_MCO2Div_4);
        IOConfigGPIOAF(io, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL), GPIO_AF_MCO);
#endif
        break;
    }
#elif defined(STM32G4)
    // G4 only supports one MCO on PA8
    UNUSED(device);

    io = IOGetByTag(DEFIO_TAG_E(PA8));
    IOInit(io, OWNER_MCO, 1);
    HAL_RCC_MCOConfig(RCC_MCO, mcoSources[config->source], mcoDividers[config->divider]);
#else
#error Unsupported MCU
#endif
}
#endif
