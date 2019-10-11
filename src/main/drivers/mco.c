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
#include "pg/mco.h"

void mcoInit(const mcoConfig_t *mcoConfig)
{
    // Only configure MCO2 with PLLI2SCLK as source for now.
    // Other MCO1 and other sources can easily be added.
    // For all F4 and F7 varianets, MCO1 is on PA8 and MCO2 is on PC9.

    if (mcoConfig->enabled[1]) {
        IO_t io = IOGetByTag(DEFIO_TAG_E(PC9));
        IOInit(io, OWNER_MCO, 2);
#if defined(STM32F7)
        HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLI2SCLK, RCC_MCODIV_4);
        IOConfigGPIOAF(io, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH,  GPIO_NOPULL), GPIO_AF0_MCO);
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
        RCC_MCO2Config(RCC_MCO2Source_PLLI2SCLK, RCC_MCO2Div_4);
        IOConfigGPIOAF(io, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL), GPIO_AF_MCO);
#else
#error Unsupported MCU
#endif
    }
}
#endif
