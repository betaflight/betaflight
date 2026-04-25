/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * RCC clock enable/disable/reset for HAL2-based STM32 families.
 * Uses direct register access via STM32_SET_BIT/STM32_READ_BIT macros
 * (the HAL2 equivalents of the old SET_BIT/READ_BIT).
 */

#include "platform.h"
#include "platform/rcc.h"

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

#define __HAL2_RCC_CLK_ENABLE(bus, suffix, enbit)   do {       \
        __IO uint32_t tmpreg;                                  \
        SET_BIT(RCC->bus ## ENR ## suffix, enbit);             \
        /* Delay after an RCC peripheral clock enabling */     \
        tmpreg = READ_BIT(RCC->bus ## ENR ## suffix, enbit);   \
        UNUSED(tmpreg);                                        \
    } while(0)

#define __HAL2_RCC_CLK_DISABLE(bus, suffix, enbit) (RCC->bus ## ENR ## suffix &= ~(enbit))

#define __HAL2_RCC_CLK(bus, suffix, enbit, newState) \
    if (newState == ENABLE) {                        \
        __HAL2_RCC_CLK_ENABLE(bus, suffix, enbit);   \
    } else {                                         \
        __HAL2_RCC_CLK_DISABLE(bus, suffix, enbit);  \
    }

#define NOSUFFIX // Empty

    switch (tag) {
    case RCC_AHB1:
        __HAL2_RCC_CLK(AHB1, NOSUFFIX, mask, NewState);
        break;

    case RCC_AHB2:
        __HAL2_RCC_CLK(AHB2, NOSUFFIX, mask, NewState);
        break;

    case RCC_AHB4:
        __HAL2_RCC_CLK(AHB4, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB1L:
        __HAL2_RCC_CLK(APB1L, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB1H:
        __HAL2_RCC_CLK(APB1H, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB2:
        __HAL2_RCC_CLK(APB2, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB3:
        __HAL2_RCC_CLK(APB3, NOSUFFIX, mask, NewState);
        break;
    }
}

void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

#define __HAL2_RCC_FORCE_RESET(bus, suffix, enbit) (RCC->bus ## RSTR ## suffix |= (enbit))
#define __HAL2_RCC_RELEASE_RESET(bus, suffix, enbit) (RCC->bus ## RSTR ## suffix &= ~(enbit))
#define __HAL2_RCC_RESET(bus, suffix, enbit, NewState) \
    if (NewState == ENABLE) {                          \
        __HAL2_RCC_RELEASE_RESET(bus, suffix, enbit);  \
    } else {                                           \
        __HAL2_RCC_FORCE_RESET(bus, suffix, enbit);    \
    }

    switch (tag) {
    case RCC_AHB1:
        __HAL2_RCC_RESET(AHB1, NOSUFFIX, mask, NewState);
        break;

    case RCC_AHB2:
        __HAL2_RCC_RESET(AHB2, NOSUFFIX, mask, NewState);
        break;

    case RCC_AHB4:
        __HAL2_RCC_RESET(AHB4, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB1L:
        __HAL2_RCC_RESET(APB1L, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB1H:
        __HAL2_RCC_RESET(APB1H, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB2:
        __HAL2_RCC_RESET(APB2, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB3:
        __HAL2_RCC_RESET(APB3, NOSUFFIX, mask, NewState);
        break;
    }
}
