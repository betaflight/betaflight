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

#include "platform.h"
#include "rcc.h"

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

#if defined(USE_HAL_DRIVER)

#define __HAL_RCC_CLK_ENABLE(bus, enbit)   do {            \
        __IO uint32_t tmpreg;                              \
        SET_BIT(RCC->bus ## ENR, enbit);                   \
        /* Delay after an RCC peripheral clock enabling */ \
        tmpreg = READ_BIT(RCC->bus ## ENR, enbit);         \
        UNUSED(tmpreg);                                    \
    } while(0)

#define __HAL_RCC_CLK_DISABLE(bus, enbit) (RCC->bus ## ENR &= ~(enbit))

#define __HAL_RCC_CLK(bus, enbit, newState) \
    if (newState == ENABLE) {               \
        __HAL_RCC_CLK_ENABLE(bus, enbit);   \
    } else {                                \
        __HAL_RCC_CLK_DISABLE(bus, enbit);  \
    }

    switch (tag) {
    case RCC_AHB1:
        __HAL_RCC_CLK(AHB1, mask, NewState);
        break;

    case RCC_AHB2:
        __HAL_RCC_CLK(AHB2, mask, NewState);
        break;

#ifndef STM32H7
    case RCC_APB1:
        __HAL_RCC_CLK(APB1, mask, NewState);
        break;
#endif

    case RCC_APB2:
        __HAL_RCC_CLK(APB2, mask, NewState);
        break;

#ifdef STM32H7

    case RCC_AHB3:
        __HAL_RCC_CLK(AHB3, mask, NewState);
        break;

    case RCC_AHB4:
        __HAL_RCC_CLK(AHB4, mask, NewState);
        break;

    case RCC_APB1L:
        __HAL_RCC_CLK(APB1L, mask, NewState);
        break;

    case RCC_APB1H:
        __HAL_RCC_CLK(APB1H, mask, NewState);
        break;

    case RCC_APB3:
        __HAL_RCC_CLK(APB3, mask, NewState);
        break;

    case RCC_APB4:
        __HAL_RCC_CLK(APB4, mask, NewState);
        break;
#endif
    }
#else
    switch (tag) {
#if defined(STM32F3) || defined(STM32F1)
    case RCC_AHB:
        RCC_AHBPeriphClockCmd(mask, NewState);
        break;
#endif
    case RCC_APB2:
        RCC_APB2PeriphClockCmd(mask, NewState);
        break;
    case RCC_APB1:
        RCC_APB1PeriphClockCmd(mask, NewState);
        break;
#if defined(STM32F4)
    case RCC_AHB1:
        RCC_AHB1PeriphClockCmd(mask, NewState);
        break;
#endif
    }
#endif
}

void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

// Peripheral reset control relies on RSTR bits are identical to ENR bits where applicable
#define __HAL_RCC_FORCE_RESET(bus, enbit) (RCC->bus ## RSTR |= (enbit))
#define __HAL_RCC_RELEASE_RESET(bus, enbit) (RCC->bus ## RSTR &= ~(enbit))
#define __HAL_RCC_RESET(bus, enbit, NewState) \
    if (NewState == ENABLE) {                 \
        __HAL_RCC_RELEASE_RESET(bus, enbit);  \
    } else {                                  \
        __HAL_RCC_FORCE_RESET(bus, enbit);    \
    }

#if defined(USE_HAL_DRIVER)

    switch (tag) {
    case RCC_AHB1:
        __HAL_RCC_RESET(AHB1, mask, NewState);
        break;

    case RCC_AHB2:
        __HAL_RCC_RESET(AHB2, mask, NewState);
        break;

#ifndef STM32H7
    case RCC_APB1:
        __HAL_RCC_RESET(APB1, mask, NewState);
        break;
#endif

    case RCC_APB2:
        __HAL_RCC_RESET(APB2, mask, NewState);
        break;

#ifdef STM32H7

    case RCC_AHB3:
        __HAL_RCC_RESET(AHB3, mask, NewState);
        break;

    case RCC_AHB4:
        __HAL_RCC_RESET(AHB4, mask, NewState);
        break;

    case RCC_APB1L:
        __HAL_RCC_RESET(APB1L, mask, NewState);
        break;

    case RCC_APB1H:
        __HAL_RCC_RESET(APB1H, mask, NewState);
        break;

    case RCC_APB3:
        __HAL_RCC_RESET(APB3, mask, NewState);
        break;

    case RCC_APB4:
        __HAL_RCC_RESET(APB4, mask, NewState);
        break;
#endif
    }

#else

    switch (tag) {
#if defined(STM32F3) || defined(STM32F10X_CL)
    case RCC_AHB:
        RCC_AHBPeriphResetCmd(mask, NewState);
        break;
#endif
    case RCC_APB2:
        RCC_APB2PeriphResetCmd(mask, NewState);
        break;
    case RCC_APB1:
        RCC_APB1PeriphResetCmd(mask, NewState);
        break;
#if defined(STM32F4)
    case RCC_AHB1:
        RCC_AHB1PeriphResetCmd(mask, NewState);
        break;
#endif
    }
#endif
}
