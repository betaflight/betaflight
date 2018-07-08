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
    (void)tag;
    (void)mask;
    (void)NewState;
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
#if defined(USE_HAL_DRIVER)
    (void)tag;
    (void)mask;
    (void)NewState;
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
