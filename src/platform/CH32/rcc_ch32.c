/*
 * This file is part of Cleanflight and Betaflight and JustFlight.
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
 * this file define the ch32h41x clock enable/reset functions
 * porting for ch32h41x by Temperslee
 *
 */

#include "platform.h"
#include "platform/rcc.h"

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

#define NOSUFFIX // Empty

#define __RCC_CLK_ENABLE(bus, suffix, enbit)   do {      \
        __IO uint32_t tmpreg;                                \
        SET_BIT(RCC->bus ## PCENR ## suffix, enbit);            \
        tmpreg = READ_BIT(RCC->bus ## PCENR ## suffix, enbit);  \
        UNUSED(tmpreg);                                      \
    } while(0)

#define __RCC_CLK_DISABLE(bus, suffix, enbit) (RCC->bus ## PCENR ## suffix &= ~(enbit))

#define __RCC_CLK(bus, suffix, enbit, newState) \
    if (newState == ENABLE) {                       \
        __RCC_CLK_ENABLE(bus, suffix, enbit);   \
    } else {                                        \
        __RCC_CLK_DISABLE(bus, suffix, enbit);  \
    }

    switch (tag) {
        case RCC_HB:
            __RCC_CLK(HB, NOSUFFIX, mask, NewState);
            break;
        case RCC_HB2:
            __RCC_CLK(HB2, NOSUFFIX, mask, NewState);
            break;
        case RCC_HB1:
            __RCC_CLK(HB1, NOSUFFIX, mask, NewState);
            break;
        // case RCC_APB1:
        //     __RCC_CLK(apb1, NOSUFFIX, mask, NewState);
        //     break;
        // case RCC_APB2:
        //     __RCC_CLK(apb2, NOSUFFIX, mask, NewState);
        //     break;
    }
}

void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

#define __RCC_FORCE_RESET(bus, suffix, enbit) (RCC->bus ## PRSTR ## suffix |= (enbit))
#define __RCC_RELEASE_RESET(bus, suffix, enbit) (RCC->bus ## PRSTR ## suffix &= ~(enbit))
#define __RCC_RESET(bus, suffix, enbit, NewState) \
    if (NewState == ENABLE) {                         \
        __RCC_RELEASE_RESET(bus, suffix, enbit);  \
    } else {                                          \
        __RCC_FORCE_RESET(bus, suffix, enbit);    \
    }

    switch (tag) {
        case RCC_HB:
            __RCC_RESET(HB, NOSUFFIX, mask, NewState);
            break;

        case RCC_HB2:
            __RCC_RESET(HB2, NOSUFFIX, mask, NewState);
            break;

        case RCC_HB1:
            __RCC_RESET(HB1, NOSUFFIX, mask, NewState);
            break;
        // case RCC_APB1:
        //     __RCC_RESET(apb1, NOSUFFIX, mask, NewState);
        //     break;

        // case RCC_APB2:
        //     __RCC_RESET(apb2, NOSUFFIX, mask, NewState);
        //     break;
    }
}
