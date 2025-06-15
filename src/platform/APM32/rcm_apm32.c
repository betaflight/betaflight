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

#include "platform.h"
#include "platform/rcc.h"

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

// Note on "suffix" macro parameter:
// ENR and RSTR naming conventions for buses with multiple registers per bus differs among MCU types.
// ST decided to use AxBn{L,H}ENR convention for H7 which can be handled with simple "ENR" (or "RSTR") contatenation,
// while use AxBnENR{1,2} convention for G4 which requires extra "suffix" to be concatenated.
// Here, we use "suffix" for all MCU types and leave it as empty where not applicable.

#define NOSUFFIX // Empty

#define __DAL_RCM_CLK_ENABLE(bus, suffix, enbit)   do {      \
        __IO uint32_t tmpreg;                                \
        SET_BIT(RCM->bus ## CLKEN ## suffix, enbit);           \
        /* Delay after an RCM peripheral clock enabling */   \
        tmpreg = READ_BIT(RCM->bus ## CLKEN ## suffix, enbit); \
        UNUSED(tmpreg);                                      \
    } while(0)

#define __DAL_RCM_CLK_DISABLE(bus, suffix, enbit) (RCM->bus ## CLKEN ## suffix &= ~(enbit))

#define __DAL_RCM_CLK(bus, suffix, enbit, newState) \
    if (newState == ENABLE) {                       \
        __DAL_RCM_CLK_ENABLE(bus, suffix, enbit);   \
    } else {                                        \
        __DAL_RCM_CLK_DISABLE(bus, suffix, enbit);  \
    }

    switch (tag) {
    case RCC_AHB1:
        __DAL_RCM_CLK(AHB1, NOSUFFIX, mask, NewState);
        break;

    case RCC_AHB2:
        __DAL_RCM_CLK(AHB2, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB1:
        __DAL_RCM_CLK(APB1, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB2:
        __DAL_RCM_CLK(APB2, NOSUFFIX, mask, NewState);
        break;

    case RCC_AHB3:
        __DAL_RCM_CLK(AHB3, NOSUFFIX, mask, NewState);
        break;
    }
}

void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

// Peripheral reset control relies on RST bits are identical to ENR bits where applicable

#define __DAL_RCM_FORCE_RESET(bus, suffix, enbit) (RCM->bus ## RST ## suffix |= (enbit))
#define __DAL_RCM_RELEASE_RESET(bus, suffix, enbit) (RCM->bus ## RST ## suffix &= ~(enbit))
#define __DAL_RCM_RESET(bus, suffix, enbit, NewState) \
    if (NewState == ENABLE) {                         \
        __DAL_RCM_RELEASE_RESET(bus, suffix, enbit);  \
    } else {                                          \
        __DAL_RCM_FORCE_RESET(bus, suffix, enbit);    \
    }

    switch (tag) {
    case RCC_AHB1:
        __DAL_RCM_RESET(AHB1, NOSUFFIX, mask, NewState);
        break;

    case RCC_AHB2:
        __DAL_RCM_RESET(AHB2, NOSUFFIX, mask, NewState);
        break;

    case RCC_AHB3:
        __DAL_RCM_RESET(AHB3, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB1:
        __DAL_RCM_RESET(APB1, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB2:
        __DAL_RCM_RESET(APB2, NOSUFFIX, mask, NewState);
        break;
    }
}
