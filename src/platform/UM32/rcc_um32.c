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
#include "platform/rcc.h"

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
	
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);


#if defined(USE_HAL_DRIVER)

// Note on "suffix" macro parameter:
// ENR and RSTR naming conventions for buses with multiple registers per bus differs among MCU types.
// ST decided to use AxBn{L,H}ENR convention for H7 which can be handled with simple "ENR" (or "RSTR") contatenation,
// while use AxBnENR{1,2} convention for G4 which requires extra "suffix" to be concatenated.
// Here, we use "suffix" for all MCU types and leave it as empty where not applicable.

#define NOSUFFIX // Empty

#define __HAL_RCC_CLK_ENABLE(bus, suffix, enbit)   do {      		\
        __IO uint32_t tmpreg;                                		\
				__HAL_RCM_UNLOCK_REGISTER();														\
        SET_BIT(RCM->bus ## CKENR ## suffix, enbit);          	\
				__HAL_RCM_LOCK_REGISTER();															\
        /* Delay after an RCC peripheral clock enabling */   		\
        tmpreg = READ_BIT(RCM->bus ## CKENR ## suffix, enbit); 	\
        UNUSED(tmpreg);                                      		\
    } while(0)

#define __HAL_RCC_CLK_DISABLE(bus, suffix, enbit) do {      		\
				__HAL_RCM_UNLOCK_REGISTER();														\
				CLEAR_BIT(RCM->bus ## CKENR ## suffix, enbit);          \
				__HAL_RCM_LOCK_REGISTER();															\
		} while(0)

#define __HAL_RCC_CLK(bus, suffix, enbit, newState) \
    if (newState == ENABLE) {                       \
        __HAL_RCC_CLK_ENABLE(bus, suffix, enbit);   \
    } else {                                        \
        __HAL_RCC_CLK_DISABLE(bus, suffix, enbit);  \
    }

		
    switch (tag) {
    case RCC_AHB:
        __HAL_RCC_CLK(AHB, NOSUFFIX, mask, NewState);
        break;
		
    case RCC_AHB0:
        __HAL_RCC_CLK(AHB0, NOSUFFIX, mask, NewState);
        break;

    case RCC_AHB1:
        __HAL_RCC_CLK(AHB1, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB0:
        __HAL_RCC_CLK(APB0, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB1:
        __HAL_RCC_CLK(APB1, NOSUFFIX, mask, NewState);
        break;
		
    case RCC_APB2:
        __HAL_RCC_CLK(APB2, NOSUFFIX, mask, NewState);
        break;
		
    case RCC_APB3:
        __HAL_RCC_CLK(APB3, NOSUFFIX, mask, NewState);
        break;
    }


#endif
}

void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

// Peripheral reset control relies on RSTR bits are identical to ENR bits where applicable

#define __HAL_RCC_FORCE_RESET(bus, suffix, enbit) do {      				\
											__HAL_RCM_UNLOCK_REGISTER();									\
											SET_BIT(RCM->bus ## RSTR ## suffix, enbit);		\
											__HAL_RCM_LOCK_REGISTER();										\
										} while(0)

#define __HAL_RCC_RELEASE_RESET(bus, suffix, enbit) do {      			\
											__HAL_RCM_UNLOCK_REGISTER();									\
											CLEAR_BIT(RCM->bus ## RSTR ## suffix,enbit);	\
											__HAL_RCM_LOCK_REGISTER();										\
										} while(0)

#define __HAL_RCC_RESET(bus, suffix, enbit, NewState) \
    if (NewState == ENABLE) {                         \
        __HAL_RCC_RELEASE_RESET(bus, suffix, enbit);  \
    } else {                                          \
        __HAL_RCC_FORCE_RESET(bus, suffix, enbit);    \
    }

#if defined(USE_HAL_DRIVER)

    switch (tag) {
    case RCC_AHB:
        __HAL_RCC_RESET(AHB, NOSUFFIX, mask, NewState);
        break;
		
    case RCC_AHB0:
        __HAL_RCC_RESET(AHB0, NOSUFFIX, mask, NewState);
        break;
		
    case RCC_AHB1:
        __HAL_RCC_RESET(AHB1, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB0:
        __HAL_RCC_RESET(APB0, NOSUFFIX, mask, NewState);
        break;
		
    case RCC_APB1:
        __HAL_RCC_RESET(APB1, NOSUFFIX, mask, NewState);
        break;

    case RCC_APB2:
        __HAL_RCC_RESET(APB2, NOSUFFIX, mask, NewState);
        break;
		
    case RCC_APB3:
        __HAL_RCC_RESET(APB3, NOSUFFIX, mask, NewState);
        break;
    }
		
#endif
}
