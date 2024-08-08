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

#pragma once

#include "rcc_types.h"

enum rcc_reg {
    RCC_EMPTY = 0,   // make sure that default value (0) does not enable anything
#ifdef STM32H7
    RCC_AHB,
    RCC_APB2,
    RCC_APB1L,
    RCC_APB1H,
    RCC_AHB1,
    RCC_AHB2,
    RCC_AHB3,
    RCC_APB3,
    RCC_AHB4,
    RCC_APB4,
#elif defined(STM32F7)
    RCC_AHB2,
    RCC_APB2,
    RCC_APB1,
    RCC_AHB1,
#elif defined(STM32G4)
    RCC_AHB2,
    RCC_APB2,
    RCC_APB11,
    RCC_APB12,
    RCC_AHB1,
#elif defined(AT32F4)
    RCC_AHB1,
    RCC_AHB2,
    RCC_AHB3,
    RCC_APB2,
    RCC_APB1,
#elif defined(APM32F4)
    RCC_AHB1,
    RCC_AHB2,
    RCC_AHB3,
    RCC_APB2,
    RCC_APB1,
#else
    RCC_AHB,
    RCC_APB2,
    RCC_APB1,
    RCC_AHB1,
#endif
};

#define RCC_ENCODE(reg, mask) (((reg) << 5) | LOG2_32BIT(mask))

#if defined(STM32F4) 
#define RCC_AHB(periph) RCC_ENCODE(RCC_AHB, RCC_AHBENR_ ## periph ## EN)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, RCC_APB2ENR_ ## periph ## EN)
#define RCC_APB1(periph) RCC_ENCODE(RCC_APB1, RCC_APB1ENR_ ## periph ## EN)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)
#elif defined(STM32H7)
#define RCC_AHB(periph) RCC_ENCODE(RCC_AHB, RCC_AHBENR_ ## periph ## EN)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, RCC_APB2ENR_ ## periph ## EN)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)
#define RCC_AHB2(periph) RCC_ENCODE(RCC_AHB2, RCC_AHB2ENR_ ## periph ## EN)
#define RCC_AHB3(periph) RCC_ENCODE(RCC_AHB3, RCC_AHB3ENR_ ## periph ## EN)
#define RCC_APB3(periph) RCC_ENCODE(RCC_APB3, RCC_APB3ENR_ ## periph ## EN)
#define RCC_AHB4(periph) RCC_ENCODE(RCC_AHB4, RCC_AHB4ENR_ ## periph ## EN)
#define RCC_APB4(periph) RCC_ENCODE(RCC_APB4, RCC_APB4ENR_ ## periph ## EN)
#define RCC_APB1L(periph) RCC_ENCODE(RCC_APB1L, RCC_APB1LENR_ ## periph ## EN)
#define RCC_APB1H(periph) RCC_ENCODE(RCC_APB1H, RCC_APB1HENR_ ## periph ## EN)
#elif defined(STM32G4)
#define RCC_AHB(periph) RCC_ENCODE(RCC_AHB, RCC_AHBENR_ ## periph ## EN)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, RCC_APB2ENR_ ## periph ## EN)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)
#define RCC_APB11(periph) RCC_ENCODE(RCC_APB11, RCC_APB1ENR1_ ## periph ## EN)
#define RCC_APB12(periph) RCC_ENCODE(RCC_APB12, RCC_APB1ENR2_ ## periph ## EN)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)
#define RCC_AHB2(periph) RCC_ENCODE(RCC_AHB2, RCC_AHB2ENR_ ## periph ## EN)
#elif defined(STM32F7)
#define RCC_AHB(periph) RCC_ENCODE(RCC_AHB, RCC_AHBENR_ ## periph ## EN)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, RCC_APB2ENR_ ## periph ## EN)
#define RCC_APB1(periph) RCC_ENCODE(RCC_APB1, RCC_APB1ENR_ ## periph ## EN)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)
#define RCC_AHB2(periph) RCC_ENCODE(RCC_AHB2, RCC_AHB2ENR_ ## periph ## EN)
#define RCC_APB1(periph) RCC_ENCODE(RCC_APB1, RCC_APB1ENR_ ## periph ## EN)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, RCC_APB2ENR_ ## periph ## EN)
#elif defined(AT32F4)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1,   CRM_AHB1_ ## periph ## _PER_MASK)
#define RCC_AHB2(periph) RCC_ENCODE(RCC_AHB2,   CRM_AHB2_ ## periph ## _PER_MASK)
#define RCC_AHB3(periph) RCC_ENCODE(RCC_AHB3,   CRM_AHB3_ ## periph ## _PER_MASK)
#define RCC_APB1(periph) RCC_ENCODE(RCC_APB1,   CRM_APB1_ ## periph ## _PER_MASK)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2,   CRM_APB2_ ## periph ## _PER_MASK)
#elif defined(APM32F4)
#define RCC_AHB(periph) RCC_ENCODE(RCC_AHB, RCM_AHBCLKEN_ ## periph ## EN)
#define RCC_APB2(periph) RCC_ENCODE(RCC_APB2, RCM_APB2CLKEN_ ## periph ## EN)
#define RCC_APB1(periph) RCC_ENCODE(RCC_APB1, RCM_APB1CLKEN_ ## periph ## EN)
#define RCC_AHB1(periph) RCC_ENCODE(RCC_AHB1, RCM_AHB1CLKEN_ ## periph ## EN)
#endif

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
