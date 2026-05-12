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
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform/rcc_types.h"

enum rcc_reg {
    RCC_EMPTY = 0,
    RCC_AHB1_1,
    RCC_AHB1_2,
    RCC_AHB1_3,
    RCC_AHB1_4,
    RCC_AHB2_1,
    RCC_AHB2_2,
    RCC_AHB5_1,
    RCC_AHB5_2,
    RCC_AHB9_1,
    RCC_APB1_1,
    RCC_APB1_2,
    RCC_APB1_3,
    RCC_APB1_4,
    RCC_APB1_5,
    RCC_APB2_1,
    RCC_APB2_2,
    RCC_APB2_3,
    RCC_APB2_4,
    RCC_APB5_1,
    RCC_APB5_2,
};

#define RCC_ENCODE(reg, mask) (((reg) << 5) | LOG2_32BIT(mask))

#define RCC_AHB1_1(periph) RCC_ENCODE(RCC_AHB1_1, RCC_AHB1_PERIPHEN_M7_ ## periph)
#define RCC_AHB1_2(periph) RCC_ENCODE(RCC_AHB1_2, RCC_AHB1_PERIPHEN_M7_ ## periph)
#define RCC_AHB1_3(periph) RCC_ENCODE(RCC_AHB1_3, RCC_AHB1_PERIPHEN_M7_ ## periph)
#define RCC_AHB1_4(periph) RCC_ENCODE(RCC_AHB1_4, RCC_AHB1_PERIPHEN_M7_ ## periph)

#define RCC_AHB2_1(periph) RCC_ENCODE(RCC_AHB2_1, RCC_AHB2_PERIPHEN_M7_ ## periph)
#define RCC_AHB2_2(periph) RCC_ENCODE(RCC_AHB2_2, RCC_AHB2_PERIPHEN_M7_ ## periph)

#define RCC_AHB5_1(periph) RCC_ENCODE(RCC_AHB5_1, RCC_AHB5_PERIPHEN_M7_ ## periph)
#define RCC_AHB5_2(periph) RCC_ENCODE(RCC_AHB5_2, RCC_AHB5_PERIPHEN_M7_ ## periph)

#define RCC_AHB9_1(periph) RCC_ENCODE(RCC_AHB9_1, RCC_AHB9_PERIPHEN_M7_ ## periph)

#define RCC_APB1_1(periph) RCC_ENCODE(RCC_APB1_1, RCC_APB1_PERIPHEN_M7_ ## periph)
#define RCC_APB1_2(periph) RCC_ENCODE(RCC_APB1_2, RCC_APB1_PERIPHEN_M7_ ## periph)
#define RCC_APB1_3(periph) RCC_ENCODE(RCC_APB1_3, RCC_APB1_PERIPHEN_M7_ ## periph)
#define RCC_APB1_4(periph) RCC_ENCODE(RCC_APB1_4, RCC_APB1_PERIPHEN_M7_ ## periph)
#define RCC_APB1_5(periph) RCC_ENCODE(RCC_APB1_5, RCC_APB1_PERIPHEN_M7_ ## periph)

#define RCC_APB2_1(periph) RCC_ENCODE(RCC_APB2_1, RCC_APB2_PERIPHEN_M7_ ## periph)
#define RCC_APB2_2(periph) RCC_ENCODE(RCC_APB2_2, RCC_APB2_PERIPHEN_M7_ ## periph)
#define RCC_APB2_3(periph) RCC_ENCODE(RCC_APB2_3, RCC_APB2_PERIPHEN_M7_ ## periph)
#define RCC_APB2_4(periph) RCC_ENCODE(RCC_APB2_4, RCC_APB2_PERIPHEN_M7_ ## periph)

#define RCC_APB5_1(periph) RCC_ENCODE(RCC_APB5_1, RCC_APB5_PERIPHEN_M7_ ## periph)
#define RCC_APB5_2(periph) RCC_ENCODE(RCC_APB5_2, RCC_APB5_PERIPHEN_M7_ ## periph)

void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState);
