/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <platform.h>

#include "common/utils.h"

#include "stm32f10x.h"
#include "rcc.h"
#include "timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1), .inputIrq = TIM1_CC_IRQn },
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2), .inputIrq = TIM2_IRQn },
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3), .inputIrq = TIM3_IRQn },
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4), .inputIrq = TIM4_IRQn },
#if defined(STM32F10X_HD) || defined(STM32F10X_CL) || defined(STM32F10X_XL) || defined(STM32F10X_HD_VL)
    { .TIMx = TIM5,  .rcc = RCC_APB1(TIM5), .inputIrq = TIM5_IRQn },
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6), .inputIrq = 0 },
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7), .inputIrq = 0 },
#endif
#if defined(STM32F10X_XL) || defined(STM32F10X_HD_VL)
    { .TIMx = TIM8,  .rcc = RCC_APB1(TIM8),  .inputIrq = TIM8_CC_IRQn },
    { .TIMx = TIM9,  .rcc = RCC_APB2(TIM9),  .inputIrq = TIM1_BRK_TIM9_IRQn },
    { .TIMx = TIM10, .rcc = RCC_APB2(TIM10), .inputIrq = TIM1_UP_TIM10_IRQn },
    { .TIMx = TIM11, .rcc = RCC_APB2(TIM11), .inputIrq = TIM1_TRG_COM_TIM11_IRQn },
    { .TIMx = TIM12, .rcc = RCC_APB1(TIM12), .inputIrq = TIM12_IRQn },
    { .TIMx = TIM13, .rcc = RCC_APB1(TIM13), .inputIrq = TIM13_IRQn },
    { .TIMx = TIM14, .rcc = RCC_APB1(TIM14), .inputIrq = TIM14_IRQn },
#endif
};

uint8_t timerClockDivisor(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return 1;
}
