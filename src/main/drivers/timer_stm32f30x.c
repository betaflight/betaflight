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

#include "platform.h"

#include "common/utils.h"

#include "stm32f30x.h"
#include "rcc.h"
#include "timer.h"

#define TIM_N(n) (1 << (n))
#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)

const timerDef_t timerDefinitions[USED_TIMER_COUNT] = {
#if USED_TIMERS & TIM_N(1)
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),  .inputIrq = TIM1_CC_IRQn },
#endif
#if USED_TIMERS & TIM_N(2)
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2),  .inputIrq = TIM2_IRQn },
#endif
#if USED_TIMERS & TIM_N(3)
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3),  .inputIrq = TIM3_IRQn },
#endif
#if USED_TIMERS & TIM_N(4)
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4),  .inputIrq = TIM4_IRQn },
#endif
#if USED_TIMERS & TIM_N(6)
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6),  .inputIrq = 0 },
#endif
#if USED_TIMERS & TIM_N(7)
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7),  .inputIrq = 0 },
#endif
#if USED_TIMERS & TIM_N(8)
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),  .inputIrq = TIM8_CC_IRQn },
#endif
#if USED_TIMERS & TIM_N(15)
    { .TIMx = TIM15, .rcc = RCC_APB2(TIM15), .inputIrq = TIM1_BRK_TIM15_IRQn },
#endif
#if USED_TIMERS & TIM_N(16)
    { .TIMx = TIM16, .rcc = RCC_APB2(TIM16), .inputIrq = TIM1_UP_TIM16_IRQn },
#endif
#if USED_TIMERS & TIM_N(17)
    { .TIMx = TIM17, .rcc = RCC_APB2(TIM17), .inputIrq = TIM1_TRG_COM_TIM17_IRQn },
#endif
};

uint32_t timerClock(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return SystemCoreClock;
}
