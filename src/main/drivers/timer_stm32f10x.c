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

#include "stm32f10x.h"
#include "rcc.h"
#include "timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1)  },
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2)  },
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3)  },
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4)  },
#if defined(STM32F10X_HD) || defined(STM32F10X_CL) || defined(STM32F10X_XL) || defined(STM32F10X_HD_VL)
    { .TIMx = TIM5,  .rcc = RCC_APB1(TIM5)  },
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6)  },
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7)  },
#endif
#if defined(STM32F10X_XL) || defined(STM32F10X_HD_VL)
    { .TIMx = TIM8,  .rcc = RCC_APB1(TIM8)  },
    { .TIMx = TIM9,  .rcc = RCC_APB2(TIM9)  },
    { .TIMx = TIM10, .rcc = RCC_APB2(TIM10) },
    { .TIMx = TIM11, .rcc = RCC_APB2(TIM11) },
    { .TIMx = TIM12, .rcc = RCC_APB1(TIM12) },
    { .TIMx = TIM13, .rcc = RCC_APB1(TIM13) },
    { .TIMx = TIM14, .rcc = RCC_APB1(TIM14) },
#endif
};

uint8_t timerClockDivisor(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return 1;
}
