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

#ifdef USE_TIMER

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/io.h"
#include "timer_def.h"

#include "stm32c5xx.h"
#include "platform/rcc.h"
#include "drivers/timer.h"
#include "platform/timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),   .inputIrq = TIM1_CC_IRQn},
    { .TIMx = TIM2,  .rcc = RCC_APB1L(TIM2),  .inputIrq = TIM2_IRQn},
#if !defined(STM32C562xx)
    { .TIMx = TIM3,  .rcc = RCC_APB1L(TIM3),  .inputIrq = TIM3_IRQn},
    { .TIMx = TIM4,  .rcc = RCC_APB1L(TIM4),  .inputIrq = TIM4_IRQn},
#endif
    { .TIMx = TIM5,  .rcc = RCC_APB1L(TIM5),  .inputIrq = TIM5_IRQn},
    { .TIMx = TIM6,  .rcc = RCC_APB1L(TIM6),  .inputIrq = TIM6_IRQn},
    { .TIMx = TIM7,  .rcc = RCC_APB1L(TIM7),  .inputIrq = TIM7_IRQn},
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),   .inputIrq = TIM8_CC_IRQn},
    { .TIMx = TIM12, .rcc = RCC_APB1L(TIM12), .inputIrq = TIM12_IRQn},
    { .TIMx = TIM15, .rcc = RCC_APB2(TIM15),  .inputIrq = TIM15_IRQn},
    { .TIMx = TIM16, .rcc = RCC_APB2(TIM16),  .inputIrq = TIM16_IRQn},
    { .TIMx = TIM17, .rcc = RCC_APB2(TIM17),  .inputIrq = TIM17_IRQn},
};

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
// Auto-generated from 'timer_def.h'
// Port A
    DEF_TIM(TIM2, CH1, PA0, 0, 0, 0),
    DEF_TIM(TIM2, CH2, PA1, 0, 0, 0),
    DEF_TIM(TIM2, CH3, PA2, 0, 0, 0),
    DEF_TIM(TIM2, CH4, PA3, 0, 0, 0),
    DEF_TIM(TIM5, CH1, PA0, 0, 0, 0),
    DEF_TIM(TIM5, CH2, PA1, 0, 0, 0),
    DEF_TIM(TIM5, CH3, PA2, 0, 0, 0),
    DEF_TIM(TIM5, CH4, PA3, 0, 0, 0),
#if !defined(STM32C562xx)
    DEF_TIM(TIM3, CH1, PA6, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PA7, 0, 0, 0),
#endif
    DEF_TIM(TIM1, CH1, PA8, 0, 0, 0),
    DEF_TIM(TIM1, CH2, PA9, 0, 0, 0),
    DEF_TIM(TIM1, CH3, PA10, 0, 0, 0),
    DEF_TIM(TIM1, CH4, PA11, 0, 0, 0),
// Port B
#if !defined(STM32C562xx)
    DEF_TIM(TIM3, CH3, PB0, 0, 0, 0),
    DEF_TIM(TIM3, CH4, PB1, 0, 0, 0),
    DEF_TIM(TIM4, CH1, PB6, 0, 0, 0),
    DEF_TIM(TIM4, CH2, PB7, 0, 0, 0),
    DEF_TIM(TIM4, CH3, PB8, 0, 0, 0),
    DEF_TIM(TIM4, CH4, PB9, 0, 0, 0),
#endif
    DEF_TIM(TIM2, CH3, PB10, 0, 0, 0),
    DEF_TIM(TIM2, CH4, PB11, 0, 0, 0),
// Port C
#if !defined(STM32C562xx)
    DEF_TIM(TIM3, CH1, PC6, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PC7, 0, 0, 0),
    DEF_TIM(TIM3, CH3, PC8, 0, 0, 0),
    DEF_TIM(TIM3, CH4, PC9, 0, 0, 0),
#endif
    DEF_TIM(TIM8, CH1, PC6, 0, 0, 0),
    DEF_TIM(TIM8, CH2, PC7, 0, 0, 0),
    DEF_TIM(TIM8, CH3, PC8, 0, 0, 0),
    DEF_TIM(TIM8, CH4, PC9, 0, 0, 0),
};
#endif

uint32_t timerClockFromInstance(const timerResource_t *tim)
{
    uint32_t pclk;
    uint32_t ppre;

    if ((uintptr_t)tim >= APB2PERIPH_BASE) {
        pclk = HAL_RCC_GetPCLK2Freq();
        ppre = (RCC->CFGR2 & RCC_CFGR2_PPRE2) >> RCC_CFGR2_PPRE2_Pos;
    } else {
        pclk = HAL_RCC_GetPCLK1Freq();
        ppre = (RCC->CFGR2 & RCC_CFGR2_PPRE1) >> RCC_CFGR2_PPRE1_Pos;
    }

    // Standard STM32 rule: if APB prescaler > 1 (bit 2 set), timer clock = 2 * PCLK
    return (ppre & 0x04) ? pclk * 2 : pclk;
}

uint32_t timerClock(const timerHardware_t *timHw)
{
    return timerClockFromInstance(timHw->tim);
}
#endif
