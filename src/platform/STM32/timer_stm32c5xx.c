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
// Generated from the STM32C5 DFP pinout descriptors
// Port A
    DEF_TIM(TIM2, CH1, PA0, 0, 0, 0),    // AF1
    DEF_TIM(TIM5, CH1, PA0, 0, 0, 0),    // AF2
    DEF_TIM(TIM2, CH2, PA1, 0, 0, 0),    // AF1
    DEF_TIM(TIM5, CH2, PA1, 0, 0, 0),    // AF2
    DEF_TIM(TIM15, CH1N, PA1, 0, 0, 0),  // AF4
    DEF_TIM(TIM2, CH3, PA2, 0, 0, 0),    // AF1
    DEF_TIM(TIM5, CH3, PA2, 0, 0, 0),    // AF2
    DEF_TIM(TIM15, CH1, PA2, 0, 0, 0),   // AF4
    DEF_TIM(TIM2, CH4, PA3, 0, 0, 0),    // AF1
    DEF_TIM(TIM15, CH2, PA3, 0, 0, 0),   // AF4
    DEF_TIM(TIM2, CH1, PA5, 0, 0, 0),    // AF1
    DEF_TIM(TIM1, CH3, PA5, 0, 0, 0),    // AF2
    DEF_TIM(TIM8, CH1N, PA5, 0, 0, 0),   // AF3
    DEF_TIM(TIM1, CH1N, PA7, 0, 0, 0),   // AF1
    DEF_TIM(TIM8, CH1N, PA7, 0, 0, 0),   // AF3
    DEF_TIM(TIM15, CH1, PA7, 0, 0, 0),   // AF4
    DEF_TIM(TIM1, CH1, PA8, 0, 0, 0),    // AF1
    DEF_TIM(TIM15, CH2, PA8, 0, 0, 0),   // AF4
    DEF_TIM(TIM5, CH4, PA8, 0, 0, 0),    // AF8
    DEF_TIM(TIM2, CH4, PA8, 0, 0, 0),    // AF14
    DEF_TIM(TIM1, CH2, PA9, 0, 0, 0),    // AF1
    DEF_TIM(TIM15, CH1N, PA9, 0, 0, 0),  // AF4
    DEF_TIM(TIM8, CH2N, PA9, 0, 0, 0),   // AF13
    DEF_TIM(TIM1, CH3, PA10, 0, 0, 0),   // AF1
    DEF_TIM(TIM1, CH4, PA11, 0, 0, 0),   // AF1
    DEF_TIM(TIM2, CH1, PA15, 0, 0, 0),   // AF1
    DEF_TIM(TIM1, CH2N, PA15, 0, 0, 0),  // AF2
    DEF_TIM(TIM8, CH4N, PA15, 0, 0, 0),  // AF13
#if defined(STM32C562xx)
    DEF_TIM(TIM5, CH4, PA3, 0, 0, 0),    // AF2
    DEF_TIM(TIM5, CH1, PA6, 0, 0, 0),    // AF2
    DEF_TIM(TIM5, CH2, PA7, 0, 0, 0),    // AF2
#else
    DEF_TIM(TIM3, CH3, PA3, 0, 0, 0),    // AF2
    DEF_TIM(TIM5, CH4, PA3, 0, 0, 0),    // AF8
    DEF_TIM(TIM3, CH4, PA4, 0, 0, 0),    // AF2
    DEF_TIM(TIM3, CH1, PA6, 0, 0, 0),    // AF2
    DEF_TIM(TIM3, CH2, PA7, 0, 0, 0),    // AF2
#endif
// Port B
    DEF_TIM(TIM1, CH2N, PB0, 0, 0, 0),   // AF1
    DEF_TIM(TIM8, CH2N, PB0, 0, 0, 0),   // AF3
    DEF_TIM(TIM1, CH3N, PB1, 0, 0, 0),   // AF1
    DEF_TIM(TIM8, CH3N, PB1, 0, 0, 0),   // AF3
    DEF_TIM(TIM8, CH4N, PB2, 0, 0, 0),   // AF3
    DEF_TIM(TIM2, CH2, PB3, 0, 0, 0),    // AF1
    DEF_TIM(TIM5, CH3, PB3, 0, 0, 0),    // AF3
    DEF_TIM(TIM8, CH1, PB3, 0, 0, 0),    // AF13
    DEF_TIM(TIM8, CH2, PB4, 0, 0, 0),    // AF13
    DEF_TIM(TIM8, CH3, PB5, 0, 0, 0),    // AF13
    DEF_TIM(TIM16, CH1N, PB6, 0, 0, 0),  // AF10
    DEF_TIM(TIM8, CH4, PB6, 0, 0, 0),    // AF13
    DEF_TIM(TIM17, CH1N, PB7, 0, 0, 0),  // AF1
    DEF_TIM(TIM16, CH1, PB7, 0, 0, 0),   // AF10
    DEF_TIM(TIM17, CH1, PB8, 0, 0, 0),   // AF1
    DEF_TIM(TIM2, CH3, PB10, 0, 0, 0),   // AF1
    DEF_TIM(TIM8, CH1, PB10, 0, 0, 0),   // AF2
    DEF_TIM(TIM8, CH3, PB12, 0, 0, 0),   // AF2
    DEF_TIM(TIM1, CH1N, PB13, 0, 0, 0),  // AF1
    DEF_TIM(TIM8, CH2, PB13, 0, 0, 0),   // AF2
    DEF_TIM(TIM1, CH2N, PB14, 0, 0, 0),  // AF1
    DEF_TIM(TIM12, CH1, PB14, 0, 0, 0),  // AF2
    DEF_TIM(TIM8, CH2N, PB14, 0, 0, 0),  // AF3
    DEF_TIM(TIM1, CH3N, PB15, 0, 0, 0),  // AF1
    DEF_TIM(TIM12, CH2, PB15, 0, 0, 0),  // AF2
    DEF_TIM(TIM8, CH3N, PB15, 0, 0, 0),  // AF3
#if defined(STM32C562xx)
    DEF_TIM(TIM5, CH3, PB0, 0, 0, 0),    // AF2
    DEF_TIM(TIM5, CH4, PB1, 0, 0, 0),    // AF2
    DEF_TIM(TIM5, CH1, PB4, 0, 0, 0),    // AF2
    DEF_TIM(TIM5, CH2, PB5, 0, 0, 0),    // AF2
#else
    DEF_TIM(TIM3, CH3, PB0, 0, 0, 0),    // AF2
    DEF_TIM(TIM3, CH4, PB1, 0, 0, 0),    // AF2
    DEF_TIM(TIM3, CH1, PB4, 0, 0, 0),    // AF2
    DEF_TIM(TIM3, CH2, PB5, 0, 0, 0),    // AF2
    DEF_TIM(TIM4, CH1, PB6, 0, 0, 0),    // AF2
    DEF_TIM(TIM4, CH2, PB7, 0, 0, 0),    // AF2
    DEF_TIM(TIM4, CH3, PB8, 0, 0, 0),    // AF2
    DEF_TIM(TIM4, CH4, PB9, 0, 0, 0),    // AF2
#endif
// Port C
    DEF_TIM(TIM17, CH1, PC2, 0, 0, 0),   // AF1
    DEF_TIM(TIM2, CH4, PC4, 0, 0, 0),    // AF1
    DEF_TIM(TIM16, CH1, PC4, 0, 0, 0),   // AF10
    DEF_TIM(TIM1, CH4N, PC5, 0, 0, 0),   // AF1
    DEF_TIM(TIM16, CH1N, PC5, 0, 0, 0),  // AF10
    DEF_TIM(TIM8, CH1, PC6, 0, 0, 0),    // AF3
    DEF_TIM(TIM8, CH2, PC7, 0, 0, 0),    // AF3
    DEF_TIM(TIM8, CH3, PC8, 0, 0, 0),    // AF3
    DEF_TIM(TIM8, CH4, PC9, 0, 0, 0),    // AF3
    DEF_TIM(TIM8, CH1N, PC10, 0, 0, 0),  // AF3
    DEF_TIM(TIM8, CH2N, PC11, 0, 0, 0),  // AF3
    DEF_TIM(TIM15, CH1, PC12, 0, 0, 0),  // AF2
    DEF_TIM(TIM8, CH3N, PC12, 0, 0, 0),  // AF3
#if defined(STM32C562xx)
    DEF_TIM(TIM5, CH1, PC6, 0, 0, 0),    // AF2
    DEF_TIM(TIM5, CH2, PC7, 0, 0, 0),    // AF2
    DEF_TIM(TIM5, CH3, PC8, 0, 0, 0),    // AF2
    DEF_TIM(TIM5, CH4, PC9, 0, 0, 0),    // AF2
#else
    DEF_TIM(TIM4, CH4, PC2, 0, 0, 0),    // AF2
    DEF_TIM(TIM3, CH1, PC6, 0, 0, 0),    // AF2
    DEF_TIM(TIM3, CH2, PC7, 0, 0, 0),    // AF2
    DEF_TIM(TIM3, CH3, PC8, 0, 0, 0),    // AF2
    DEF_TIM(TIM3, CH4, PC9, 0, 0, 0),    // AF2
#endif
// Port D
    DEF_TIM(TIM8, CH4N, PD0, 0, 0, 0),   // AF3
    DEF_TIM(TIM1, CH4N, PD5, 0, 0, 0),   // AF1
    DEF_TIM(TIM8, CH1N, PD12, 0, 0, 0),  // AF3
    DEF_TIM(TIM8, CH2N, PD13, 0, 0, 0),  // AF3
    DEF_TIM(TIM8, CH3N, PD14, 0, 0, 0),  // AF3
    DEF_TIM(TIM8, CH4N, PD15, 0, 0, 0),  // AF3
#if !defined(STM32C562xx)
    DEF_TIM(TIM4, CH1, PD12, 0, 0, 0),   // AF2
    DEF_TIM(TIM4, CH2, PD13, 0, 0, 0),   // AF2
    DEF_TIM(TIM4, CH3, PD14, 0, 0, 0),   // AF2
    DEF_TIM(TIM4, CH4, PD15, 0, 0, 0),   // AF2
#endif
// Port E
    DEF_TIM(TIM15, CH1N, PE4, 0, 0, 0),  // AF4
    DEF_TIM(TIM15, CH1, PE5, 0, 0, 0),   // AF4
    DEF_TIM(TIM15, CH2, PE6, 0, 0, 0),   // AF4
    DEF_TIM(TIM1, CH1N, PE8, 0, 0, 0),   // AF1
    DEF_TIM(TIM1, CH1, PE9, 0, 0, 0),    // AF1
    DEF_TIM(TIM1, CH2N, PE10, 0, 0, 0),  // AF1
    DEF_TIM(TIM1, CH2, PE11, 0, 0, 0),   // AF1
    DEF_TIM(TIM1, CH3N, PE12, 0, 0, 0),  // AF1
    DEF_TIM(TIM1, CH3, PE13, 0, 0, 0),   // AF1
    DEF_TIM(TIM1, CH4, PE14, 0, 0, 0),   // AF1
    DEF_TIM(TIM1, CH4N, PE15, 0, 0, 0),  // AF3
#if !defined(STM32C562xx)
// Port F
    DEF_TIM(TIM16, CH1, PF6, 0, 0, 0),   // AF10
    DEF_TIM(TIM17, CH1, PF7, 0, 0, 0),   // AF1
    DEF_TIM(TIM16, CH1N, PF8, 0, 0, 0),  // AF10
    DEF_TIM(TIM17, CH1N, PF9, 0, 0, 0),  // AF1
#endif
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
