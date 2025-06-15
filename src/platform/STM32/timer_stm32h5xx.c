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

#include "stm32h5xx.h"
#include "platform/rcc.h"
#include "drivers/timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),   .inputIrq = TIM1_CC_IRQn},
    { .TIMx = TIM2,  .rcc = RCC_APB1L(TIM2),  .inputIrq = TIM2_IRQn},
    { .TIMx = TIM3,  .rcc = RCC_APB1L(TIM3),  .inputIrq = TIM3_IRQn},
    { .TIMx = TIM4,  .rcc = RCC_APB1L(TIM4),  .inputIrq = TIM4_IRQn},
    { .TIMx = TIM5,  .rcc = RCC_APB1L(TIM5),  .inputIrq = TIM5_IRQn},
    { .TIMx = TIM6,  .rcc = RCC_APB1L(TIM6),  .inputIrq = TIM6_DAC_IRQn},
    { .TIMx = TIM7,  .rcc = RCC_APB1L(TIM7),  .inputIrq = TIM7_IRQn},
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),   .inputIrq = TIM8_CC_IRQn},
    { .TIMx = TIM12, .rcc = RCC_APB1L(TIM12), .inputIrq = TIM8_BRK_TIM12_IRQn},
    { .TIMx = TIM13, .rcc = RCC_APB1L(TIM13), .inputIrq = TIM8_UP_TIM13_IRQn},
    { .TIMx = TIM14, .rcc = RCC_APB1L(TIM14), .inputIrq = TIM8_TRG_COM_TIM14_IRQn},
    { .TIMx = TIM15, .rcc = RCC_APB2(TIM15),  .inputIrq = TIM15_IRQn},
    { .TIMx = TIM16, .rcc = RCC_APB2(TIM16),  .inputIrq = TIM16_IRQn},
    { .TIMx = TIM17, .rcc = RCC_APB2(TIM17),  .inputIrq = TIM17_IRQn},
};

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
// Auto-generated from 'timer_def.h'
// Port A
    DEF_TIM(TIM2, CH1, PA0, 0, 0, 0), // AF1
    DEF_TIM(TIM2, CH2, PA1, 0, 0, 0),
    DEF_TIM(TIM2, CH3, PA2, 0, 0, 0),
    DEF_TIM(TIM2, CH4, PA3, 0, 0, 0),
    DEF_TIM(TIM2, CH1, PA5, 0, 0, 0),
    DEF_TIM(TIM1, CH1N, PA7, 0, 0, 0),
    DEF_TIM(TIM1, CH1, PA8, 0, 0, 0),
    DEF_TIM(TIM1, CH2, PA9, 0, 0, 0),
    DEF_TIM(TIM1, CH3, PA10, 0, 0, 0),
    DEF_TIM(TIM1, CH4, PA11, 0, 0, 0),
    DEF_TIM(TIM2, CH1, PA15, 0, 0, 0),

    DEF_TIM(TIM5, CH1, PA0, 0, 0, 0), // AF2
    DEF_TIM(TIM5, CH2, PA1, 0, 0, 0),
    DEF_TIM(TIM5, CH3, PA2, 0, 0, 0),
    DEF_TIM(TIM5, CH4, PA3, 0, 0, 0),
    DEF_TIM(TIM3, CH1, PA6, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PA7, 0, 0, 0),

    DEF_TIM(TIM8, CH1N, PA5, 0, 0, 0), // AF3
    DEF_TIM(TIM8, CH1N, PA7, 0, 0, 0),

    DEF_TIM(TIM13, CH1, PA6, 0, 0, 0), // AF9
    DEF_TIM(TIM14, CH1, PA7, 0, 0, 0),

    DEF_TIM(TIM15, CH1N, PA1, 0, 0, 0), // AF4
    DEF_TIM(TIM15, CH1, PA2, 0, 0, 0),
    DEF_TIM(TIM15, CH2, PA3, 0, 0, 0),

// Port B
    DEF_TIM(TIM1, CH2N, PB0, 0, 0, 0), // AF1
    DEF_TIM(TIM1, CH3N, PB1, 0, 0, 0),
    DEF_TIM(TIM2, CH2, PB3, 0, 0, 0),
    DEF_TIM(TIM16, CH1N, PB6, 0, 0, 0),
    DEF_TIM(TIM17, CH1N, PB7, 0, 0, 0),
    DEF_TIM(TIM16, CH1, PB8, 0, 0, 0),
    DEF_TIM(TIM17, CH1, PB9, 0, 0, 0),
    DEF_TIM(TIM2, CH3, PB10, 0, 0, 0),
    DEF_TIM(TIM2, CH4, PB11, 0, 0, 0),
    DEF_TIM(TIM1, CH1N, PB13, 0, 0, 0),
    DEF_TIM(TIM1, CH2N, PB14, 0, 0, 0),
    DEF_TIM(TIM1, CH3N, PB15, 0, 0, 0),

    DEF_TIM(TIM3, CH3, PB0, 0, 0, 0), // AF2
    DEF_TIM(TIM3, CH4, PB1, 0, 0, 0),
    DEF_TIM(TIM3, CH1, PB4, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PB5, 0, 0, 0),
    DEF_TIM(TIM4, CH1, PB6, 0, 0, 0),
    DEF_TIM(TIM4, CH2, PB7, 0, 0, 0),
    DEF_TIM(TIM4, CH3, PB8, 0, 0, 0),
    DEF_TIM(TIM4, CH4, PB9, 0, 0, 0),

    // DEF_TIM(TIM12, CH1, PB14, 0, 0, 0), // AF9 (SDMMC2_D0)
    // DEF_TIM(TIM12, CH2, PB15, 0, 0, 0), // AF9 (SDMMC2_D1)

// Port C
    DEF_TIM(TIM3, CH1, PC6, 0, 0, 0), // AF2
    DEF_TIM(TIM3, CH2, PC7, 0, 0, 0),
    DEF_TIM(TIM3, CH3, PC8, 0, 0, 0),
    DEF_TIM(TIM3, CH4, PC9, 0, 0, 0),

    DEF_TIM(TIM8, CH1, PC6, 0, 0, 0), // AF3
    DEF_TIM(TIM8, CH2, PC7, 0, 0, 0),
    DEF_TIM(TIM8, CH3, PC8, 0, 0, 0),
    DEF_TIM(TIM8, CH4, PC9, 0, 0, 0),

// Port D
    DEF_TIM(TIM1, CH4N, PD5, 0, 0, 0), // AF1 (ADDED)

    DEF_TIM(TIM4, CH1, PD12, 0, 0, 0), // AF2
    DEF_TIM(TIM4, CH2, PD13, 0, 0, 0),
    DEF_TIM(TIM4, CH3, PD14, 0, 0, 0),
    DEF_TIM(TIM4, CH4, PD15, 0, 0, 0),

// Port E
    DEF_TIM(TIM1, CH1N, PE8, 0, 0, 0), // AF1
    DEF_TIM(TIM1, CH1, PE9, 0, 0, 0),
    DEF_TIM(TIM1, CH2N, PE10, 0, 0, 0),
    DEF_TIM(TIM1, CH2, PE11, 0, 0, 0),
    DEF_TIM(TIM1, CH3N, PE12, 0, 0, 0),
    DEF_TIM(TIM1, CH3, PE13, 0, 0, 0),
    DEF_TIM(TIM1, CH4, PE14, 0, 0, 0),

    DEF_TIM(TIM1, CH4N, PE15, 0, 0, 0), // AF3 (ADDED)

    DEF_TIM(TIM15, CH1N, PE4, 0, 0, 0), // AF4
    DEF_TIM(TIM15, CH1, PE5, 0, 0, 0),
    DEF_TIM(TIM15, CH2, PE6, 0, 0, 0),

// Port F
    DEF_TIM(TIM16, CH1, PF6, 0, 0, 0), // AF1
    DEF_TIM(TIM17, CH1, PF7, 0, 0, 0),
    DEF_TIM(TIM16, CH1N, PF8, 0, 0, 0), // AF3
    DEF_TIM(TIM17, CH1N, PF9, 0, 0, 0),

    DEF_TIM(TIM13, CH1, PF8, 0, 0, 0), // AF9 (WAS TIM13_CH1N)
    DEF_TIM(TIM14, CH1, PF9, 0, 0, 0), // AF9 (WAS TIM14_CH1N)

// Port H

    DEF_TIM(TIM1, CHN3, PH6, 0, 0, 0), // AF1 (ADDED)
    DEF_TIM(TIM1, CH3, PH7, 0, 0, 0),
    DEF_TIM(TIM1, CHN2, PH8, 0, 0, 0),
    DEF_TIM(TIM1, CH2, PH9, 0, 0, 0),
    DEF_TIM(TIM1, CHN1, PH10, 0, 0, 0),
    DEF_TIM(TIM1, CH1, PH11, 0, 0, 0),

    DEF_TIM(TIM12, CH1, PH6, 0, 0, 0), // AF2 (ADDED)
    DEF_TIM(TIM12, CH2, PH9, 0, 0, 0),
    DEF_TIM(TIM5, CH1, PH10, 0, 0, 0),
    DEF_TIM(TIM5, CH2, PH11, 0, 0, 0),
    DEF_TIM(TIM5, CH3, PH12, 0, 0, 0),

    DEF_TIM(TIM8, CH1, PH6, 0, 0, 0), // AF3 (ADDED)
    DEF_TIM(TIM8, CH1N, PH7, 0, 0, 0),
    DEF_TIM(TIM8, CH2, PH8, 0, 0, 0),
    DEF_TIM(TIM8, CH2N, PH9, 0, 0, 0),
    DEF_TIM(TIM8, CH3, PH10, 0, 0, 0),
    DEF_TIM(TIM8, CH3N, PH11, 0, 0, 0),
    DEF_TIM(TIM8, CH1N, PH13, 0, 0, 0),
    DEF_TIM(TIM8, CH2N, PH14, 0, 0, 0),
    DEF_TIM(TIM8, CH3N, PH15, 0, 0, 0),

// Port I
    DEF_TIM(TIM5, CH4, PI0, 0, 0, 0), // AF2 (ADDED)

    DEF_TIM(TIM8, CH4, PI2, 0, 0, 0), // AF3 (ADDED)
    DEF_TIM(TIM8, CH1, PI5, 0, 0, 0),
    DEF_TIM(TIM8, CH2, PI6, 0, 0, 0),
    DEF_TIM(TIM8, CH3, PI7, 0, 0, 0),
};
#endif


uint32_t timerClock(const TIM_TypeDef *tim)
{
    int timpre;
    uint32_t pclk;
    uint32_t ppre;

    // This function is used to calculate the timer clock frequency.
    // RM0481 (Rev 3) Table 


    RCC_ClkInitTypeDef  clkConfig, uint32_t fLatency
    HAL_RCC_GetClockConfig(&clkConfig, &fLatency);

    if ((uintptr_t)tim >= APB2PERIPH_BASE ) { // APB2 
        pclk = HAL_RCC_GetPCLK2Freq();
        ppre = clkConfig.APB2CLKDivider;
    } else {  // all other timers are on APB1
        pclk = HAL_RCC_GetPCLK1Freq();
        ppre = clkConfig.APB1CLKDivider;
    }
    timpre = (RCC->CFGR & RCC_CFGR_TIMPRE) ? 1 : 0;
#define PC(m) (0x80 | (m))  // multiply pclk
#define HC(m) (0x00 | (m))  // multiply hclk
        static const uint8_t timpreTab[2][8] = { //  see RM0481 TIMPRE: timers clocks prescaler selection
        //  1      1      1      1      2      4      8      16 
        { HC(1), HC(1), HC(1), HC(1), HC(1), PC(2), PC(2), PC(2) }, // TIMPRE = 0 
        { PC(2), PC(2), PC(2), PC(2), PC(2), PC(2), PC(4), PC(4) }     // TIMPRE = 1 
      }
#undef PC
#undef HC
     int flagMult = timpreTab[timpre][ppre];

     if (flagMult & 0x80) { // PCLK based
        return pclk * (flagMult & 0x7f);
     } else {
         return HAL_RCC_GetHCLKFreq() *  (flagMult & 0x7f);
     }

    return pclk * periphToKernel[timpre][ppre];

}
#endif
