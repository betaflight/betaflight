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

#ifdef USE_TIMER

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/io.h"
#include "timer_def.h"

#include "stm32g4xx.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"


const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),  .inputIrq = TIM1_CC_IRQn},
    { .TIMx = TIM2,  .rcc = RCC_APB11(TIM2), .inputIrq = TIM2_IRQn},
    { .TIMx = TIM3,  .rcc = RCC_APB11(TIM3), .inputIrq = TIM3_IRQn},
    { .TIMx = TIM4,  .rcc = RCC_APB11(TIM4), .inputIrq = TIM4_IRQn},
    { .TIMx = TIM5,  .rcc = RCC_APB11(TIM5), .inputIrq = TIM5_IRQn},
    { .TIMx = TIM6,  .rcc = RCC_APB11(TIM6), .inputIrq = TIM6_DAC_IRQn},
    { .TIMx = TIM7,  .rcc = RCC_APB11(TIM7), .inputIrq = TIM7_DAC_IRQn},
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),  .inputIrq = TIM8_CC_IRQn},
    { .TIMx = TIM15, .rcc = RCC_APB2(TIM15), .inputIrq = TIM1_BRK_TIM15_IRQn},
    { .TIMx = TIM16, .rcc = RCC_APB2(TIM16), .inputIrq = TIM1_UP_TIM16_IRQn},
    { .TIMx = TIM17, .rcc = RCC_APB2(TIM17), .inputIrq = TIM1_TRG_COM_TIM17_IRQn},
    { .TIMx = TIM20, .rcc = RCC_APB2(TIM20), .inputIrq = TIM20_CC_IRQn},
};

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    // Auto-generated from 'timer_def.h'
// Port A
    DEF_TIM(TIM2, CH1, PA0, 0, 0, 0),
    DEF_TIM(TIM2, CH2, PA1, 0, 0, 0),
    DEF_TIM(TIM2, CH3, PA2, 0, 0, 0),
    DEF_TIM(TIM2, CH4, PA3, 0, 0, 0),
    DEF_TIM(TIM2, CH1, PA5, 0, 0, 0),
    DEF_TIM(TIM16, CH1, PA6, 0, 0, 0),
    DEF_TIM(TIM17, CH1, PA7, 0, 0, 0),
    DEF_TIM(TIM16, CH1, PA12, 0, 0, 0),
    DEF_TIM(TIM16, CH1N, PA13, 0, 0, 0),
    DEF_TIM(TIM2, CH1, PA15, 0, 0, 0),

    DEF_TIM(TIM5, CH1, PA0, 0, 0, 0),
    DEF_TIM(TIM5, CH2, PA1, 0, 0, 0),
    DEF_TIM(TIM5, CH3, PA2, 0, 0, 0),
    DEF_TIM(TIM5, CH4, PA3, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PA4, 0, 0, 0),
    DEF_TIM(TIM3, CH1, PA6, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PA7, 0, 0, 0),
    DEF_TIM(TIM8, CH1, PA15, 0, 0, 0),

    DEF_TIM(TIM8, CH1N, PA7, 0, 0, 0),

    DEF_TIM(TIM8, CH2, PA14, 0, 0, 0),
    
    DEF_TIM(TIM1, CH1N, PA7, 0, 0, 0),
    DEF_TIM(TIM1, CH1, PA8, 0, 0, 0),
    DEF_TIM(TIM1, CH2, PA9, 0, 0, 0),
    DEF_TIM(TIM1, CH3, PA10, 0, 0, 0),
    DEF_TIM(TIM1, CH1N, PA11, 0, 0, 0),
    DEF_TIM(TIM1, CH2N, PA12, 0, 0, 0),

    DEF_TIM(TIM15, CH1N, PA1, 0, 0, 0),
    DEF_TIM(TIM15, CH1, PA2, 0, 0, 0),
    DEF_TIM(TIM15, CH2, PA3, 0, 0, 0),

    DEF_TIM(TIM2, CH3, PA9, 0, 0, 0),
    DEF_TIM(TIM2, CH4, PA10, 0, 0, 0),
    DEF_TIM(TIM4, CH1, PA11, 0, 0, 0),
    DEF_TIM(TIM4, CH2, PA12, 0, 0, 0),
    DEF_TIM(TIM4, CH3, PA13, 0, 0, 0),

    DEF_TIM(TIM1, CH4, PA11, 0, 0, 0),

// Port B
    DEF_TIM(TIM2, CH2, PB3, 0, 0, 0),
    DEF_TIM(TIM16, CH1, PB4, 0, 0, 0),
    DEF_TIM(TIM16, CH1N, PB6, 0, 0, 0),
    DEF_TIM(TIM17, CH1N, PB7, 0, 0, 0),
    DEF_TIM(TIM16, CH1, PB8, 0, 0, 0),
    DEF_TIM(TIM17, CH1, PB9, 0, 0, 0),
    DEF_TIM(TIM2, CH3, PB10, 0, 0, 0),
    DEF_TIM(TIM2, CH4, PB11, 0, 0, 0),
    DEF_TIM(TIM15, CH1, PB14, 0, 0, 0),
    DEF_TIM(TIM15, CH2, PB15, 0, 0, 0),

    DEF_TIM(TIM3, CH3, PB0, 0, 0, 0),
    DEF_TIM(TIM3, CH4, PB1, 0, 0, 0),
    DEF_TIM(TIM5, CH1, PB2, 0, 0, 0),
    DEF_TIM(TIM3, CH1, PB4, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PB5, 0, 0, 0),
    DEF_TIM(TIM4, CH1, PB6, 0, 0, 0),
    DEF_TIM(TIM4, CH2, PB7, 0, 0, 0),
    DEF_TIM(TIM4, CH3, PB8, 0, 0, 0),
    DEF_TIM(TIM4, CH4, PB9, 0, 0, 0),
    DEF_TIM(TIM15, CH1N, PB15, 0, 0, 0),

    DEF_TIM(TIM20, CH1, PB2, 0, 0, 0),
    DEF_TIM(TIM8, CH3N, PB5, 0, 0, 0),

    DEF_TIM(TIM8, CH2N, PB0, 0, 0, 0),
    DEF_TIM(TIM8, CH3N, PB1, 0, 0, 0),
    DEF_TIM(TIM8, CH1N, PB3, 0, 0, 0),
    DEF_TIM(TIM8, CH2N, PB4, 0, 0, 0),
    DEF_TIM(TIM1, CH3N, PB15, 0, 0, 0),
    
    DEF_TIM(TIM8, CH1, PB6, 0, 0, 0),

    DEF_TIM(TIM1, CH2N, PB0, 0, 0, 0),
    DEF_TIM(TIM1, CH3N, PB1, 0, 0, 0),
    DEF_TIM(TIM1, CH1N, PB13, 0, 0, 0),
    DEF_TIM(TIM1, CH2N, PB14, 0, 0, 0),

    DEF_TIM(TIM17, CH1, PB5, 0, 0, 0),
    DEF_TIM(TIM3, CH4, PB7, 0, 0, 0),
    DEF_TIM(TIM8, CH2, PB8, 0, 0, 0),
    DEF_TIM(TIM8, CH3, PB9, 0, 0, 0),

    DEF_TIM(TIM1, CH3N, PB9, 0, 0, 0),

// Port C
    DEF_TIM(TIM5, CH2, PC12, 0, 0, 0),

    DEF_TIM(TIM1, CH1, PC0, 0, 0, 0),
    DEF_TIM(TIM1, CH2, PC1, 0, 0, 0),
    DEF_TIM(TIM1, CH3, PC2, 0, 0, 0),
    DEF_TIM(TIM1, CH4, PC3, 0, 0, 0),
    DEF_TIM(TIM3, CH1, PC6, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PC7, 0, 0, 0),
    DEF_TIM(TIM3, CH3, PC8, 0, 0, 0),
    DEF_TIM(TIM3, CH4, PC9, 0, 0, 0),

    DEF_TIM(TIM8, CH1, PC6, 0, 0, 0),
    DEF_TIM(TIM8, CH2, PC7, 0, 0, 0),
    DEF_TIM(TIM8, CH3, PC8, 0, 0, 0),
    DEF_TIM(TIM8, CH4, PC9, 0, 0, 0),
    DEF_TIM(TIM8, CH1N, PC10, 0, 0, 0),
    DEF_TIM(TIM8, CH2N, PC11, 0, 0, 0),
    DEF_TIM(TIM8, CH3N, PC12, 0, 0, 0),
    DEF_TIM(TIM1, CH1N, PC13, 0, 0, 0),

    DEF_TIM(TIM20, CH2, PC2, 0, 0, 0),
    DEF_TIM(TIM1, CH4N, PC5, 0, 0, 0),
    DEF_TIM(TIM20, CH3, PC8, 0, 0, 0),
    DEF_TIM(TIM8, CH4N, PC13, 0, 0, 0),
};
#endif

uint32_t timerClock(TIM_TypeDef *tim)
{
    /*
     * RM0440 Rev.1
     * 6.2.13 Timer clock
     * The timer clock frequencies are automatically defined by hardware. There are two cases:
     * 1. If the APB prescaler equals 1, the timer clock frequencies are set to the same frequency as that of the APB domain.
     * 2. Otherwise, they are set to twice (×2) the frequency of the APB domain.
     */

    uint32_t pclk;

    if (tim == TIM1 || tim == TIM8 || tim == TIM15 || tim == TIM16 || tim == TIM17 || tim == TIM20) {
        // Timers on APB2; PCLK2
        pclk = HAL_RCC_GetPCLK2Freq();
        if (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2)) {
            pclk *= 2;
        }
    } else {
        // Timers on APB1; PCLK1
        pclk = HAL_RCC_GetPCLK1Freq();
        if (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1)) {
            pclk *= 2;
        }
    }

    return pclk;
}
#endif
