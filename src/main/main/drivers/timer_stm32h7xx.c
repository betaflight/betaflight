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

#include "stm32h7xx.h"
#include "rcc.h"
#include "timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),   .inputIrq = TIM1_CC_IRQn},
    { .TIMx = TIM2,  .rcc = RCC_APB1L(TIM2),  .inputIrq = TIM2_IRQn},
    { .TIMx = TIM3,  .rcc = RCC_APB1L(TIM3),  .inputIrq = TIM3_IRQn},
    { .TIMx = TIM4,  .rcc = RCC_APB1L(TIM4),  .inputIrq = TIM4_IRQn},
    { .TIMx = TIM5,  .rcc = RCC_APB1L(TIM5),  .inputIrq = TIM5_IRQn},
    { .TIMx = TIM6,  .rcc = RCC_APB1L(TIM6),  .inputIrq = 0},
    { .TIMx = TIM7,  .rcc = RCC_APB1L(TIM7),  .inputIrq = 0},
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),   .inputIrq = TIM8_CC_IRQn},
    { .TIMx = TIM12, .rcc = RCC_APB1L(TIM12), .inputIrq = TIM8_BRK_TIM12_IRQn},
    { .TIMx = TIM13, .rcc = RCC_APB1L(TIM13), .inputIrq = TIM8_UP_TIM13_IRQn},
    { .TIMx = TIM14, .rcc = RCC_APB1L(TIM14), .inputIrq = TIM8_TRG_COM_TIM14_IRQn},
    { .TIMx = TIM15, .rcc = RCC_APB2(TIM15),  .inputIrq = TIM15_IRQn},
    { .TIMx = TIM16, .rcc = RCC_APB2(TIM16),  .inputIrq = TIM16_IRQn},
    { .TIMx = TIM17, .rcc = RCC_APB2(TIM17),  .inputIrq = TIM17_IRQn},
};

uint32_t timerClock(TIM_TypeDef *tim)
{
    int timpre;
    uint32_t pclk;
    uint32_t ppre;

    // Implement the table:
    // RM0433 "Table 48. Ratio between clock timer and pclk"

    if (tim == TIM1 || tim == TIM8 || tim == TIM15 || tim == TIM16 || tim == TIM17) {
        // Timers on APB2
        pclk = HAL_RCC_GetPCLK2Freq();
        ppre = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos;
    } else {
        // Timers on APB1
        pclk = HAL_RCC_GetPCLK1Freq();
        ppre = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
    }

    timpre = (RCC->CFGR & RCC_CFGR_TIMPRE) ? 1 : 0;

    int index = (timpre << 3) | ppre;

    static uint8_t periphToKernel[16] = { // The mutiplier table
        1, 1, 1, 1, 2, 2, 2, 2, // TIMPRE = 0
        1, 1, 1, 1, 2, 4, 4, 4  // TIMPRE = 1
    };

    return pclk * periphToKernel[index];
}
#endif
