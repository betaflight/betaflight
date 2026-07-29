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

#include "um324xx.h"
#include "platform/rcc.h"
#include "drivers/timer.h"
#include "platform/timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),  .inputIrq = TIM1_CC_IRQn},
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2),  .inputIrq = TIM2_IRQn},
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3),  .inputIrq = TIM3_IRQn},
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4),  .inputIrq = TIM4_IRQn},
    { .TIMx = TIM5,  .rcc = RCC_APB1(TIM5),  .inputIrq = TIM5_IRQn},
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6),  .inputIrq = 0},
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7),  .inputIrq = TIM7_IRQn},
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),  .inputIrq = TIM8_CC_IRQn},
    { .TIMx = TIM9,  .rcc = RCC_APB2(TIM9),  .inputIrq = TIM1_BRK_TIM9_IRQn},
    { .TIMx = TIM10, .rcc = RCC_APB2(TIM10), .inputIrq = TIM1_UP_TIM10_IRQn},
    { .TIMx = TIM11, .rcc = RCC_APB2(TIM11), .inputIrq = TIM1_TRG_COM_TIM11_IRQn},
    { .TIMx = TIM12, .rcc = RCC_APB1(TIM12), .inputIrq = TIM8_BRK_TIM12_IRQn},
    { .TIMx = TIM13, .rcc = RCC_APB1(TIM13), .inputIrq = TIM8_UP_TIM13_IRQn},
    { .TIMx = TIM14, .rcc = RCC_APB1(TIM14), .inputIrq = TIM8_TRG_COM_TIM14_IRQn},
};

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    // Auto-generated from 'timer_def.h'
	  DEF_TIM(TIM12, CH2, PB15, 0, 0),  //MOTOR PB15
	  DEF_TIM(TIM13, CH2, PD8, 0, 0),   //MOTOR PD8, shared PC9
	  DEF_TIM(TIM14, CH2, PE7, 0, 0),   //MOTOR PE7, shared PC8
	  DEF_TIM(TIM1, CH1, PA8, 0, 0),    //MOTOR PA8
	  DEF_TIM(TIM9, CH1, PE5, 0, 0),    //MOTOR PE5, shared PA9
        
      DEF_TIM(TIM2, CH2, PB3,  0, 0),
      DEF_TIM(TIM2, CH3, PB10, 0, 0),   //QFN76 MOTOR PB10

      DEF_TIM(TIM3, CH2, PB5, 0, 0),
      DEF_TIM(TIM3, CH3, PB0, 0, 0),
      DEF_TIM(TIM3, CH4, PB1, 0, 0),    //PB1

      DEF_TIM(TIM4, CH1, PB6, 0, 0),
      DEF_TIM(TIM4, CH2, PD13, 0, 0),

      DEF_TIM(TIM8, CH1, PC6, 0, 0),
      DEF_TIM(TIM8, CH2, PC7, 0, 0),

      DEF_TIM(TIM10, CH1, PC4, 0, 0),
      DEF_TIM(TIM10, CH2, PC5, 0, 0),
      DEF_TIM(TIM10, CH3, PB2, 0, 0),

      DEF_TIM(TIM11, CH1, PB9, 0, 0),
      DEF_TIM(TIM12, CH3, PB12, 0, 0),

      DEF_TIM(TIM13, CH1, PD7, 0, 0),
      DEF_TIM(TIM13, CH4, PD10, 0, 0),
};
#endif

uint32_t timerClockFromInstance(const timerResource_t *tim)
{
#if defined(UM324xF)
    uint32_t pclk;
    const TIM_TypeDef *tim_ptr = (const TIM_TypeDef *)tim;

    if (tim_ptr == TIM1 || tim_ptr == TIM8 || tim_ptr == TIM9 || tim_ptr == TIM10 || tim_ptr == TIM11) {
        // Timers on APB2; PCLK2
        pclk = HAL_RCM_GetPCLK2Freq();
    } else {
        // Timers on APB1; PCLK1
        pclk = HAL_RCM_GetPCLK1Freq();
    }

    return pclk;
#else
    #error "No timer clock defined correctly for MCU"
#endif
}

uint32_t timerClock(const timerHardware_t *timHw)
{
    return timerClockFromInstance(timHw->tim);
}
#endif
