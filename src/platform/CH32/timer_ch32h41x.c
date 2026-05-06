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

#include "timer_def.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "platform/rcc.h"
#include "drivers/timer.h"
#include "platform/timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_HB2(TIM1),  .inputIrq = TIM1_CC_IRQn },
#ifndef USE_INTERNAL_OSD        
    { .TIMx = TIM2,  .rcc = RCC_HB1(TIM2),  .inputIrq = TIM2_IRQn },
#endif
    { .TIMx = TIM3,  .rcc = RCC_HB1(TIM3),  .inputIrq = TIM3_IRQn },
    { .TIMx = TIM4,  .rcc = RCC_HB1(TIM4),  .inputIrq = TIM4_IRQn },
    { .TIMx = TIM5,  .rcc = RCC_HB1(TIM5),  .inputIrq = TIM5_IRQn },
    { .TIMx = TIM6,  .rcc = RCC_HB1(TIM6),  .inputIrq = TIM6_IRQn },
    { .TIMx = TIM7,  .rcc = RCC_HB1(TIM7),  .inputIrq = TIM7_IRQn} ,
    { .TIMx = TIM8,  .rcc = RCC_HB2(TIM8),  .inputIrq = TIM8_CC_IRQn },
    { .TIMx = TIM9,  .rcc = RCC_HB2(TIM9),  .inputIrq = TIM9_IRQn },
#ifndef USE_INTERNAL_OSD
    { .TIMx = TIM10, .rcc = RCC_HB2(TIM10), .inputIrq = TIM10_IRQn },
    { .TIMx = TIM11, .rcc = RCC_HB2(TIM11), .inputIrq = TIM11_IRQn },
    { .TIMx = TIM12, .rcc = RCC_HB2(TIM12), .inputIrq = TIM12_IRQn },
#endif
};

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {

    DEF_TIM(TIM1,  CH1,  PE9, 0, 0, 0),
    DEF_TIM(TIM1,  CH1,  PA8, 0, 0, 0),
    DEF_TIM(TIM1,  CH2,  PE11, 0, 0, 0),
    DEF_TIM(TIM1,  CH2,  PA9, 0, 0, 0),
    DEF_TIM(TIM1,  CH3,  PE13, 0, 0, 0),
    DEF_TIM(TIM1,  CH3,  PA10, 0, 0, 0),
    DEF_TIM(TIM1,  CH4,  PE14, 0, 0, 0),
    DEF_TIM(TIM1,  CH4,  PA11, 0, 0, 0),

    DEF_TIM(TIM1,  CH1N,  PE8, 0, 0, 0),
    DEF_TIM(TIM1,  CH1N,  PA7, 0, 0, 0),
    DEF_TIM(TIM1,  CH1N,  PB13, 0, 0, 0),
    DEF_TIM(TIM1,  CH2N,  PE10, 0, 0, 0),
    DEF_TIM(TIM1,  CH2N,  PB0, 0, 0, 0),
    DEF_TIM(TIM1,  CH2N,  PB14, 0, 0, 0),
    DEF_TIM(TIM1,  CH3N,  PE12, 0, 0, 0),
    DEF_TIM(TIM1,  CH3N,  PB1, 0, 0, 0),
    DEF_TIM(TIM1,  CH3N,  PB15, 0, 0, 0),

    DEF_TIM(TIM8,  CH1,  PE3, 0, 0, 0),
    DEF_TIM(TIM8,  CH2,  PE4, 0, 0, 0),
    DEF_TIM(TIM8,  CH3,  PE5, 0, 0, 0),
    DEF_TIM(TIM8,  CH4,  PE6, 0, 0, 0),
    DEF_TIM(TIM8,  CH1,  PC6, 0, 0, 0),
    DEF_TIM(TIM8,  CH2,  PC7, 0, 0, 0),
    DEF_TIM(TIM8,  CH3,  PC8, 0, 0, 0),
    DEF_TIM(TIM8,  CH4,  PC9, 0, 0, 0),

    DEF_TIM(TIM8,  CH1N,  PC1, 0, 0, 0),
    DEF_TIM(TIM8,  CH2N,  PC2, 0, 0, 0),
    DEF_TIM(TIM8,  CH3N,  PC3, 0, 0, 0),
    DEF_TIM(TIM8,  CH1N,  PA5, 0, 0, 0),
    DEF_TIM(TIM8,  CH2N,  PB14, 0, 0, 0),
    DEF_TIM(TIM8,  CH3N,  PB15, 0, 0, 0),
    DEF_TIM(TIM8,  CH1N,  PA7, 0, 0, 0),
    DEF_TIM(TIM8,  CH2N,  PB0, 0, 0, 0),
    DEF_TIM(TIM8,  CH3N,  PB1, 0, 0, 0),

    DEF_TIM(TIM2,  CH1,  PA0, 0, 0, 0),
    DEF_TIM(TIM2,  CH1,  PA5, 0, 0, 0),
    DEF_TIM(TIM2,  CH1,  PA15, 0, 0, 0),
    DEF_TIM(TIM2,  CH2,  PA1, 0, 0, 0),
    DEF_TIM(TIM2,  CH2,  PB3, 0, 0, 0),
    DEF_TIM(TIM2,  CH3,  PA2, 0, 0, 0),
    DEF_TIM(TIM2,  CH3,  PB10, 0, 0, 0),
    DEF_TIM(TIM2,  CH4,  PA3, 0, 0, 0),
    DEF_TIM(TIM2,  CH4,  PB11, 0, 0, 0),

    DEF_TIM(TIM3,  CH1,  PA6, 0, 0, 0),
    DEF_TIM(TIM3,  CH1,  PB4, 0, 0, 0),
    DEF_TIM(TIM3,  CH1,  PC6, 0, 0, 0),
    DEF_TIM(TIM3,  CH1,  PD3, 0, 0, 0),
    DEF_TIM(TIM3,  CH2,  PA7, 0, 0, 0),
    DEF_TIM(TIM3,  CH2,  PB5, 0, 0, 0),
    DEF_TIM(TIM3,  CH2,  PC7, 0, 0, 0),
    DEF_TIM(TIM3,  CH2,  PD4, 0, 0, 0),
    DEF_TIM(TIM3,  CH3,  PB0, 0, 0, 0),
    DEF_TIM(TIM3,  CH3,  PC8, 0, 0, 0),
    DEF_TIM(TIM3,  CH3,  PD5, 0, 0, 0),
    DEF_TIM(TIM3,  CH4,  PB1, 0, 0, 0),
    DEF_TIM(TIM3,  CH4,  PC9, 0, 0, 0),
    DEF_TIM(TIM3,  CH4,  PD6, 0, 0, 0),

    DEF_TIM(TIM4,  CH1,  PB6, 0, 0, 0),
    DEF_TIM(TIM4,  CH1,  PD12, 0, 0, 0),
    DEF_TIM(TIM4,  CH1,  PE3, 0, 0, 0),
    DEF_TIM(TIM4,  CH2,  PB7, 0, 0, 0),
    DEF_TIM(TIM4,  CH2,  PD13, 0, 0, 0),
    DEF_TIM(TIM4,  CH2,  PE4, 0, 0, 0),
    DEF_TIM(TIM4,  CH3,  PB8, 0, 0, 0),
    DEF_TIM(TIM4,  CH3,  PD14, 0, 0, 0),
    DEF_TIM(TIM4,  CH3,  PE5, 0, 0, 0),
    DEF_TIM(TIM4,  CH4,  PB9, 0, 0, 0),
    DEF_TIM(TIM4,  CH4,  PD15, 0, 0, 0),
    DEF_TIM(TIM4,  CH4,  PE6, 0, 0, 0),

    DEF_TIM(TIM5,  CH1,  PA0, 0, 0, 0),
    DEF_TIM(TIM5,  CH1,  PD12, 0, 0, 0),
    DEF_TIM(TIM5,  CH1,  PC1, 0, 0, 0),
    DEF_TIM(TIM5,  CH2,  PA1, 0, 0, 0),
    DEF_TIM(TIM5,  CH2,  PD13, 0, 0, 0),
    DEF_TIM(TIM5,  CH2,  PC2, 0, 0, 0),
    DEF_TIM(TIM5,  CH3,  PA2, 0, 0, 0),
    DEF_TIM(TIM5,  CH3,  PD14, 0, 0, 0),
    DEF_TIM(TIM5,  CH3,  PC3, 0, 0, 0),
    DEF_TIM(TIM5,  CH4,  PA3, 0, 0, 0),
    DEF_TIM(TIM5,  CH4,  PD15, 0, 0, 0),
    DEF_TIM(TIM5,  CH4,  PB0, 0, 0, 0),

    DEF_TIM(TIM9,  CH1,  PB14, 0, 0, 0),
    DEF_TIM(TIM9,  CH1,  PA0, 0, 0, 0),
    DEF_TIM(TIM9,  CH1,  PC9, 0, 0, 0),
    DEF_TIM(TIM9,  CH2,  PB15, 0, 0, 0),
    DEF_TIM(TIM9,  CH2,  PA1, 0, 0, 0),
    DEF_TIM(TIM9,  CH2,  PC10, 0, 0, 0),
    DEF_TIM(TIM9,  CH2,  PB10, 0, 0, 0),
    DEF_TIM(TIM9,  CH3,  PE5, 0, 0, 0),
    DEF_TIM(TIM9,  CH3,  PA2, 0, 0, 0),
    DEF_TIM(TIM9,  CH3,  PC12, 0, 0, 0),
    DEF_TIM(TIM9,  CH3,  PB12, 0, 0, 0),
    DEF_TIM(TIM9,  CH4,  PE6, 0, 0, 0),
    DEF_TIM(TIM9,  CH4,  PA3, 0, 0, 0),
    DEF_TIM(TIM9,  CH4,  PC11, 0, 0, 0),
    DEF_TIM(TIM9,  CH4,  PB11, 0, 0, 0),


    DEF_TIM(TIM10,  CH1,  PF8, 0, 0, 0),
    DEF_TIM(TIM10,  CH1,  PA6, 0, 0, 0),
    DEF_TIM(TIM10,  CH1,  PB6, 0, 0, 0),
    DEF_TIM(TIM10,  CH2,  PF9, 0, 0, 0),
    DEF_TIM(TIM10,  CH2,  PA7, 0, 0, 0),
    DEF_TIM(TIM10,  CH2,  PB7, 0, 0, 0),
    DEF_TIM(TIM10,  CH3,  PF6, 0, 0, 0),
    DEF_TIM(TIM10,  CH3,  PA3, 0, 0, 0),
    DEF_TIM(TIM10,  CH3,  PB8, 0, 0, 0),
    DEF_TIM(TIM10,  CH4,  PF7, 0, 0, 0),
    DEF_TIM(TIM10,  CH4,  PA4, 0, 0, 0),
    DEF_TIM(TIM10,  CH4,  PB9, 0, 0, 0),

    DEF_TIM(TIM11,  CH1,  PD3, 0, 0, 0),
    DEF_TIM(TIM11,  CH1,  PF6, 0, 0, 0),
    DEF_TIM(TIM11,  CH1,  PE0, 0, 0, 0),
    DEF_TIM(TIM11,  CH2,  PD4, 0, 0, 0),
    DEF_TIM(TIM11,  CH2,  PF7, 0, 0, 0),
    DEF_TIM(TIM11,  CH2,  PE1, 0, 0, 0),
    DEF_TIM(TIM11,  CH3,  PD5, 0, 0, 0),
    DEF_TIM(TIM11,  CH3,  PF8, 0, 0, 0),
    DEF_TIM(TIM11,  CH3,  PD7, 0, 0, 0),
    DEF_TIM(TIM11,  CH4,  PD6, 0, 0, 0),
    DEF_TIM(TIM11,  CH4,  PF9, 0, 0, 0),

    DEF_TIM(TIM12,  CH1,  PB1, 0, 0, 0),
    DEF_TIM(TIM12,  CH1,  PE3, 0, 0, 0),

    DEF_TIM(TIM12,  CH2,  PB2, 0, 0, 0),
    DEF_TIM(TIM12,  CH2,  PE4, 0, 0, 0),
    DEF_TIM(TIM12,  CH2,  PE13, 0, 0, 0),

    DEF_TIM(TIM12,  CH3,  PF12, 0, 0, 0),
    DEF_TIM(TIM12,  CH3,  PE5, 0, 0, 0),
    DEF_TIM(TIM12,  CH3,  PE14, 0, 0, 0),

    DEF_TIM(TIM12,  CH4,  PF13, 0, 0, 0),
    DEF_TIM(TIM12,  CH4,  PE6, 0, 0, 0),
    DEF_TIM(TIM12,  CH4,  PE15, 0, 0, 0),

  };
#endif

uint32_t timerClock(const timerHardware_t *timHw)
{
    UNUSED(timHw);
    return HCLKClock;
}
#endif
