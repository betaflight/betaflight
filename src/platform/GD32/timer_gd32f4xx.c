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

#include "gd32f4xx.h"
#include "platform/rcc.h"
#include "drivers/timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = (void *)TIMER0,  .rcc = RCC_APB2(TIMER0),  .inputIrq = TIMER0_Channel_IRQn},
    { .TIMx = (void *)TIMER1,  .rcc = RCC_APB1(TIMER1),  .inputIrq = TIMER1_IRQn},
    { .TIMx = (void *)TIMER2,  .rcc = RCC_APB1(TIMER2),  .inputIrq = TIMER2_IRQn},
    { .TIMx = (void *)TIMER3,  .rcc = RCC_APB1(TIMER3),  .inputIrq = TIMER3_IRQn},

#if defined (GD32F450) || defined (GD32F460) || defined (GD32F470) || defined (GD32F405) || defined (GD32F425) || defined (GD32F407) || defined (GD32F427)
    { .TIMx = (void *)TIMER4,  .rcc = RCC_APB1(TIMER4),  .inputIrq = TIMER4_IRQn},
    { .TIMx = (void *)TIMER5,  .rcc = RCC_APB1(TIMER5),  .inputIrq = TIMER5_DAC_IRQn},
    { .TIMx = (void *)TIMER6,  .rcc = RCC_APB1(TIMER6),  .inputIrq = TIMER6_IRQn},
#else
    { .TIMx = (void *)TIMER4,  .rcc = RCC_APB1(TIMER4),  .inputIrq = 0}, 
    { .TIMx = (void *)TIMER5,  .rcc = RCC_APB1(TIMER5),  .inputIrq = 0},
    { .TIMx = (void *)TIMER6,  .rcc = RCC_APB1(TIMER6),  .inputIrq = 0},
#endif /* GD32F450 || GD32F460 || GD32F470 || (GD32F405) || defined (GD32F425)  defined (GD32F407) || defined (GD32F427)  */

    { .TIMx = (void *)TIMER7,  .rcc = RCC_APB2(TIMER7),  .inputIrq = TIMER7_Channel_IRQn},    
    { .TIMx = (void *)TIMER8,  .rcc = RCC_APB2(TIMER8),  .inputIrq = TIMER0_BRK_TIMER8_IRQn},
    { .TIMx = (void *)TIMER9,  .rcc = RCC_APB2(TIMER9),  .inputIrq = TIMER0_UP_TIMER9_IRQn},
    { .TIMx = (void *)TIMER10, .rcc = RCC_APB2(TIMER10), .inputIrq = TIMER0_TRG_CMT_TIMER10_IRQn},

    { .TIMx = (void *)TIMER11, .rcc = RCC_APB1(TIMER11), .inputIrq = TIMER7_BRK_TIMER11_IRQn},
    { .TIMx = (void *)TIMER12, .rcc = RCC_APB1(TIMER12), .inputIrq = TIMER7_UP_TIMER12_IRQn},
    { .TIMx = (void *)TIMER13, .rcc = RCC_APB1(TIMER13), .inputIrq = TIMER7_TRG_CMT_TIMER13_IRQn},
};

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    // Auto-generated from 'timer_def.h'
//PORTA
    DEF_TIMER(TIMER1, CH0,  PA0, 0, 0),
    DEF_TIMER(TIMER1, CH1,  PA1, 0, 0),
    DEF_TIMER(TIMER1, CH2,  PA2, 0, 0),
    DEF_TIMER(TIMER1, CH3,  PA3, 0, 0),
    DEF_TIMER(TIMER1, CH0,  PA5, 0, 0),
    DEF_TIMER(TIMER0, CH0N, PA7, 0, 0),
    DEF_TIMER(TIMER0, CH0,  PA8, 0, 0),
    DEF_TIMER(TIMER0, CH1,  PA9, 0, 0),
    DEF_TIMER(TIMER0, CH2,  PA10, 0, 0),
    DEF_TIMER(TIMER0, CH3,  PA11, 0, 0),
    DEF_TIMER(TIMER1, CH0,  PA15, 0, 0),

    DEF_TIMER(TIMER4, CH0,  PA0, 0, 0),
    DEF_TIMER(TIMER4, CH1,  PA1, 0, 0),
    DEF_TIMER(TIMER4, CH2,  PA2, 0, 0),
    DEF_TIMER(TIMER4, CH3,  PA3, 0, 0),
    DEF_TIMER(TIMER2, CH0,  PA6, 0, 0),
    DEF_TIMER(TIMER2, CH1,  PA7, 0, 0),

    DEF_TIMER(TIMER8, CH0,  PA2, 0, 0),
    DEF_TIMER(TIMER8, CH1,  PA3, 0, 0),
    DEF_TIMER(TIMER7, CH0N, PA5, 0, 0),
    DEF_TIMER(TIMER7, CH0N, PA7, 0, 0),

    DEF_TIMER(TIMER12, CH0, PA6, 0, 0),
    DEF_TIMER(TIMER13, CH0, PA7, 0, 0),

//PORTB
    DEF_TIMER(TIMER0, CH1N, PB0, 0, 0),
    DEF_TIMER(TIMER0, CH2N, PB1, 0, 0),
    DEF_TIMER(TIMER1, CH3,  PB2, 0, 0),
    DEF_TIMER(TIMER1, CH1,  PB3, 0, 0),  
    DEF_TIMER(TIMER1, CH2,  PB10, 0, 0),
    DEF_TIMER(TIMER1, CH3,  PB11, 0, 0),
    DEF_TIMER(TIMER0, CH0N, PB13, 0, 0),
    DEF_TIMER(TIMER0, CH1N, PB14, 0, 0),
    DEF_TIMER(TIMER0, CH2N, PB15, 0, 0),

    DEF_TIMER(TIMER2, CH2,  PB0, 0, 0),
    DEF_TIMER(TIMER2, CH3,  PB1, 0, 0),
    DEF_TIMER(TIMER2, CH0,  PB4, 0, 0),
    DEF_TIMER(TIMER2, CH1,  PB5, 0, 0),
    DEF_TIMER(TIMER3, CH0,  PB6, 0, 0),
    DEF_TIMER(TIMER3, CH1,  PB7, 0, 0),
    DEF_TIMER(TIMER3, CH2,  PB8, 0, 0),
    DEF_TIMER(TIMER3, CH3,  PB9, 0, 0),

    DEF_TIMER(TIMER7,  CH1N, PB0, 0, 0),
    DEF_TIMER(TIMER7,  CH2N, PB1, 0, 0),
    DEF_TIMER(TIMER9,  CH0,  PB8, 0, 0),
    DEF_TIMER(TIMER10, CH0,  PB9, 0, 0),
    DEF_TIMER(TIMER7,  CH1N, PB14, 0, 0),
    DEF_TIMER(TIMER7,  CH2N, PB15, 0, 0),

    DEF_TIMER(TIMER11, CH0, PB14, 0, 0),
    DEF_TIMER(TIMER11, CH1, PB15, 0, 0),

//PORTC
    DEF_TIMER(TIMER2, CH0, PC6, 0, 0),
    DEF_TIMER(TIMER2, CH1, PC7, 0, 0),
    DEF_TIMER(TIMER2, CH2, PC8, 0, 0),
    DEF_TIMER(TIMER2, CH3, PC9, 0, 0),

    DEF_TIMER(TIMER7, CH0, PC6, 0, 0),
    DEF_TIMER(TIMER7, CH1, PC7, 0, 0),
    DEF_TIMER(TIMER7, CH2, PC8, 0, 0),
    DEF_TIMER(TIMER7, CH3, PC9, 0, 0),

//PORTD
    DEF_TIMER(TIMER3, CH0, PD12, 0, 0),
    DEF_TIMER(TIMER3, CH1, PD13, 0, 0),
    DEF_TIMER(TIMER3, CH2, PD14, 0, 0),
    DEF_TIMER(TIMER3, CH3, PD15, 0, 0),

//PORTE
    DEF_TIMER(TIMER0, CH0N, PE8, 0, 0),
    DEF_TIMER(TIMER0, CH0,  PE9, 0, 0),
    DEF_TIMER(TIMER0, CH1N, PE10, 0, 0),
    DEF_TIMER(TIMER0, CH1,  PE11, 0, 0),
    DEF_TIMER(TIMER0, CH2N, PE12, 0, 0),
    DEF_TIMER(TIMER0, CH2,  PE13, 0, 0),
    DEF_TIMER(TIMER0, CH3,  PE14, 0, 0),

    DEF_TIMER(TIMER8, CH0,  PE5, 0, 0),
    DEF_TIMER(TIMER8, CH1,  PE6, 0, 0),

//PORTF
    DEF_TIMER(TIMER9,  CH0, PF6, 0, 0),
    DEF_TIMER(TIMER10, CH0, PF7, 0, 0),
};
#endif

uint32_t timerClock(const TIM_TypeDef *tim)
{
    uint32_t timer = (uint32_t)tim;
    if (timer == TIMER7 || timer == TIMER0 || timer == TIMER8 || timer == TIMER9 || timer == TIMER10) {
        return SystemCoreClock;
    } else {
        return SystemCoreClock / 2;
    }
}

uint32_t timerPrescaler(const TIM_TypeDef *tim)
{
    uint32_t timer = (uint32_t)tim;
    return TIMER_PSC(timer) + 1;
}

void gd32_timer_input_capture_config(void* timer, uint16_t channel, uint8_t state)
{
    if (timer == NULL) {
        return;
    }

    switch(channel) {
    case TIMER_CH_0:
        if(state) {
            TIMER_CHCTL2((uint32_t)timer) |= (uint32_t)TIMER_CHCTL2_CH0EN;
        } else {
            TIMER_CHCTL2((uint32_t)timer) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
        }
        break;
    case TIMER_CH_1:
        if(state) {
            TIMER_CHCTL2((uint32_t)timer) |= (uint32_t)TIMER_CHCTL2_CH1EN;
        } else {
            TIMER_CHCTL2((uint32_t)timer) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
        }
        break;
    case TIMER_CH_2:
        if(state) {
            TIMER_CHCTL2((uint32_t)timer) |= (uint32_t)TIMER_CHCTL2_CH2EN;
        } else {
            TIMER_CHCTL2((uint32_t)timer) &= (~(uint32_t)TIMER_CHCTL2_CH2EN);
        }
        break;
    case TIMER_CH_3:
        if(state) {
            TIMER_CHCTL2((uint32_t)timer) |= (uint32_t)TIMER_CHCTL2_CH3EN;
        } else {
            TIMER_CHCTL2((uint32_t)timer) &= (~(uint32_t)TIMER_CHCTL2_CH3EN);
        }
        break;
    default:
        break;
    }
}

#endif
