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
#include "platform/dma.h"
#include "drivers/io.h"
#include "timer_def.h"

#include "gd32h7xx.h"
#include "platform/rcc.h"
#include "drivers/timer.h"
#include "platform/timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = (void *)TIMER0,  .rcc = RCC_APB2(TIMER0),  .inputIrq = TIMER0_Channel_IRQn},
    { .TIMx = (void *)TIMER1,  .rcc = RCC_APB1(TIMER1),  .inputIrq = TIMER1_IRQn},
    { .TIMx = (void *)TIMER2,  .rcc = RCC_APB1(TIMER2),  .inputIrq = TIMER2_IRQn},
    { .TIMx = (void *)TIMER3,  .rcc = RCC_APB1(TIMER3),  .inputIrq = TIMER3_IRQn},

    { .TIMx = (void *)TIMER4,  .rcc = RCC_APB1(TIMER4),  .inputIrq = TIMER4_IRQn},
    { .TIMx = (void *)TIMER5,  .rcc = RCC_APB1(TIMER5),  .inputIrq = TIMER5_DAC_UDR_IRQn},
    { .TIMx = (void *)TIMER6,  .rcc = RCC_APB1(TIMER6),  .inputIrq = TIMER6_IRQn},

    { .TIMx = (void *)TIMER7,  .rcc = RCC_APB2(TIMER7),  .inputIrq = TIMER7_Channel_IRQn},
};

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    // Auto-generated from 'timer_def.h'
//PORTA
    DEF_TIMER(TIMER1, CH0,  PA0, 0, 0, 0),
    DEF_TIMER(TIMER1, CH1,  PA1, 0, 0, 0),
    DEF_TIMER(TIMER1, CH2,  PA2, 0, 0, 0),
    DEF_TIMER(TIMER1, CH3,  PA3, 0, 0, 0),
    DEF_TIMER(TIMER1, CH0,  PA5, 0, 0, 0),
    DEF_TIMER(TIMER0, CH0N, PA7, 0, 0, 0),
    DEF_TIMER(TIMER0, CH0,  PA8, 0, 0, 0),
    DEF_TIMER(TIMER0, CH1,  PA9, 0, 0, 0),
    DEF_TIMER(TIMER0, CH2,  PA10, 0, 0, 0),

    DEF_TIMER(TIMER1, CH0,  PA15, 0, 0, 0),

    DEF_TIMER(TIMER4, CH0,  PA0, 0, 0, 0),
    DEF_TIMER(TIMER4, CH1,  PA1, 0, 0, 0),
    DEF_TIMER(TIMER4, CH2,  PA2, 0, 0, 0),
    DEF_TIMER(TIMER4, CH3,  PA3, 0, 0, 0),
    DEF_TIMER(TIMER2, CH0,  PA6, 0, 0, 0),
    DEF_TIMER(TIMER2, CH1,  PA7, 0, 0, 0),

    DEF_TIMER(TIMER7, CH0N, PA5, 0, 0, 0),
    DEF_TIMER(TIMER7, CH0N, PA7, 0, 0, 0),

//PORTB
    DEF_TIMER(TIMER0, CH1N, PB0, 0, 0, 0),
    DEF_TIMER(TIMER0, CH2N, PB1, 0, 0, 0),
    DEF_TIMER(TIMER1, CH1,  PB3, 0, 0, 0),
    DEF_TIMER(TIMER1, CH2,  PB10, 0, 0, 0),
    DEF_TIMER(TIMER1, CH3,  PB11, 0, 0, 0),
    DEF_TIMER(TIMER0, CH0N, PB13, 0, 0, 0),
    DEF_TIMER(TIMER0, CH1N, PB14, 0, 0, 0),
    DEF_TIMER(TIMER0, CH2N, PB15, 0, 0, 0),

    DEF_TIMER(TIMER2, CH2,  PB0, 0, 0, 0),
    DEF_TIMER(TIMER2, CH3,  PB1, 0, 0, 0),
    DEF_TIMER(TIMER2, CH0,  PB4, 0, 0, 0),
    DEF_TIMER(TIMER2, CH1,  PB5, 0, 0, 0),
    DEF_TIMER(TIMER3, CH0,  PB6, 0, 0, 0),
    DEF_TIMER(TIMER3, CH1,  PB7, 0, 0, 0),
    DEF_TIMER(TIMER3, CH2,  PB8, 0, 0, 0),
    DEF_TIMER(TIMER3, CH3,  PB9, 0, 0, 0),

    DEF_TIMER(TIMER7,  CH1N, PB0, 0, 0, 0),
    DEF_TIMER(TIMER7,  CH2N, PB1, 0, 0, 0),
    DEF_TIMER(TIMER7,  CH1N, PB14, 0, 0, 0),
    DEF_TIMER(TIMER7,  CH2N, PB15, 0, 0, 0),

//PORTC
    DEF_TIMER(TIMER0, CH3, PC7, 0, 0, 0),
    DEF_TIMER(TIMER0, CH3, PC10, 0, 0, 0),

    DEF_TIMER(TIMER2, CH0, PC6, 0, 0, 0),
    DEF_TIMER(TIMER2, CH1, PC7, 0, 0, 0),
    DEF_TIMER(TIMER2, CH2, PC8, 0, 0, 0),
    DEF_TIMER(TIMER2, CH3, PC9, 0, 0, 0),

    DEF_TIMER(TIMER7, CH0, PC6, 0, 0, 0),
    DEF_TIMER(TIMER7, CH1, PC7, 0, 0, 0),
    DEF_TIMER(TIMER7, CH2, PC8, 0, 0, 0),
    DEF_TIMER(TIMER7, CH3, PC9, 0, 0, 0),

//PORTD
    DEF_TIMER(TIMER7, CH2, PD0, 0, 0, 0),
    DEF_TIMER(TIMER7, CH3, PD5, 0, 0, 0),
    DEF_TIMER(TIMER3, CH0, PD12, 0, 0, 0),
    DEF_TIMER(TIMER3, CH1, PD13, 0, 0, 0),
    DEF_TIMER(TIMER3, CH2, PD14, 0, 0, 0),
    DEF_TIMER(TIMER3, CH3, PD15, 0, 0, 0),

//PORTE
    DEF_TIMER(TIMER0, CH0N, PE8, 0, 0, 0),
    DEF_TIMER(TIMER0, CH0,  PE9, 0, 0, 0),
    DEF_TIMER(TIMER0, CH1N, PE10, 0, 0, 0),
    DEF_TIMER(TIMER0, CH1,  PE11, 0, 0, 0),
    DEF_TIMER(TIMER0, CH2N, PE12, 0, 0, 0),
    DEF_TIMER(TIMER0, CH2,  PE13, 0, 0, 0),
    DEF_TIMER(TIMER0, CH3,  PE14, 0, 0, 0),

//PORTF

};
#endif

uint32_t timerClockFromInstance(const timerResource_t *tim)
{
    uint32_t timer = (uint32_t)tim;

    if (timer == TIMER0 || timer == TIMER7 || timer == TIMER14 || timer == TIMER15 || timer == TIMER16) {
        return SystemCoreClock / 2; // APB2-300MHz
    } else {
        return SystemCoreClock / 4; // APB1-150MHz
    }
}

uint32_t timerClock(const timerHardware_t *timHw)
{
    return timerClockFromInstance(timHw->tim);
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
