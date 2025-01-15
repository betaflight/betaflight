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

#include "apm32f4xx.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TMR1,  .rcc = RCC_APB2(TMR1),  .inputIrq = TMR1_CC_IRQn},
    { .TIMx = TMR2,  .rcc = RCC_APB1(TMR2),  .inputIrq = TMR2_IRQn},
    { .TIMx = TMR3,  .rcc = RCC_APB1(TMR3),  .inputIrq = TMR3_IRQn},
    { .TIMx = TMR4,  .rcc = RCC_APB1(TMR4),  .inputIrq = TMR4_IRQn},
    { .TIMx = TMR5,  .rcc = RCC_APB1(TMR5),  .inputIrq = TMR5_IRQn},
    { .TIMx = TMR6,  .rcc = RCC_APB1(TMR6),  .inputIrq = 0},
    { .TIMx = TMR7,  .rcc = RCC_APB1(TMR7),  .inputIrq = 0},
    { .TIMx = TMR8,  .rcc = RCC_APB2(TMR8),  .inputIrq = TMR8_CC_IRQn},
    { .TIMx = TMR9,  .rcc = RCC_APB2(TMR9),  .inputIrq = TMR1_BRK_TMR9_IRQn},
    { .TIMx = TMR10, .rcc = RCC_APB2(TMR10), .inputIrq = TMR1_UP_TMR10_IRQn},
    { .TIMx = TMR11, .rcc = RCC_APB2(TMR11), .inputIrq = TMR1_TRG_COM_TMR11_IRQn},
    { .TIMx = TMR12, .rcc = RCC_APB1(TMR12), .inputIrq = TMR8_BRK_TMR12_IRQn},
    { .TIMx = TMR13, .rcc = RCC_APB1(TMR13), .inputIrq = TMR8_UP_TMR13_IRQn},
    { .TIMx = TMR14, .rcc = RCC_APB1(TMR14), .inputIrq = TMR8_TRG_COM_TMR14_IRQn},
};

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
    // Auto-generated from 'timer_def.h'
//PORTA
    DEF_TIM(TMR2, CH1,  PA0, 0, 0),
    DEF_TIM(TMR2, CH2,  PA1, 0, 0),
    DEF_TIM(TMR2, CH3,  PA2, 0, 0),
    DEF_TIM(TMR2, CH4,  PA3, 0, 0),
    DEF_TIM(TMR2, CH1,  PA5, 0, 0),
    DEF_TIM(TMR1, CH1N, PA7, 0, 0),
    DEF_TIM(TMR1, CH1,  PA8, 0, 0),
    DEF_TIM(TMR1, CH2,  PA9, 0, 0),
    DEF_TIM(TMR1, CH3,  PA10, 0, 0),
    DEF_TIM(TMR1, CH1N, PA11, 0, 0),
    DEF_TIM(TMR2, CH1,  PA15, 0, 0),

    DEF_TIM(TMR5, CH1,  PA0, 0, 0),
    DEF_TIM(TMR5, CH2,  PA1, 0, 0),
    DEF_TIM(TMR5, CH3,  PA2, 0, 0),
    DEF_TIM(TMR5, CH4,  PA3, 0, 0),
    DEF_TIM(TMR3, CH1,  PA6, 0, 0),
    DEF_TIM(TMR3, CH2,  PA7, 0, 0),

    DEF_TIM(TMR9, CH1,  PA2, 0, 0),
    DEF_TIM(TMR9, CH2,  PA3, 0, 0),

    DEF_TIM(TMR8, CH1N, PA5, 0, 0),
    DEF_TIM(TMR8, CH1N, PA7, 0, 0),

    DEF_TIM(TMR13, CH1, PA6, 0, 0),
    DEF_TIM(TMR14, CH1, PA7, 0, 0),

//PORTB
    DEF_TIM(TMR1, CH2N, PB0, 0, 0),
    DEF_TIM(TMR1, CH3N, PB1, 0, 0),
    DEF_TIM(TMR2, CH2,  PB3, 0, 0),
    DEF_TIM(TMR2, CH3,  PB10, 0, 0),
    DEF_TIM(TMR2, CH4,  PB11, 0, 0),
    DEF_TIM(TMR1, CH1N, PB13, 0, 0),
    DEF_TIM(TMR1, CH2N, PB14, 0, 0),
    DEF_TIM(TMR1, CH3N, PB15, 0, 0),

    DEF_TIM(TMR3, CH3,  PB0, 0, 0),
    DEF_TIM(TMR3, CH4,  PB1, 0, 0),
    DEF_TIM(TMR3, CH1,  PB4, 0, 0),
    DEF_TIM(TMR3, CH2,  PB5, 0, 0),
    DEF_TIM(TMR4, CH1,  PB6, 0, 0),
    DEF_TIM(TMR4, CH2,  PB7, 0, 0),
    DEF_TIM(TMR4, CH3,  PB8, 0, 0),
    DEF_TIM(TMR4, CH4,  PB9, 0, 0),

    DEF_TIM(TMR8, CH2N, PB0, 0, 0),
    DEF_TIM(TMR8, CH3N, PB1, 0, 0),

    DEF_TIM(TMR10, CH1, PB8, 0, 0),
    DEF_TIM(TMR11, CH1, PB9, 0, 0),

    DEF_TIM(TMR8, CH2N, PB14, 0, 0),
    DEF_TIM(TMR8, CH3N, PB15, 0, 0),

    DEF_TIM(TMR12, CH1, PB14, 0, 0),
    DEF_TIM(TMR12, CH2, PB15, 0, 0),

//PORTC
    DEF_TIM(TMR3, CH1, PC6, 0, 0),
    DEF_TIM(TMR3, CH2, PC7, 0, 0),
    DEF_TIM(TMR3, CH3, PC8, 0, 0),
    DEF_TIM(TMR3, CH4, PC9, 0, 0),

    DEF_TIM(TMR8, CH1, PC6, 0, 0),
    DEF_TIM(TMR8, CH2, PC7, 0, 0),
    DEF_TIM(TMR8, CH3, PC8, 0, 0),
    DEF_TIM(TMR8, CH4, PC9, 0, 0),

//PORTD
    DEF_TIM(TMR4, CH1, PD12, 0, 0),
    DEF_TIM(TMR4, CH2, PD13, 0, 0),
    DEF_TIM(TMR4, CH3, PD14, 0, 0),
    DEF_TIM(TMR4, CH4, PD15, 0, 0),

//PORTE
    DEF_TIM(TMR1, CH1N, PE8, 0, 0),
    DEF_TIM(TMR1, CH1,  PE9, 0, 0),
    DEF_TIM(TMR1, CH2N, PE10, 0, 0),
    DEF_TIM(TMR1, CH2,  PE11, 0, 0),
    DEF_TIM(TMR1, CH3N, PE12, 0, 0),
    DEF_TIM(TMR1, CH3,  PE13, 0, 0),
    DEF_TIM(TMR1, CH4,  PE14, 0, 0),

    DEF_TIM(TMR9, CH1,  PE5, 0, 0),
    DEF_TIM(TMR9, CH2,  PE6, 0, 0),

//PORTF
    DEF_TIM(TMR10, CH1, PF6, 0, 0),
    DEF_TIM(TMR11, CH1, PF7, 0, 0),

//PORTH
// Port H is not available for LPQFP-100 or 144 package
//    DEF_TIM(TMR5, CH1, PH10, 0, 0),
//    DEF_TIM(TMR5, CH2, PH11, 0, 0),
//    DEF_TIM(TMR5, CH3, PH12, 0, 0),
//
//#if !defined(STM32F411xE)
//    DEF_TIM(TMR8, CH1N, PH13, 0, 0),
//    DEF_TIM(TMR8, CH2N, PH14, 0, 0),
//    DEF_TIM(TMR8, CH3N, PH15, 0, 0),
//
//    DEF_TIM(TMR12, CH1, PH6, 0, 0),
//    DEF_TIM(TMR12, CH2, PH9, 0, 0),
//#endif

//PORTI
// Port I is not available for LPQFP-100 or 144 package
//    DEF_TIM(TMR5, CH4, PI0, 0, 0),
//
//#if !defined(STM32F411xE)
//    DEF_TIM(TMR8, CH4, PI2, 0, 0),
//    DEF_TIM(TMR8, CH1, PI5, 0, 0),
//    DEF_TIM(TMR8, CH2, PI6, 0, 0),
//    DEF_TIM(TMR8, CH3, PI7, 0, 0),
//#endif
};
#endif

/*
    need a mapping from dma and timers to pins, and the values should all be set here to the dmaMotors array.
    this mapping could be used for both these motors and for led strip.

    only certain pins have OC output (already used in normal PWM et al) but then
    there are only certain DMA streams/channels available for certain timers and channels.
     *** (this may highlight some hardware limitations on some targets) ***

    DMA1

    Channel Stream0     Stream1     Stream2     Stream3     Stream4     Stream5     Stream6     Stream7
    0
    1
    2       TMR4_CH1                            TMR4_CH2                                        TMR4_CH3
    3                   TMR2_CH3                                        TMR2_CH1    TMR2_CH1    TMR2_CH4
                                                                                    TMR2_CH4
    4
    5                               TMR3_CH4                TMR3_CH1    TMR3_CH2                TMR3_CH3
    6       TMR5_CH3    TMR5_CH4    TMR5_CH1    TMR5_CH4    TMR5_CH2
    7

    DMA2

    Channel Stream0     Stream1     Stream2     Stream3     Stream4     Stream5     Stream6     Stream7
    0                               TMR8_CH1                                        TMR1_CH1
                                    TMR8_CH2                                        TMR1_CH2
                                    TMR8_CH3                                        TMR1_CH3
    1
    2
    3
    4
    5
    6       TMR1_CH1    TMR1_CH2    TMR1_CH1                TMR1_CH4                TMR1_CH3
    7                   TMR8_CH1    TMR8_CH2    TMR8_CH3                                        TMR8_CH4
*/

uint32_t timerClock(const TIM_TypeDef *tim)
{
    if (tim == TMR8 || tim == TMR1 || tim == TMR9 || tim == TMR10 || tim == TMR11) {
        return SystemCoreClock;
    } else {
        return SystemCoreClock / 2;
    }
}
#endif
