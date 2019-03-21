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

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/timer_def.h"

#include "stm32f7xx.h"
#include "rcc.h"
#include "timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),  .inputIrq = TIM1_CC_IRQn},
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2),  .inputIrq = TIM2_IRQn},
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3),  .inputIrq = TIM3_IRQn},
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4),  .inputIrq = TIM4_IRQn},
    { .TIMx = TIM5,  .rcc = RCC_APB1(TIM5),  .inputIrq = TIM5_IRQn},
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6),  .inputIrq = 0},
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7),  .inputIrq = 0},
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
//PORTA
    DEF_TIM(TIM2, CH1, PA0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH2, PA1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH3, PA2, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH4, PA3, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH1, PA5, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1N, PA7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH2, PA9, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3, PA10, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1N, PA11, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH1, PA15, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM5, CH1, PA0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM5, CH2, PA1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM5, CH3, PA2, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM5, CH4, PA3, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH1, PA6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH2, PA7, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM9, CH1, PA2, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM9, CH2, PA3, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH1N, PA5, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH1N, PA7, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM13, CH1, PA6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM14, CH1, PA7, TIM_USE_ANY, 0, 0),

//PORTB
    DEF_TIM(TIM1, CH2N, PB0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3N, PB1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH2, PB3, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH3, PB10, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM2, CH4, PB11, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1N, PB13, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH2N, PB14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3N, PB15, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM3, CH3, PB0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH1, PB4, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH2, PB5, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH1, PB6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH3, PB8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH4, PB9, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM8, CH2N, PB0, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH3N, PB1, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM10, CH1, PB8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM11, CH1, PB9, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH2N, PB14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH3N, PB15, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM12, CH1, PB14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM12, CH2, PB15, TIM_USE_ANY, 0, 0),

//PORTC
    DEF_TIM(TIM3, CH1, PC6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH2, PC7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH3, PC8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM3, CH4, PC9, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM8, CH1, PC6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH2, PC7, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_ANY, 0, 0),

//PORTD
    DEF_TIM(TIM4, CH1, PD12, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH2, PD13, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH3, PD14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM4, CH4, PD15, TIM_USE_ANY, 0, 0),

//PORTE
    DEF_TIM(TIM1, CH1N, PE8, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH1, PE9, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH2N, PE10, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3N, PE12, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM1, CH4, PE14, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM9, CH1, PE5, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM9, CH2, PE6, TIM_USE_ANY, 0, 0),

//PORTF
    DEF_TIM(TIM10, CH1, PF6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM11, CH1, PF7, TIM_USE_ANY, 0, 0),

//PORTH
    DEF_TIM(TIM5, CH1, PH10, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM5, CH2, PH11, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM5, CH3, PH12, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM8, CH1N, PH13, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH2N, PH14, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM8, CH3N, PH15, TIM_USE_ANY, 0, 0),

    DEF_TIM(TIM12, CH1, PH6, TIM_USE_ANY, 0, 0),
    DEF_TIM(TIM12, CH2, PH9, TIM_USE_ANY, 0, 0),

//PORTI
// Not yet used
//    DEF_TIM(TIM5, CH4, PI0, TIM_USE_ANY, 0, 0),
//
//    DEF_TIM(TIM8, CH4, PI2, TIM_USE_ANY, 0, 0),
//    DEF_TIM(TIM8, CH1, PI5, TIM_USE_ANY, 0, 0),
//    DEF_TIM(TIM8, CH2, PI6, TIM_USE_ANY, 0, 0),
//    DEF_TIM(TIM8, CH3, PI7, TIM_USE_ANY, 0, 0),
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
    2       TIM4_CH1                            TIM4_CH2                                        TIM4_CH3
    3                   TIM2_CH3                                        TIM2_CH1    TIM2_CH1    TIM2_CH4
                                                                                    TIM2_CH4
    4
    5                               TIM3_CH4                TIM3_CH1    TIM3_CH2                TIM3_CH3
    6       TIM5_CH3    TIM5_CH4    TIM5_CH1    TIM5_CH4    TIM5_CH2
    7

    DMA2

    Channel Stream0     Stream1     Stream2     Stream3     Stream4     Stream5     Stream6     Stream7
    0                               TIM8_CH1                                        TIM1_CH1
                                    TIM8_CH2                                        TIM1_CH2
                                    TIM8_CH3                                        TIM1_CH3
    1
    2
    3
    4
    5
    6                   TIM1_CH1    TIM1_CH2    TIM1_CH1    TIM1_CH4                TIM1_CH3
    7                               TIM8_CH1    TIM8_CH2    TIM8_CH3                            TIM8_CH4
*/

uint32_t timerClock(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return SystemCoreClock;
}
