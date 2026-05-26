/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_TIMER

#include "timer_def.h"
#include "platform/rcc.h"
#include "drivers/timer.h"
#include "platform/timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = ATIM1,  .rcc = RCC_APB2_1(ATIM1),  .inputIrq = ATIM1_CC_IRQn },
    { .TIMx = ATIM2,  .rcc = RCC_APB2_1(ATIM2),  .inputIrq = ATIM2_CC_IRQn },
    { .TIMx = GTIMA1, .rcc = RCC_APB2_1(GTIMA1), .inputIrq = GTIMA1_IRQn },
    { .TIMx = GTIMA2, .rcc = RCC_APB2_1(GTIMA2), .inputIrq = GTIMA2_IRQn },
    { .TIMx = GTIMA3, .rcc = RCC_APB2_1(GTIMA3), .inputIrq = GTIMA3_IRQn },
    { .TIMx = GTIMA4, .rcc = RCC_APB1_1(GTIMA4), .inputIrq = GTIMA4_IRQn },
    { .TIMx = GTIMA5, .rcc = RCC_APB1_2(GTIMA5), .inputIrq = GTIMA5_IRQn },
    { .TIMx = GTIMA6, .rcc = RCC_APB1_2(GTIMA6), .inputIrq = GTIMA6_IRQn },
    { .TIMx = GTIMA7, .rcc = RCC_APB1_2(GTIMA7), .inputIrq = GTIMA7_IRQn },
    { .TIMx = GTIMB1, .rcc = RCC_APB1_1(GTIMB1), .inputIrq = GTIMB1_IRQn },
    { .TIMx = GTIMB2, .rcc = RCC_APB1_1(GTIMB2), .inputIrq = GTIMB2_IRQn },
    { .TIMx = GTIMB3, .rcc = RCC_APB1_1(GTIMB3), .inputIrq = GTIMB3_IRQn },
    { .TIMx = ATIM3,  .rcc = RCC_APB5_1(ATIM3),  .inputIrq = ATIM3_CC_IRQn },
    { .TIMx = ATIM4,  .rcc = RCC_APB5_1(ATIM4),  .inputIrq = ATIM4_CC_IRQn },
};

const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
// Port A
    DEF_TIM(GTIMA1, CH1, PA0, 0, 0, 0),
    DEF_TIM(GTIMA4, CH1, PA0, 0, 0, 0),
    DEF_TIM(GTIMA1, CH2, PA1, 0, 0, 0),
    DEF_TIM(GTIMA4, CH2, PA1, 0, 0, 0),
    DEF_TIM(GTIMB1, CH1N, PA1, 0, 0, 0),
    DEF_TIM(GTIMA1, CH3, PA2, 0, 0, 0),
    DEF_TIM(GTIMA4, CH3, PA2, 0, 0, 0),
    DEF_TIM(GTIMB1, CH1, PA2, 0, 0, 0),
    DEF_TIM(GTIMA1, CH4, PA3, 0, 0, 0),
    DEF_TIM(GTIMA4, CH4, PA3, 0, 0, 0),
    DEF_TIM(GTIMB1, CH2, PA3, 0, 0, 0),
    DEF_TIM(ATIM2, CH1N, PA5, 0, 0, 0),
    DEF_TIM(GTIMA1, CH1, PA5, 0, 0, 0),
    DEF_TIM(ATIM3, CH1, PA6, 0, 0, 0),
    DEF_TIM(GTIMA2, CH1, PA6, 0, 0, 0),
    DEF_TIM(ATIM1, CH1N, PA7, 0, 0, 0),
    DEF_TIM(ATIM2, CH1N, PA7, 0, 0, 0),
    DEF_TIM(ATIM4, CH1, PA7, 0, 0, 0),
    DEF_TIM(GTIMA2, CH2, PA7, 0, 0, 0),
    DEF_TIM(ATIM1, CH1, PA8, 0, 0, 0),
    DEF_TIM(ATIM1, CH2, PA9, 0, 0, 0),
    DEF_TIM(ATIM1, CH3, PA10, 0, 0, 0),
    DEF_TIM(ATIM1, CH4, PA11, 0, 0, 0),
    DEF_TIM(GTIMA1, CH1, PA15, 0, 0, 0),

// Port B
    DEF_TIM(ATIM1, CH3N, PB1, 0, 0, 0),
    DEF_TIM(ATIM2, CH3N, PB1, 0, 0, 0),
    DEF_TIM(GTIMA2, CH4, PB1, 0, 0, 0),
    DEF_TIM(GTIMA1, CH2, PB3, 0, 0, 0),
    DEF_TIM(GTIMA2, CH1, PB4, 0, 0, 0),
    DEF_TIM(GTIMA2, CH2, PB5, 0, 0, 0),
    DEF_TIM(GTIMA2, CH3, PB6, 0, 0, 0),
    DEF_TIM(GTIMA3, CH1, PB6, 0, 0, 0),
    DEF_TIM(GTIMB2, CH1N, PB6, 0, 0, 0),
    DEF_TIM(GTIMA2, CH4, PB7, 0, 0, 0),
    DEF_TIM(GTIMA3, CH2, PB7, 0, 0, 0),
    DEF_TIM(GTIMB3, CH1N, PB7, 0, 0, 0),
    DEF_TIM(GTIMA3, CH3, PB8, 0, 0, 0),
    DEF_TIM(GTIMB2, CH1, PB8, 0, 0, 0),
    DEF_TIM(GTIMA3, CH4, PB9, 0, 0, 0),
    DEF_TIM(GTIMB3, CH1, PB9, 0, 0, 0),
    DEF_TIM(GTIMA1, CH3, PB10, 0, 0, 0),
    DEF_TIM(GTIMA1, CH4, PB11, 0, 0, 0),
    DEF_TIM(ATIM1, CH1N, PB13, 0, 0, 0),
    DEF_TIM(ATIM1, CH2N, PB14, 0, 0, 0),
    DEF_TIM(ATIM2, CH2N, PB14, 0, 0, 0),
    DEF_TIM(GTIMA7, CH1, PB14, 0, 0, 0),
    DEF_TIM(ATIM1, CH3N, PB15, 0, 0, 0),
    DEF_TIM(ATIM2, CH3N, PB15, 0, 0, 0),
    DEF_TIM(GTIMA7, CH2, PB15, 0, 0, 0),

// Port C
    DEF_TIM(ATIM2, CH1, PC6, 0, 0, 0),
    DEF_TIM(GTIMA2, CH1, PC6, 0, 0, 0),
    DEF_TIM(ATIM2, CH2, PC7, 0, 0, 0),
    DEF_TIM(GTIMA2, CH2, PC7, 0, 0, 0),
    DEF_TIM(ATIM2, CH3, PC8, 0, 0, 0),
    DEF_TIM(GTIMA2, CH3, PC8, 0, 0, 0),
    DEF_TIM(ATIM2, CH4, PC9, 0, 0, 0),
    DEF_TIM(GTIMA2, CH4, PC9, 0, 0, 0),
    DEF_TIM(GTIMB1, CH1, PC12, 0, 0, 0),

// Port D
    DEF_TIM(GTIMB2, CH1N, PD0, 0, 0, 0),
    DEF_TIM(GTIMB2, CH1, PD1, 0, 0, 0),
    DEF_TIM(GTIMB2, CH2, PD2, 0, 0, 0),
    DEF_TIM(GTIMB2, CH3, PD3, 0, 0, 0),
    DEF_TIM(GTIMB2, CH4, PD4, 0, 0, 0),
    DEF_TIM(ATIM3, CH3, PD8, 0, 0, 0),
    DEF_TIM(ATIM3, CH3N, PD9, 0, 0, 0),
    DEF_TIM(ATIM1, CH4N, PD10, 0, 0, 0),
    DEF_TIM(GTIMA3, CH1, PD12, 0, 0, 0),
    DEF_TIM(GTIMA3, CH2, PD13, 0, 0, 0),
    DEF_TIM(ATIM3, CH4, PD14, 0, 0, 0),
    DEF_TIM(GTIMA3, CH3, PD14, 0, 0, 0),
    DEF_TIM(ATIM3, CH4N, PD15, 0, 0, 0),
    DEF_TIM(GTIMA3, CH4, PD15, 0, 0, 0),

// Port E
    DEF_TIM(GTIMB1, CH3, PE0, 0, 0, 0),
    DEF_TIM(GTIMB1, CH4, PE1, 0, 0, 0),
    DEF_TIM(GTIMB1, CH1N, PE4, 0, 0, 0),
    DEF_TIM(GTIMB1, CH1, PE5, 0, 0, 0),
    DEF_TIM(GTIMB1, CH2, PE6, 0, 0, 0),
    DEF_TIM(ATIM1, CH1N, PE8, 0, 0, 0),
    DEF_TIM(ATIM1, CH1, PE9, 0, 0, 0),
    DEF_TIM(ATIM1, CH2N, PE10, 0, 0, 0),
    DEF_TIM(ATIM1, CH2, PE11, 0, 0, 0),
    DEF_TIM(ATIM1, CH3N, PE12, 0, 0, 0),
    DEF_TIM(ATIM1, CH3, PE13, 0, 0, 0),
    DEF_TIM(ATIM1, CH4, PE14, 0, 0, 0),
};

uint32_t timerClock(const timerHardware_t *timHw)
{
    return timerClockFromInstance(timHw->tim);
}

#endif
