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

#include "stm32f30x.h"
#include "rcc.h"
#include "timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),  .inputIrq = TIM1_CC_IRQn },
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2),  .inputIrq = TIM2_IRQn },
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3),  .inputIrq = TIM3_IRQn },
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4),  .inputIrq = TIM4_IRQn },
    { .TIMx = TIM6,  .rcc = RCC_APB1(TIM6),  .inputIrq = 0 },
    { .TIMx = TIM7,  .rcc = RCC_APB1(TIM7),  .inputIrq = 0 },
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),  .inputIrq = TIM8_CC_IRQn },
    { .TIMx = TIM15, .rcc = RCC_APB2(TIM15), .inputIrq = TIM1_BRK_TIM15_IRQn },
    { .TIMx = TIM16, .rcc = RCC_APB2(TIM16), .inputIrq = TIM1_UP_TIM16_IRQn },
    { .TIMx = TIM17, .rcc = RCC_APB2(TIM17), .inputIrq = TIM1_TRG_COM_TIM17_IRQn },
};

uint32_t timerClock(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return SystemCoreClock;
}
#endif

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {

        // Auto-generated from 'timer_def.h'
        // Search: \#define DEF_TIM_AF__(.*)__TCH_(.*)_([^ ]*).*
        // Replace: DEF_TIM($2, $3, $1, TIM_USE_ANY, 0),

DEF_TIM(TIM2, CH1, PA0, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH2, PA1, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH3, PA2, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH4, PA3, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH1, PA5, TIM_USE_ANY, 0),
DEF_TIM(TIM16, CH1, PA6, TIM_USE_ANY, 0),
DEF_TIM(TIM17, CH1, PA7, TIM_USE_ANY, 0),
DEF_TIM(TIM16, CH1, PA12, TIM_USE_ANY, 0),
DEF_TIM(TIM16, CH1N, PA13, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH1, PA15, TIM_USE_ANY, 0),

DEF_TIM(TIM3, CH2, PA4, TIM_USE_ANY, 0),
DEF_TIM(TIM3, CH1, PA6, TIM_USE_ANY, 0),
DEF_TIM(TIM3, CH2, PA7, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH1, PA15, TIM_USE_ANY, 0),

DEF_TIM(TIM8, CH1N, PA7, TIM_USE_ANY, 0),

DEF_TIM(TIM8, CH2, PA14, TIM_USE_ANY, 0),

DEF_TIM(TIM1, CH1N, PA7, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH1, PA8, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH2, PA9, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH3, PA10, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH1N, PA11, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH2N, PA12, TIM_USE_ANY, 0),

DEF_TIM(TIM15, CH1N, PA1, TIM_USE_ANY, 0),
DEF_TIM(TIM15, CH1, PA2, TIM_USE_ANY, 0),
DEF_TIM(TIM15, CH2, PA3, TIM_USE_ANY, 0),

DEF_TIM(TIM2, CH3, PA9, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH4, PA10, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH1, PA11, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH2, PA12, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH3, PA13, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH4, PA11, TIM_USE_ANY, 0),

DEF_TIM(TIM2, CH2, PB3, TIM_USE_ANY, 0),
DEF_TIM(TIM16, CH1, PB4, TIM_USE_ANY, 0),
DEF_TIM(TIM16, CH1N, PB6, TIM_USE_ANY, 0),
DEF_TIM(TIM17, CH1N, PB7, TIM_USE_ANY, 0),
DEF_TIM(TIM16, CH1, PB8, TIM_USE_ANY, 0),
DEF_TIM(TIM17, CH1, PB9, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH3, PB10, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH4, PB11, TIM_USE_ANY, 0),
DEF_TIM(TIM15, CH1, PB14, TIM_USE_ANY, 0),
DEF_TIM(TIM15, CH2, PB15, TIM_USE_ANY, 0),

DEF_TIM(TIM3, CH3, PB0, TIM_USE_ANY, 0),
DEF_TIM(TIM3, CH4, PB1, TIM_USE_ANY, 0),
DEF_TIM(TIM3, CH1, PB4, TIM_USE_ANY, 0),
DEF_TIM(TIM3, CH2, PB5, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH1, PB6, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH2, PB7, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH3, PB8, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH4, PB9, TIM_USE_ANY, 0),
DEF_TIM(TIM15, CH1N, PB15, TIM_USE_ANY, 0),

DEF_TIM(TIM8, CH3N, PB5, TIM_USE_ANY, 0),

DEF_TIM(TIM8, CH2N, PB0, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH3N, PB1, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH1N, PB3, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH2N, PB4, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH3N, PB15, TIM_USE_ANY, 0),

DEF_TIM(TIM8, CH1, PB6, TIM_USE_ANY, 0),

DEF_TIM(TIM1, CH2N, PB0, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH3N, PB1, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH1N, PB13, TIM_USE_ANY, 0),
DEF_TIM(TIM1, CH2N, PB14, TIM_USE_ANY, 0),

DEF_TIM(TIM17, CH1, PB5, TIM_USE_ANY, 0),
DEF_TIM(TIM3, CH4, PB7, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH2, PB8, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH3, PB9, TIM_USE_ANY, 0),

DEF_TIM(TIM3, CH1, PC6, TIM_USE_ANY, 0),
DEF_TIM(TIM3, CH2, PC7, TIM_USE_ANY, 0),
DEF_TIM(TIM3, CH3, PC8, TIM_USE_ANY, 0),
DEF_TIM(TIM3, CH4, PC9, TIM_USE_ANY, 0),

DEF_TIM(TIM8, CH1, PC6, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH2, PC7, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH3, PC8, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH4, PC9, TIM_USE_ANY, 0),

DEF_TIM(TIM8, CH1N, PC10, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH2N, PC11, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH3N, PC12, TIM_USE_ANY, 0),
DEF_TIM(TIM8, CH1N, PC13, TIM_USE_ANY, 0),

DEF_TIM(TIM2, CH1, PD3, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH2, PD4, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH4, PD6, TIM_USE_ANY, 0),
DEF_TIM(TIM2, CH3, PD7, TIM_USE_ANY, 0),

DEF_TIM(TIM4, CH1, PD12, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH2, PD13, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH3, PD14, TIM_USE_ANY, 0),
DEF_TIM(TIM4, CH4, PD15, TIM_USE_ANY, 0),

DEF_TIM(TIM8, CH4, PD1, TIM_USE_ANY, 0),

DEF_TIM(TIM15, CH1, PF9, TIM_USE_ANY, 0),
DEF_TIM(TIM15, CH2, PF10, TIM_USE_ANY, 0),

};
#endif
