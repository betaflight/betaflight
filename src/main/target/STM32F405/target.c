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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
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
};
