/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            TIMER_OUTPUT_ENABLED|TIMER_OUTPUT_INVERTED, IOCFG_AF_PP, },
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               TIMER_OUTPUT_ENABLED|TIMER_OUTPUT_INVERTED, IOCFG_AF_PP, },
    { TIM15, IO_TAG(PB14), TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     TIMER_OUTPUT_ENABLED|TIMER_OUTPUT_INVERTED, IOCFG_AF_PP, },
    { TIM15, IO_TAG(PB15), TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     TIMER_OUTPUT_ENABLED|TIMER_OUTPUT_INVERTED, IOCFG_AF_PP, },
    { TIM16, IO_TAG(PA6),  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      TIMER_OUTPUT_ENABLED|TIMER_OUTPUT_INVERTED, IOCFG_AF_PP, },
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, TIMER_OUTPUT_ENABLED|TIMER_OUTPUT_INVERTED, IOCFG_AF_PP, },

    { TIM2,  IO_TAG(PB3),  TIM_Channel_2, TIM2_IRQn,               0, IOCFG_AF_PP, }, // TODO - Cleanup. KISS FC uses the same pin for serial and PPM
    { TIM2,  IO_TAG(PA15), TIM_Channel_1, TIM2_IRQn,               0, IOCFG_AF_PP, },
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM2_IRQn,               0, IOCFG_AF_PP, },
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM2_IRQn,               0, IOCFG_AF_PP, },
    { TIM4,  IO_TAG(PA13), TIM_Channel_2, TIM4_IRQn,               0, IOCFG_AF_PP, },
    { TIM8,  IO_TAG(PA14), TIM_Channel_3, TIM8_CC_IRQn,            0, IOCFG_AF_PP, },
};

