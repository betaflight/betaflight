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

#include <string.h>

#include "platform.h"

#if defined(USE_TIMER_MGMT) && defined(USE_TIMER_UP_CONFIG)

#include "drivers/dma_reqmap.h"
#include "drivers/timer.h"

#include "timerup.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(timerUpConfig_t, HARDWARE_TIMER_DEFINITION_COUNT, timerUpConfig, PG_TIMER_UP_CONFIG, 0);

void pgResetFn_timerUpConfig(timerUpConfig_t *config)
{
    for (unsigned timno = 0; timno < HARDWARE_TIMER_DEFINITION_COUNT; timno++) {
        config[timno].dmaopt = DMA_OPT_UNUSED;
    }

#if defined(TIMUP1_DMA_OPT) && (TIMUP_TIMERS & TIM_N(1))
    config[TIMER_INDEX(1)].dmaopt = TIMUP1_DMA_OPT;
#endif
#if defined(TIMUP2_DMA_OPT) && (TIMUP_TIMERS & TIM_N(2))
    config[TIMER_INDEX(2)].dmaopt = TIMUP2_DMA_OPT;
#endif
#if defined(TIMUP3_DMA_OPT) && (TIMUP_TIMERS & TIM_N(3))
    config[TIMER_INDEX(3)].dmaopt = TIMUP3_DMA_OPT;
#endif
#if defined(TIMUP4_DMA_OPT) && (TIMUP_TIMERS & TIM_N(4))
    config[TIMER_INDEX(4)].dmaopt = TIMUP4_DMA_OPT;
#endif
#if defined(TIMUP5_DMA_OPT) && (TIMUP_TIMERS & TIM_N(5))
    config[TIMER_INDEX(5)].dmaopt = TIMUP5_DMA_OPT;
#endif
#if defined(TIMUP6_DMA_OPT) && (TIMUP_TIMERS & TIM_N(6))
    config[TIMER_INDEX(6)].dmaopt = TIMUP6_DMA_OPT;
#endif
#if defined(TIMUP7_DMA_OPT) && (TIMUP_TIMERS & TIM_N(7))
    config[TIMER_INDEX(7)].dmaopt = TIMUP7_DMA_OPT;
#endif
#if defined(TIMUP8_DMA_OPT) && (TIMUP_TIMERS & TIM_N(8))
    config[TIMER_INDEX(8)].dmaopt = TIMUP8_DMA_OPT;
#endif
#if defined(TIMUP15_DMA_OPT) && (TIMUP_TIMERS & TIM_N(15))
    config[TIMER_INDEX(15)].dmaopt = TIMUP15_DMA_OPT;
#endif
#if defined(TIMUP16_DMA_OPT) && (TIMUP_TIMERS & TIM_N(16))
    config[TIMER_INDEX(16)].dmaopt = TIMUP16_DMA_OPT;
#endif
#if defined(TIMUP17_DMA_OPT) && (TIMUP_TIMERS & TIM_N(17))
    config[TIMER_INDEX(17)].dmaopt = TIMUP17_DMA_OPT;
#endif
#if defined(TIMUP20_DMA_OPT) && (TIMUP_TIMERS & TIM_N(20))
    config[TIMER_INDEX(20)].dmaopt = TIMUP20_DMA_OPT;
#endif
}
#endif
