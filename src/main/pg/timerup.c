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

#if defined(TIMUP1_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 0) && (TIMUP_TIMERS & BIT(1))
    config[0].dmaopt = TIMUP1_DMA_OPT;
#endif
#if defined(TIMUP2_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 1) && (TIMUP_TIMERS & BIT(2))
    config[1].dmaopt = TIMUP2_DMA_OPT;
#endif
#if defined(TIMUP3_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 2) && (TIMUP_TIMERS & BIT(3))
    config[2].dmaopt = TIMUP3_DMA_OPT;
#endif
#if defined(TIMUP4_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 3) && (TIMUP_TIMERS & BIT(4))
    config[3].dmaopt = TIMUP4_DMA_OPT;
#endif
#if defined(TIMUP5_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 4) && (TIMUP_TIMERS & BIT(5))
    config[4].dmaopt = TIMUP5_DMA_OPT;
#endif
#if defined(TIMUP6_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 5) && (TIMUP_TIMERS & BIT(6))
    config[5].dmaopt = TIMUP6_DMA_OPT;
#endif
#if defined(TIMUP7_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 6) && (TIMUP_TIMERS & BIT(7))
    config[6].dmaopt = TIMUP7_DMA_OPT;
#endif
#if defined(TIMUP8_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 7) && (TIMUP_TIMERS & BIT(8))
    config[7].dmaopt = TIMUP8_DMA_OPT;
#endif
#if defined(TIMUP15_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 14) && (TIMUP_TIMERS & BIT(15))
    config[14].dmaopt = TIMUP15_DMA_OPT;
#endif
#if defined(TIMUP16_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 15) && (TIMUP_TIMERS & BIT(16))
    config[15].dmaopt = TIMUP16_DMA_OPT;
#endif
#if defined(TIMUP17_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 16) && (TIMUP_TIMERS & BIT(17))
    config[16].dmaopt = TIMUP17_DMA_OPT;
#endif
#if defined(TIMUP20_DMA_OPT) && (HARDWARE_TIMER_DEFINITION_COUNT > 19) && (TIMUP_TIMERS & BIT(20))
    config[19].dmaopt = TIMUP20_DMA_OPT;
#endif
}
#endif
