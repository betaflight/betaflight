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

#if defined(USE_TIMER_MGMT) && (defined(STM32H7) || defined(STM32G4))

#include "drivers/dma_reqmap.h"
#include "drivers/timer.h"

#include "timerup.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(timerUpConfig_t, HARDWARE_TIMER_DEFINITION_COUNT, timerUpConfig, PG_TIMER_UP_CONFIG, 0);

void pgResetFn_timerUpConfig(timerUpConfig_t *config)
{
    for (unsigned timno = 0; timno < HARDWARE_TIMER_DEFINITION_COUNT; timno++) {
        config[timno].dmaopt = DMA_OPT_UNUSED;
    }

#if USABLE_TIMER_CHANNEL_COUNT > 0
    // Scan target timerHardware and extract dma option for TIMUP
    for (unsigned i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        const timerHardware_t *timhw = &timerHardware[i];
        uint8_t timnum = timerGetTIMNumber(timhw->tim) - 1;
        config[timnum].dmaopt = dmaGetUpOptionByTimer(timhw);
    }
#endif
}
#endif
