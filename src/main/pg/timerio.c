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

#ifdef USE_TIMER_MGMT

#include "drivers/dma_reqmap.h"
#include "drivers/timer.h"

#include "timerio.h"

/*
Details of the TIMER_PIN_MAP macro:
    i => is the index in the PG timer IO config array,
    p => is the hardware pin to be mapped,
    o => is the hardware pin occurrence (1 based index) within timerHardware (full) that should be used
                e.g. 1 = first occurrence of the pin, 2 = second occurrence, etc.
    d => the configured dma opt for the pin
*/
#define TIMER_PIN_MAP(i, p, o, d)  \
        { config[i].ioTag = IO_TAG(p); config[i].index = o; config[i].dmaopt = d; }

PG_REGISTER_ARRAY_WITH_RESET_FN(timerIOConfig_t, MAX_TIMER_PINMAP_COUNT, timerIOConfig, PG_TIMER_IO_CONFIG, 0);

void pgResetFn_timerIOConfig(timerIOConfig_t *config)
{
#ifdef TIMER_PIN_MAPPING
    TIMER_PIN_MAPPING
#else
    UNUSED(config);
#endif
}
#endif
