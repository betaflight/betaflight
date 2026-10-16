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

/*
 * Lookup helpers for the HPMicro extended timer hardware table.
 */

#include "platform.h"
#include "timer_hw_ext.h"

#include "drivers/io.h"

FAST_CODE const hpmicroTimerHwExt_t *hpmicroTimerHwExtByTag(ioTag_t tag)
{
    for (int i = 0; i < FULL_TIMER_CHANNEL_COUNT; i++) {
        if (hpmicroTimerHwExt[i].tag == tag) {
            return &hpmicroTimerHwExt[i];
        }
    }
    return NULL;
}

FAST_CODE const hpmicroTimerHwExt_t *hpmicroTimerHwExtByTimer(const timerHardware_t *timHw)
{
    if (timHw == NULL) {
        return NULL;
    }
    return hpmicroTimerHwExtByTag(timHw->tag);
}
