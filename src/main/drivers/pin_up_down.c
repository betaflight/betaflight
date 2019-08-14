/*
 * This file is part of Betaflight.
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

#ifdef USE_PIN_UP_DOWN

#include "build/debug.h"

#include "pg/pin_up_down.h"

#include "drivers/io.h"

static void initPins(const pinUpDownConfig_t * config, resourceOwner_e owner)
{
    for (int i = 0; i < PIN_UP_DOWN_COUNT; i++) {
        IO_t io = IOGetByTag(config->ioTag[i]);

        if (!io) {
            continue;
        }

        IOInit(io, OWNER_PINIO, RESOURCE_INDEX(i));

        if (owner == OWNER_PULLUP) {
            IOConfigGPIO(io, IOCFG_IPU);
        } else if (owner == OWNER_PULLDOWN) {
            IOConfigGPIO(io, IOCFG_IPD);
        }
    }
}

void pinPullupInit(const pinUpDownConfig_t * pullupConfig)
{
    initPins(pullupConfig, OWNER_PULLUP);
}

void pinPulldownInit(const pinUpDownConfig_t * pulldownConfig)
{
    initPins(pulldownConfig, OWNER_PULLDOWN);
}

#endif
