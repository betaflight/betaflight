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

#ifdef USE_PIN_PULL_UP_DOWN

#include "build/debug.h"
#include "drivers/io.h"
#include "pg/pin_pull_up_down.h"
#include "pin_pull_up_down.h"


static void initPin(const pinPullUpDownConfig_t* config, resourceOwner_e owner, uint8_t index)
{
    IO_t io = IOGetByTag(config->ioTag);
    if (!io) {
        return;
    }

    IOInit(io, owner, RESOURCE_INDEX(index));

    if (owner == OWNER_PULLUP) {
        IOConfigGPIO(io, IOCFG_IPU);
    } else if (owner == OWNER_PULLDOWN) {
        IOConfigGPIO(io, IOCFG_IPD);
    }
}

void pinPullupPulldownInit(void)
{
    for (uint8_t i = 0; i < PIN_PULL_UP_DOWN_COUNT; i++) {
        initPin(pinPullupConfig(i), OWNER_PULLUP, i);
        initPin(pinPulldownConfig(i), OWNER_PULLDOWN, i);
    }
}

#endif
