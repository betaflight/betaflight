/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * porting for ch32h41x by Temperslee
 */
#include "platform.h"
#include "build/debug.h"
#include "drivers/io.h"

#include "build/debug_pin.h"

void debugInit(void)
{
    // IO_t io = IOGetByTag(DEFIO_TAG_E(PB9)); // SWDIO
    // if (IOGetOwner(io) == OWNER_FREE) {
    //     IOInit(io, OWNER_SWD, 0);
    // }
    // io = IOGetByTag(DEFIO_TAG_E(PB8));      // SWCLK
    // if (IOGetOwner(io) == OWNER_FREE) {
    //     IOInit(io, OWNER_SWD, 0);
    // }
}

#ifdef USE_DEBUG_PIN

dbgPin_t dbgPins[DEBUG_PIN_COUNT] = {
    { .tag = IO_TAG(DEBUG_PIN0) },
    { .tag = IO_TAG(DEBUG_PIN1)},
};

#endif
