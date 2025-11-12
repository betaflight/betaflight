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

#include "platform.h"

#include "build/debug_pin.h"

#ifdef USE_DEBUG_PIN

#ifndef DEBUG_PIN_COUNT
#define DEBUG_PIN_COUNT 0
#endif

#include "drivers/io.h"
#include "drivers/io_impl.h"

dbgPin_t dbgPins[DEBUG_PIN_COUNT] = {
#ifdef DEBUG_PIN1
    { .tag = IO_TAG(DEBUG_PIN1) },
#endif
#ifdef DEBUG_PIN2
    { .tag = IO_TAG(DEBUG_PIN2) },
#endif
};
IO_t dbgPinIOs[DEBUG_PIN_COUNT] = { 0 };
#endif

void dbgPinInit(void)
{
#ifdef USE_DEBUG_PIN
    for (unsigned i = 0; i < ARRAYLEN(dbgPins); i++) {
        const dbgPin_t *dbgPin = &dbgPins[i];
        IO_t io = IOGetByTag(dbgPin->tag);
        if (!io) {
            continue;
        }
        IOInit(io, OWNER_SYSTEM, 0);
        IOConfigGPIO(io, IOCFG_OUT_PP);
        dbgPinIOs[i] = io;
    }
#endif
}

void dbgPinHi(int index)
{
#ifdef USE_DEBUG_PIN
    if ((unsigned)index >= ARRAYLEN(dbgPinIOs) || !dbgPinIOs[index]) {
        return;
    }

    IOHi(dbgPinIOs[index]);
#else
    UNUSED(index);
#endif
}

void dbgPinLo(int index)
{
#ifdef USE_DEBUG_PIN
    if ((unsigned)index >= ARRAYLEN(dbgPinIOs) || !dbgPinIOs[index]) {
        return;
    }

    IOLo(dbgPinIOs[index]);
#else
    UNUSED(index);
#endif
}
