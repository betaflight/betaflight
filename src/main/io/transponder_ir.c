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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#include <platform.h>

#include <build/build_config.h>

#include "drivers/transponder_ir.h"
#include "drivers/system.h"
#include "drivers/usb_io.h"

#include "fc/config.h"

#include "io/transponder_ir.h"

static bool transponderInitialised = false;
static bool transponderRepeat = false;

// timers
static timeUs_t nextUpdateAtUs = 0;

#define JITTER_DURATION_COUNT (sizeof(jitterDurations) / sizeof(uint8_t))
static uint8_t jitterDurations[] = {0,9,4,8,3,9,6,7,1,6,9,7,8,2,6};

void transponderUpdate(timeUs_t currentTimeUs)
{
    static uint32_t jitterIndex = 0;

    if (!(transponderInitialised && transponderRepeat && isTransponderIrReady())) {
        return;
    }

    const bool updateNow = (timeDelta_t)(currentTimeUs - nextUpdateAtUs) >= 0L;
    if (!updateNow) {
        return;
    }

    // TODO use a random number genenerator for random jitter?  The idea here is to avoid multiple transmitters transmitting at the same time.
    uint32_t jitter = (1000 * jitterDurations[jitterIndex++]);
    if (jitterIndex >= JITTER_DURATION_COUNT) {
        jitterIndex = 0;
    }

    nextUpdateAtUs = currentTimeUs + 4500 + jitter;

#ifdef REDUCE_TRANSPONDER_CURRENT_DRAW_WHEN_USB_CABLE_PRESENT
    // reduce current draw when USB cable is plugged in by decreasing the transponder transmit rate.
    if (usbCableIsInserted()) {
        nextUpdateAtUs = currentTimeUs + (1000 * 1000) / 10; // 10 hz.
    }
#endif

    transponderIrTransmit();
}

void transponderInit(uint8_t* transponderData)
{
    transponderInitialised = transponderIrInit();
    if (!transponderInitialised) {
        return;
    }

    transponderIrUpdateData(transponderData);
}

void transponderStopRepeating(void)
{
    transponderRepeat = false;
}

void transponderStartRepeating(void)
{
    if (!transponderInitialised) {
        return;
    }

    transponderRepeat = true;
}

void transponderUpdateData(uint8_t* transponderData)
{
    if (!transponderInitialised) {
        return;
    }

    transponderIrUpdateData(transponderData);
}

void transponderTransmitOnce(void) {

    if (!transponderInitialised) {
        return;
    }
    transponderIrTransmit();
}
