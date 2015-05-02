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
#include <stdint.h>

#include "platform.h"

#include "build_config.h"

#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "rx/msp.h"

static bool rxMspFrameDone = false;

static uint16_t rxMspReadRawRC(rxRuntimeConfig_t *rxRuntimeConfigPtr, uint8_t chan)
{
    UNUSED(rxRuntimeConfigPtr);
    return rcData[chan];
}

void rxMspFrameRecieve(void)
{
    rxMspFrameDone = true;
}

bool rxMspFrameComplete(void)
{
    if (!rxMspFrameDone) {
        return false;
    }

    rxMspFrameDone = false;
    return true;
}

void rxMspInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxConfig);
    rxRuntimeConfig->channelCount = 8; // Limited to 8 channels due to MSP_SET_RAW_RC command.
    if (callback)
        *callback = rxMspReadRawRC;
}
