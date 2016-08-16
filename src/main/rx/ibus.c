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
 *
 *
 * Driver for IBUS (Flysky) receiver
 *   - initial implementation for MultiWii by Cesco/Pl¸schi
 *   - implementation for BaseFlight by Andreas (fiendie) Tacke
 *   - ported to CleanFlight by Konstantin (digitalentity) Sharlaimov
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "build/build_config.h"

#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#ifdef TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "rx/rx.h"
#include "rx/ibus.h"

#define IBUS_MAX_CHANNEL 14
#define IBUS_BUFFSIZE 32
#define IBUS_MODEL_IA6B 0
#define IBUS_MODEL_IA6 1

#define IBUS_BAUDRATE 115200

static uint8_t ibusModel = 0;
static uint8_t ibusSyncByte = 0x20;
static uint8_t ibusFrameSize = 32;
static uint8_t ibusChannelOffset = 2;
static uint32_t ibusInterframeGap = 3000;
static uint16_t ibusChecksum = 0xFFFF;

static bool ibusFrameDone = false;
static uint32_t ibusChannelData[IBUS_MAX_CHANNEL];

static void ibusDataReceive(uint16_t c);
static uint16_t ibusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

bool ibusInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{

    if (rxConfig->ibus_model == IBUS_MODEL_IA6) {
        ibusModel = 1;
        ibusSyncByte = 0x55;
        ibusFrameSize = 31;
        ibusInterframeGap = 500;
        ibusChecksum = 0x0000;
        ibusChannelOffset = 1;
    }

    if (callback)
        *callback = ibusReadRawRC;

    rxRuntimeConfig->channelCount = IBUS_MAX_CHANNEL;

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig);
#else
    bool portShared = false;
#endif

    serialPort_t *ibusPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, ibusDataReceive, IBUS_BAUDRATE, portShared ? MODE_RXTX : MODE_RX, SERIAL_NOT_INVERTED);

#ifdef TELEMETRY
    if (portShared) {
        telemetrySharedPort = ibusPort;
    }
#endif

    return ibusPort != NULL;
}

static uint8_t ibus[IBUS_BUFFSIZE] = { 0, };

// Receive ISR callback
static void ibusDataReceive(uint16_t c)
{
    uint32_t ibusTime;
    static uint32_t ibusTimeLast;
    static uint8_t ibusFramePosition;

    ibusTime = micros();

    if ((ibusTime - ibusTimeLast) > ibusInterframeGap) 
        ibusFramePosition = 0;

    ibusTimeLast = ibusTime;

    if (ibusFramePosition == 0 && c != ibusSyncByte)
        return;

    ibus[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == ibusFrameSize - 1) {
        ibusFrameDone = true;
    } else {
        ibusFramePosition++;
    }
}

uint8_t ibusFrameStatus(void)
{
    uint8_t i, offset;
    uint8_t frameStatus = SERIAL_RX_FRAME_PENDING;
    uint16_t chksum, rxsum;

    if (!ibusFrameDone) {
        return frameStatus;
    }

    ibusFrameDone = false;

    chksum = ibusChecksum;
    rxsum = ibus[ibusFrameSize - 2] + (ibus[ibusFrameSize - 1] << 8);
    
    if (ibusModel == IBUS_MODEL_IA6) {
	    for (i = 0, offset = ibusChannelOffset; i < IBUS_MAX_CHANNEL; i++, offset += 2)
		chksum += ibus[offset] + (ibus[offset + 1] << 8);
    } else { 
	    for (i = 0; i < 30; i++)
		chksum -= ibus[i];
    }

    if (chksum == rxsum) {
        for (i = 0, offset = ibusChannelOffset; i < IBUS_MAX_CHANNEL; i++, offset += 2) {
	    ibusChannelData[i] = ibus[offset] + (ibus[offset + 1] << 8);
	}
	frameStatus = SERIAL_RX_FRAME_COMPLETE;
    }

    return frameStatus;
}

static uint16_t ibusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return ibusChannelData[chan];
}
