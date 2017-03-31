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
 *   - initial implementation for MultiWii by Cesco/Plüschi
 *   - implementation for BaseFlight by Andreas (fiendie) Tacke
 *   - ported to CleanFlight by Konstantin (digitalentity) Sharlaimov
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_IBUS

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "telemetry/ibus.h"
#include "telemetry/ibus_shared.h"
    
#include "rx/rx.h"
#include "rx/ibus.h"


#define IBUS_MAX_CHANNEL (10)
#define IBUS_TELEMETRY_PACKET_LENGTH (4)
#define IBUS_SERIAL_RX_PACKET_LENGTH (32)

static uint8_t rxBytesToIgnore = 0;
static uint8_t ibusFramePosition = 0;
static bool ibusFrameDone = false;
static uint32_t ibusChannelData[IBUS_MAX_CHANNEL];
static uint8_t ibus[IBUS_SERIAL_RX_PACKET_LENGTH] = { 0, };

static bool isValidIbusPacketLength(uint8_t length) {
    return (length == IBUS_TELEMETRY_PACKET_LENGTH) || (length == IBUS_SERIAL_RX_PACKET_LENGTH);
}

// Receive ISR callback
static void ibusDataReceive(uint16_t c)
{
    timeUs_t ibusTime;
    static timeUs_t ibusTimeLast;
    static uint8_t ibusLength = 0;

    ibusTime = micros();

    if (cmpTimeUs(ibusTime, ibusTimeLast) > 3000) {
        ibusFramePosition = 0;
        ibusLength = 0;
        rxBytesToIgnore = 0;
    }
    if (rxBytesToIgnore) {
        rxBytesToIgnore--;
        return;
    }

    ibusTimeLast = ibusTime;

    if (ibusFramePosition == 0) {
       if (isValidIbusPacketLength(c)) {
           ibusLength = c;
       } else {
           return;
       }
    }

    ibus[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == ibusLength - 1) {
        ibusFrameDone = true;
    } else {
        ibusFramePosition++;
    }
}

uint8_t ibusFrameStatus(void)
{
    uint8_t i, offset;
    uint8_t frameStatus = RX_FRAME_PENDING;
    uint8_t ibusLength;

    if (!ibusFrameDone) {
        return frameStatus;
    }
    
    ibusLength = ibus[0];
    ibusFrameDone = false;
   
    if (isChecksumOk(ibus, ibusLength)) {
        if (ibusLength == IBUS_SERIAL_RX_PACKET_LENGTH) {
            for (i = 0, offset = 2; i < IBUS_MAX_CHANNEL; i++, offset += 2) {
                ibusChannelData[i] = ibus[offset] + (ibus[offset + 1] << 8);
            }
            frameStatus = RX_FRAME_COMPLETE;
        } else {
#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)
            rxBytesToIgnore = respondToIbusRequest(ibus);
#endif
        }
    }
    ibusFramePosition = 0;

    return frameStatus;
}

static uint16_t ibusReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return ibusChannelData[chan];
}

bool ibusInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    serialPort_t *ibusPort = NULL;
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = IBUS_MAX_CHANNEL;
    rxRuntimeConfig->rxRefreshRate = 20000; // TODO - Verify speed 11000 ?

    rxRuntimeConfig->rcReadRawFn = ibusReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = ibusFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }
    rxBytesToIgnore = 0;
#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)
    if (isSerialPortShared(portConfig, FUNCTION_RX_SERIAL, FUNCTION_TELEMETRY_IBUS)) {
        //combined, open port as bidirectional
        ibusPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, ibusDataReceive, IBUS_BAUDRATE, MODE_RXTX, SERIAL_BIDIR | SERIAL_NOT_INVERTED);
        initSharedIbusTelemetry(ibusPort);
    } else 
#endif
    {
        ibusPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, ibusDataReceive, IBUS_BAUDRATE, MODE_RX, SERIAL_NOT_INVERTED);
    }    
    return ibusPort != NULL;
}
#endif // USE_SERIALRX_IBUS
