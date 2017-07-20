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

#include "common/utils.h"
#include "common/time.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "telemetry/telemetry.h"

#include "rx/rx.h"
#include "rx/ibus.h"
#include "telemetry/ibus.h"
#include "telemetry/ibus_shared.h"

#define IBUS_MAX_CHANNEL 14
#define IBUS_BUFFSIZE 32
#define IBUS_MODEL_IA6B 0
#define IBUS_MODEL_IA6 1
#define IBUS_FRAME_GAP 500

#define IBUS_TELEMETRY_PACKET_LENGTH (4)
#define IBUS_SERIAL_RX_PACKET_LENGTH (32)

static uint8_t ibusModel;
static uint8_t ibusSyncByte;
static uint8_t ibusFrameSize;
static uint8_t ibusChannelOffset;
static uint8_t rxBytesToIgnore;
static uint16_t ibusChecksum;

static bool ibusFrameDone = false;
static uint32_t ibusChannelData[IBUS_MAX_CHANNEL];

static uint8_t ibus[IBUS_BUFFSIZE] = { 0, };


static bool isValidIa6bIbusPacketLength(uint8_t length)
{
    return (length == IBUS_TELEMETRY_PACKET_LENGTH) || (length == IBUS_SERIAL_RX_PACKET_LENGTH);
}


// Receive ISR callback
static void ibusDataReceive(uint16_t c)
{
    timeUs_t ibusTime;
    static timeUs_t ibusTimeLast;
    static uint8_t ibusFramePosition;

    ibusTime = micros();

    if ((ibusTime - ibusTimeLast) > IBUS_FRAME_GAP) {
        ibusFramePosition = 0;
        rxBytesToIgnore = 0;
    } else if (rxBytesToIgnore) {
        rxBytesToIgnore--;
        return;
    }

    ibusTimeLast = ibusTime;

    if (ibusFramePosition == 0) {
        if (isValidIa6bIbusPacketLength(c)) {
            ibusModel = IBUS_MODEL_IA6B;
            ibusSyncByte = c;
            ibusFrameSize = c;
            ibusChannelOffset = 2;
            ibusChecksum = 0xFFFF;
        } else if ((ibusSyncByte == 0) && (c == 0x55)) {
            ibusModel = IBUS_MODEL_IA6;
            ibusSyncByte = 0x55;
            ibusFrameSize = 31;
            ibusChecksum = 0x0000;
            ibusChannelOffset = 1;
        } else if (ibusSyncByte != c) {
            return;
        }
    }

    ibus[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == ibusFrameSize - 1) {
        ibusFrameDone = true;
    } else {
        ibusFramePosition++;
    }
}

uint16_t ibusCalculateChecksum(const uint8_t *ibusPacket, size_t packetLength)
{
    uint16_t checksum = 0xFFFF;
    for (size_t i = 0; i < packetLength - IBUS_CHECKSUM_SIZE; i++) {
        checksum -= ibusPacket[i];
    }

    return checksum;
}

static bool isChecksumOkIa6(void)
{
    uint8_t offset;
    uint8_t i;
    uint16_t chksum, rxsum;
    chksum = ibusChecksum;
    rxsum = ibus[ibusFrameSize - 2] + (ibus[ibusFrameSize - 1] << 8);
    for (i = 0, offset = ibusChannelOffset; i < IBUS_MAX_CHANNEL; i++, offset += 2) {
        chksum += ibus[offset] + (ibus[offset + 1] << 8);
    }
    return chksum == rxsum;
}

bool ibusIsChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length)
{
    uint16_t calculatedChecksum = ibusCalculateChecksum(ibusPacket, length);

    // Note that there's a byte order swap to little endian here
    return (calculatedChecksum >> 8) == ibusPacket[length - 1]
           && (calculatedChecksum & 0xFF) == ibusPacket[length - 2];
}

static bool checksumIsOk(void) {
    if (ibusModel == IBUS_MODEL_IA6 ) {
        return isChecksumOkIa6();
    } else {
        return ibusIsChecksumOkIa6b(ibus, ibusFrameSize);
    }
}

static void updateChannelData(void) {
    uint8_t i;
    uint8_t offset;

    for (i = 0, offset = ibusChannelOffset; i < IBUS_MAX_CHANNEL; i++, offset += 2) {
        ibusChannelData[i] = ibus[offset] + (ibus[offset + 1] << 8);
    }
}

static uint8_t ibusFrameStatus(void)
{
    uint8_t frameStatus = RX_FRAME_PENDING;

    if (!ibusFrameDone) {
        return frameStatus;
    }

    ibusFrameDone = false;

    if (checksumIsOk()) {
        if (ibusModel == IBUS_MODEL_IA6 || ibusSyncByte == 0x20) {
            updateChannelData();
            frameStatus = RX_FRAME_COMPLETE;
        }
        else
        {
#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)
            rxBytesToIgnore = respondToIbusRequest(ibus);
#endif
        }
    }

    return frameStatus;
}

static uint16_t ibusReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return ibusChannelData[chan];
}


bool ibusInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);
    ibusSyncByte = 0;

    rxRuntimeConfig->channelCount = IBUS_MAX_CHANNEL;
    rxRuntimeConfig->rxRefreshRate = 20000; // TODO - Verify speed

    rxRuntimeConfig->rcReadRawFn = ibusReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = ibusFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef TELEMETRY
    bool portShared = isSerialPortShared(portConfig, FUNCTION_RX_SERIAL, FUNCTION_TELEMETRY_IBUS);
#else
    bool portShared = false;
#endif

    rxBytesToIgnore = 0;
    serialPort_t *ibusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        ibusDataReceive,
        IBUS_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        SERIAL_NOT_INVERTED | (rxConfig->halfDuplex || portShared ? SERIAL_BIDIR : 0)
        );

#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)
    if (portShared) {
        initSharedIbusTelemetry(ibusPort);
    }
#endif

    return ibusPort != NULL;
}

#endif // USE_SERIALRX_IBUS
