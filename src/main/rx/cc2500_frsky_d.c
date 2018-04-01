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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_FRSKY_SPI_D

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/adc.h"
#include "drivers/rx/rx_cc2500.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_frsky_shared.h"

#include "sensors/battery.h"

#include "telemetry/frsky_hub.h"

#include "cc2500_frsky_d.h"

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
static uint8_t frame[20];
static uint8_t telemetryId;

#if defined(USE_TELEMETRY_FRSKY_HUB)
static bool telemetryEnabled = false;

#define MAX_SERIAL_BYTES 64

static uint8_t telemetryBytesGenerated;
static uint8_t serialBuffer[MAX_SERIAL_BYTES]; // buffer for telemetry serial data

static uint8_t appendFrSkyHubData(uint8_t *buf)
{
    static uint8_t telemetryBytesSent = 0;
    static uint8_t telemetryBytesAcknowledged = 0;
    static uint8_t telemetryIdExpected = 0;

    if (telemetryId == telemetryIdExpected) {
        telemetryBytesAcknowledged = telemetryBytesSent;

        telemetryIdExpected = (telemetryId + 1) & 0x1F;
        if (!telemetryBytesGenerated) {
            telemetryBytesSent = 0;

            processFrSkyHubTelemetry(micros());
        }
    } else { // rx re-requests last packet
        telemetryBytesSent = telemetryBytesAcknowledged;
    }

    uint8_t index = 0;
    for (uint8_t i = 0; i < 10; i++) {
        if (telemetryBytesSent == telemetryBytesGenerated) {
            telemetryBytesGenerated = 0;

            break;
        }
        buf[i] = serialBuffer[telemetryBytesSent];
        telemetryBytesSent = (telemetryBytesSent + 1) & (MAX_SERIAL_BYTES - 1);
        index++;
    }
    return index;
}

static void frSkyDTelemetryWriteByte(const char data)
{
    if (telemetryBytesGenerated < MAX_SERIAL_BYTES) {
        serialBuffer[telemetryBytesGenerated++] = data;
    }
}
#endif

static void buildTelemetryFrame(uint8_t *packet)
{
    const uint16_t adcExternal1Sample = adcGetChannel(ADC_EXTERNAL1);
    const uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
    telemetryId = packet[4];
    frame[0] = 0x11; // length
    frame[1] = rxFrSkySpiConfig()->bindTxId[0];
    frame[2] = rxFrSkySpiConfig()->bindTxId[1];
    frame[3] = (uint8_t)((adcExternal1Sample & 0xff0) >> 4); // A1
    frame[4] = (uint8_t)((adcRssiSample & 0xff0) >> 4);      // A2
    frame[5] = (uint8_t)rssiDbm;
    uint8_t bytesUsed = 0;
#if defined(USE_TELEMETRY_FRSKY_HUB)
    if (telemetryEnabled) {
        bytesUsed = appendFrSkyHubData(&frame[8]);
    }
 #endif
    frame[6] = bytesUsed;
    frame[7] = telemetryId;
}
#endif // USE_RX_FRSKY_SPI_TELEMETRY

#define FRSKY_D_CHANNEL_SCALING (2.0f / 3)

static void decodeChannelPair(uint16_t *channels, const uint8_t *packet, const uint8_t highNibbleOffset) {
    channels[0] = FRSKY_D_CHANNEL_SCALING * (uint16_t)((packet[highNibbleOffset] & 0xf) << 8 | packet[0]);
    channels[1] = FRSKY_D_CHANNEL_SCALING * (uint16_t)((packet[highNibbleOffset] & 0xf0) << 4 | packet[1]);
}

void frSkyDSetRcData(uint16_t *rcData, const uint8_t *packet)
{
    static uint16_t dataErrorCount = 0;

    uint16_t channels[RC_CHANNEL_COUNT_FRSKY_D];
    bool dataError = false;

    decodeChannelPair(channels, packet + 6, 4);
    decodeChannelPair(channels + 2, packet + 8, 3);
    decodeChannelPair(channels + 4, packet + 12, 4);
    decodeChannelPair(channels + 6, packet + 14, 3);

    for (int i = 0; i < RC_CHANNEL_COUNT_FRSKY_D; i++) {
        if ((channels[i] < 800) || (channels[i] > 2200)) {
            dataError = true;

            break;
        }
    }

    if (!dataError) {
        for (int i = 0; i < RC_CHANNEL_COUNT_FRSKY_D; i++) {
            rcData[i] = channels[i];
        }
    } else {
        DEBUG_SET(DEBUG_RX_FRSKY_SPI, DEBUG_DATA_ERROR_COUNT, ++dataErrorCount);
    }
}

rx_spi_received_e frSkyDHandlePacket(uint8_t * const packet, uint8_t * const protocolState)
{
    static timeUs_t lastPacketReceivedTime = 0;
    static timeUs_t telemetryTimeUs;

    static bool ledIsOn;

    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    const timeUs_t currentPacketReceivedTime = micros();

    switch (*protocolState) {
    case STATE_STARTING:
        listLength = 47;
        initialiseData(0);
        *protocolState = STATE_UPDATE;
        nextChannel(1);
        cc2500Strobe(CC2500_SRX);

        break;
    case STATE_UPDATE:
        lastPacketReceivedTime = currentPacketReceivedTime;
        *protocolState = STATE_DATA;

        if (checkBindRequested(false)) {
            lastPacketReceivedTime = 0;
            timeoutUs = 50;
            missingPackets = 0;

            *protocolState = STATE_INIT;

            break;
        }
        FALLTHROUGH; //!!TODO -check this fall through is correct
    // here FS code could be
    case STATE_DATA:
        if (IORead(gdoPin)) {
            uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (ccLen >= 20) {
                cc2500ReadFifo(packet, 20);
                if (packet[19] & 0x80) {
                    missingPackets = 0;
                    timeoutUs = 1;
                    if (packet[0] == 0x11) {
                        if ((packet[1] == rxFrSkySpiConfig()->bindTxId[0]) &&
                            (packet[2] == rxFrSkySpiConfig()->bindTxId[1])) {
                            LedOn();
                            nextChannel(1);
#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
                            if ((packet[3] % 4) == 2) {
                                telemetryTimeUs = micros();
                                setRssiDbm(packet[18]);
                                buildTelemetryFrame(packet);
                                *protocolState = STATE_TELEMETRY;
                            } else
#endif
                            {
                                cc2500Strobe(CC2500_SRX);
                                *protocolState = STATE_UPDATE;
                            }
                            ret = RX_SPI_RECEIVED_DATA;
                            lastPacketReceivedTime = currentPacketReceivedTime;
                        }
                    }
                }
            }
        }

        if (cmpTimeUs(currentPacketReceivedTime, lastPacketReceivedTime) > (timeoutUs * SYNC_DELAY_MAX)) {
#if defined(USE_RX_FRSKY_SPI_PA_LNA)
            TxDisable();
#endif
            if (timeoutUs == 1) {
#if defined(USE_RX_FRSKY_SPI_PA_LNA) && defined(USE_RX_FRSKY_SPI_DIVERSITY) // SE4311 chip
                if (missingPackets >= 2) {
                    switchAntennae();
                }
#endif

                if (missingPackets > MAX_MISSING_PKT) {
                    timeoutUs = 50;

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
                    setRssiFiltered(0, RSSI_SOURCE_RX_PROTOCOL);
#endif
                }

                missingPackets++;
                nextChannel(1);
            } else {
                if (ledIsOn) {
                    LedOff();
                } else {
                    LedOn();
                }
                ledIsOn = !ledIsOn;

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
                setRssiUnfiltered(0, RSSI_SOURCE_RX_PROTOCOL);
#endif
                nextChannel(13);
            }

            cc2500Strobe(CC2500_SRX);
            *protocolState = STATE_UPDATE;
        }
        break;
#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
    case STATE_TELEMETRY:
        if (cmpTimeUs(micros(), telemetryTimeUs) >= 1380) {
            cc2500Strobe(CC2500_SIDLE);
            cc2500SetPower(6);
            cc2500Strobe(CC2500_SFRX);
#if defined(USE_RX_FRSKY_SPI_PA_LNA)
            TxEnable();
#endif
            cc2500Strobe(CC2500_SIDLE);
            cc2500WriteFifo(frame, frame[0] + 1);
            *protocolState = STATE_DATA;
            ret = RX_SPI_RECEIVED_DATA;
            lastPacketReceivedTime = currentPacketReceivedTime;
        }

        break;

#endif
    }

    return ret;
}

void frSkyDInit(void)
{
#if defined(USE_RX_FRSKY_SPI_TELEMETRY) && defined(USE_TELEMETRY_FRSKY_HUB)
    telemetryEnabled = initFrSkyHubTelemetryExternal(frSkyDTelemetryWriteByte);
#endif
}
#endif
