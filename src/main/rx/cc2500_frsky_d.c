/*
 * This file is part of Cleanflight and Betaflight.
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

#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/io.h"
#include "drivers/rx/rx_cc2500.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "config/config.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"

#include "rx/rx_spi_common.h"
#include "rx/cc2500_common.h"
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

#define A1_CONST_D 100

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
    uint8_t a1Value;
    switch (rxCc2500SpiConfig()->a1Source) {
    case FRSKY_SPI_A1_SOURCE_EXTADC:
        a1Value = (adcGetChannel(ADC_EXTERNAL1) & 0xff0) >> 4;
        break;
#if defined(USE_TELEMETRY_FRSKY_HUB)
    case FRSKY_SPI_A1_SOURCE_CONST:
        a1Value = A1_CONST_D & 0xff;
        break;
#endif
    case FRSKY_SPI_A1_SOURCE_VBAT:
    default:
        a1Value = (getBatteryVoltage() / 5) & 0xff;
        break;
    }
    const uint8_t a2Value = (adcGetChannel(ADC_RSSI)) >> 4;
    telemetryId = packet[4];
    frame[0] = 0x11; // length
    frame[1] = rxCc2500SpiConfig()->bindTxId[0];
    frame[2] = rxCc2500SpiConfig()->bindTxId[1];
    frame[3] = a1Value;
    frame[4] = a2Value;
    frame[5] = (uint8_t)cc2500getRssiDbm();
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

static void decodeChannelPair(uint16_t *channels, const uint8_t *packet, const uint8_t highNibbleOffset)
{
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

    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    const timeUs_t currentPacketReceivedTime = micros();

    switch (*protocolState) {
    case STATE_STARTING:
        listLength = 47;
        initialiseData(false);
        *protocolState = STATE_UPDATE;
        nextChannel(1);
        cc2500Strobe(CC2500_SRX);

        break;
    case STATE_UPDATE:
        lastPacketReceivedTime = currentPacketReceivedTime;
        *protocolState = STATE_DATA;

        if (rxSpiCheckBindRequested(false)) {
            lastPacketReceivedTime = 0;
            timeoutUs = 50;
            missingPackets = 0;

            *protocolState = STATE_INIT;

            break;
        }
        FALLTHROUGH; //!!TODO -check this fall through is correct
    // here FS code could be
    case STATE_DATA:
        if (rxSpiGetExtiState()) {
            uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            bool packetOk = false;
            if (ccLen >= 20) {
                cc2500ReadFifo(packet, 20);
                if (packet[19] & 0x80) {
                    packetOk = true;
                    missingPackets = 0;
                    timeoutUs = 1;
                    if (packet[0] == 0x11) {
                        if ((packet[1] == rxCc2500SpiConfig()->bindTxId[0]) &&
                            (packet[2] == rxCc2500SpiConfig()->bindTxId[1]) &&
                            (packet[5] == rxCc2500SpiConfig()->bindTxId[2])) {
                            rxSpiLedOn();
                            nextChannel(1);
                            cc2500setRssiDbm(packet[18]);
#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
                            if ((packet[3] % 4) == 2) {
                                telemetryTimeUs = micros();
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
            if (!packetOk) {
                cc2500Strobe(CC2500_SFRX);
            }
        }

        if (cmpTimeUs(currentPacketReceivedTime, lastPacketReceivedTime) > (timeoutUs * SYNC_DELAY_MAX)) {
#if defined(USE_RX_CC2500_SPI_PA_LNA)
            cc2500TxDisable();
#endif
            if (timeoutUs == 1) {
#if defined(USE_RX_CC2500_SPI_PA_LNA) && defined(USE_RX_CC2500_SPI_DIVERSITY) // SE4311 chip
                if (missingPackets >= 2) {
                    cc2500switchAntennae();
                }
#endif

                if (missingPackets > MAX_MISSING_PKT) {
                    timeoutUs = 50;

                    setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
                }

                missingPackets++;
                nextChannel(1);
            } else {
                rxSpiLedToggle();

                setRssi(0, RSSI_SOURCE_RX_PROTOCOL);
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
#if defined(USE_RX_CC2500_SPI_PA_LNA)
            cc2500TxEnable();
#endif
            cc2500Strobe(CC2500_SIDLE);
            cc2500WriteFifo(frame, frame[0] + 1);
            *protocolState = STATE_DATA;
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
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        telemetryEnabled = initFrSkyHubTelemetryExternal(frSkyDTelemetryWriteByte);
    }
#endif
}
#endif
