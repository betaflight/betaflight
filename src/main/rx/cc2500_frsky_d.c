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
#include "drivers/cc2500.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "config/feature.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_frsky_shared.h"
#include "rx/cc2500_frsky_d.h"

#include "sensors/battery.h"

#include "telemetry/frsky.h"

#define RC_CHANNEL_COUNT 8
#define MAX_MISSING_PKT 100

#define DEBUG_DATA_ERROR_COUNT 0

#define SYNC 9000
#define FS_THR 960

static uint32_t missingPackets;
static uint8_t cnt;
static int32_t t_out;
static timeUs_t lastPacketReceivedTime;
static uint8_t protocolState;
static uint32_t start_time;
static uint16_t dataErrorCount = 0;

#if defined(USE_RX_FRSKY_SPI_PA_LNA)
static uint8_t pass;
#endif

#ifdef USE_RX_FRSKY_SPI_TELEMETRY
static uint8_t frame[20];
static uint8_t telemetry_id;
static uint32_t telemetryTime;

#if defined(USE_TELEMETRY_FRSKY)
#define MAX_SERIAL_BYTES 64
static uint8_t hub_index;
static uint8_t idxx = 0;
static uint8_t idx_ok = 0;
static uint8_t telemetry_expected_id = 0;
static uint8_t srx_data[MAX_SERIAL_BYTES]; // buffer for telemetry serial data
#endif
#endif

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
#if defined(USE_TELEMETRY_FRSKY)
static uint8_t frsky_append_hub_data(uint8_t *buf)
{
    if (telemetry_id == telemetry_expected_id)
        idx_ok = idxx;
    else // rx re-requests last packet
        idxx = idx_ok;

    telemetry_expected_id = (telemetry_id + 1) & 0x1F;
    uint8_t index = 0;
    for (uint8_t i = 0; i < 10; i++) {
        if (idxx == hub_index) {
            break;
        }
        buf[i] = srx_data[idxx];
        idxx = (idxx + 1) & (MAX_SERIAL_BYTES - 1);
        index++;
    }
    return index;
}

static void frSkyTelemetryInitFrameSpi(void)
{
    hub_index = 0;
    idxx = 0;
}

static void frSkyTelemetryWriteSpi(uint8_t ch)
{
    if (hub_index < MAX_SERIAL_BYTES) {
        srx_data[hub_index++] = ch;
    }
}
#endif

static void telemetry_build_frame(uint8_t *packet)
{
    const uint16_t adcExternal1Sample = adcGetChannel(ADC_EXTERNAL1);
    const uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
    uint8_t bytes_used = 0;
    telemetry_id = packet[4];
    frame[0] = 0x11; // length
    frame[1] = rxFrSkySpiConfig()->bindTxId[0];
    frame[2] = rxFrSkySpiConfig()->bindTxId[1];
    frame[3] = (uint8_t)((adcExternal1Sample & 0xff0) >> 4); // A1
    frame[4] = (uint8_t)((adcRssiSample & 0xff0) >> 4);      // A2
    frame[5] = (uint8_t)RSSI_dBm;
#if defined(USE_TELEMETRY_FRSKY)
    bytes_used = frsky_append_hub_data(&frame[8]);
#endif
    frame[6] = bytes_used;
    frame[7] = telemetry_id;
}

#endif // USE_RX_FRSKY_SPI_TELEMETRY

#define FRSKY_D_CHANNEL_SCALING (2.0f / 3)

static void decodeChannelPair(uint16_t *channels, const uint8_t *packet, const uint8_t highNibbleOffset) {
    channels[0] = FRSKY_D_CHANNEL_SCALING * (uint16_t)((packet[highNibbleOffset] & 0xf) << 8 | packet[0]);
    channels[1] = FRSKY_D_CHANNEL_SCALING * (uint16_t)((packet[highNibbleOffset] & 0xf0) << 4 | packet[1]);
}

void frSkyDSetRcData(uint16_t *rcData, const uint8_t *packet)
{
    uint16_t channels[RC_CHANNEL_COUNT];
    bool dataError = false;

    decodeChannelPair(channels, packet + 6, 4);
    decodeChannelPair(channels + 2, packet + 8, 3);
    decodeChannelPair(channels + 4, packet + 12, 4);
    decodeChannelPair(channels + 6, packet + 14, 3);

    for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
        if ((channels[i] < 800) || (channels[i] > 2200)) {
            dataError = true;

            break;
        }
    }

    if (!dataError) {
        for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
            rcData[i] = channels[i];
        }
    } else {
        DEBUG_SET(DEBUG_RX_FRSKY_SPI, DEBUG_DATA_ERROR_COUNT, ++dataErrorCount);
    }
}

rx_spi_received_e frSkyDDataReceived(uint8_t *packet)
{
    const timeUs_t currentPacketReceivedTime = micros();

    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    switch (protocolState) {
    case STATE_INIT:
        if ((millis() - start_time) > 10) {
            initialize();

            protocolState = STATE_BIND;
        }

        break;
    case STATE_BIND:
    case STATE_BIND_TUNING:
    case STATE_BIND_BINDING1:
    case STATE_BIND_BINDING2:
    case STATE_BIND_COMPLETE:
        protocolState = handleBinding(protocolState, packet);

        break;
    case STATE_STARTING:
        listLength = 47;
        initialiseData(0);
        protocolState = STATE_UPDATE;
        nextChannel(1);
        cc2500Strobe(CC2500_SRX);
        ret = RX_SPI_RECEIVED_BIND;

        break;
    case STATE_UPDATE:
        lastPacketReceivedTime = currentPacketReceivedTime;
        protocolState = STATE_DATA;

        if (checkBindRequested(false)) {
            lastPacketReceivedTime = 0;
            t_out = 50;
            missingPackets = 0;

            protocolState = STATE_INIT;

            break;
        }
    // here FS code could be
    case STATE_DATA:
        if (IORead(gdoPin)) {
            uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (ccLen >= 20) {
                cc2500ReadFifo(packet, 20);
                if (packet[19] & 0x80) {
                    missingPackets = 0;
                    t_out = 1;
                    if (packet[0] == 0x11) {
                        if ((packet[1] == rxFrSkySpiConfig()->bindTxId[0]) &&
                            (packet[2] == rxFrSkySpiConfig()->bindTxId[1])) {
                            IOHi(frSkyLedPin);
                            nextChannel(1);
#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
                            if ((packet[3] % 4) == 2) {
                                telemetryTime = micros();
                                setRssiDbm(packet[18]);
                                telemetry_build_frame(packet);
                                protocolState = STATE_TELEMETRY;
                            } else
#endif
                            {
                                cc2500Strobe(CC2500_SRX);
                                protocolState = STATE_UPDATE;
                            }
                            ret = RX_SPI_RECEIVED_DATA;
                            lastPacketReceivedTime = currentPacketReceivedTime;
                        }
                    }
                }
            }
        }

        if (cmpTimeUs(currentPacketReceivedTime, lastPacketReceivedTime) > (t_out * SYNC)) {
#if defined(USE_RX_FRSKY_SPI_PA_LNA)
            RxEnable();
#endif
            if (t_out == 1) {
#if defined(USE_RX_FRSKY_SPI_PA_LNA) && defined(USE_RX_FRSKY_SPI_DIVERSITY) // SE4311 chip
                if (missingPackets >= 2) {
                    if (pass & 0x01) {
                        IOHi(antSelPin);
                    } else {
                        IOLo(antSelPin);
                    }
                    pass++;
                }
#endif

                if (missingPackets > MAX_MISSING_PKT) {
                    t_out = 50;

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
                    setRssiFiltered(0, RSSI_SOURCE_RX_PROTOCOL);
#endif
                }

                missingPackets++;
                nextChannel(1);
            } else {
                if (cnt++ & 0x01) {
                    IOLo(frSkyLedPin);
                } else {
                    IOHi(frSkyLedPin);
                }

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
                setRssiUnfiltered(0, RSSI_SOURCE_RX_PROTOCOL);
#endif
                nextChannel(13);
            }

            cc2500Strobe(CC2500_SRX);
            protocolState = STATE_UPDATE;
        }
        break;
#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
    case STATE_TELEMETRY:
        if ((micros() - telemetryTime) >= 1380) {
            cc2500Strobe(CC2500_SIDLE);
            cc2500SetPower(6);
            cc2500Strobe(CC2500_SFRX);
#if defined(USE_RX_FRSKY_SPI_PA_LNA)
            TxEnable();
#endif
            cc2500Strobe(CC2500_SIDLE);
            cc2500WriteFifo(frame, frame[0] + 1);
            protocolState = STATE_DATA;
            ret = RX_SPI_RECEIVED_DATA;
            lastPacketReceivedTime = currentPacketReceivedTime;
        }

        break;

#endif
    }
    return ret;
}

void frSkyDInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT;

    protocolState = STATE_INIT;
    lastPacketReceivedTime = 0;
    missingPackets = 0;
    t_out = 50;

#if defined(USE_RX_FRSKY_SPI_TELEMETRY) && defined(USE_TELEMETRY_FRSKY)
    initFrSkyExternalTelemetry(&frSkyTelemetryInitFrameSpi,
                               &frSkyTelemetryWriteSpi);
#endif

    frskySpiRxSetup(rxConfig->rx_spi_protocol);

    start_time = millis();
}
#endif
