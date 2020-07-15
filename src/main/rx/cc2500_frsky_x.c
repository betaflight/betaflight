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

#include <string.h>

#include "platform.h"

#ifdef USE_RX_FRSKY_SPI_X

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/io.h"
#include "drivers/io_def.h"
#include "drivers/io_types.h"
#include "drivers/resource.h"
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

#include "telemetry/smartport.h"

#include "cc2500_frsky_x.h"

const uint16_t crcTable_Short[] = {
        0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
        0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
};

#define TELEMETRY_OUT_BUFFER_SIZE  64

#define TELEMETRY_SEQUENCE_LENGTH 4

#define A1_CONST_X 50

typedef struct telemetrySequenceMarkerData_s {
    unsigned int packetSequenceId: 2;
    unsigned int unused: 1;
    unsigned int initRequest: 1;
    unsigned int ackSequenceId: 2;
    unsigned int retransmissionRequested: 1;
    unsigned int initResponse: 1;
} __attribute__ ((__packed__)) telemetrySequenceMarkerData_t;

typedef union telemetrySequenceMarker_s {
    uint8_t raw;
    telemetrySequenceMarkerData_t data;
} __attribute__ ((__packed__)) telemetrySequenceMarker_t;

#define SEQUENCE_MARKER_REMOTE_PART 0xf0

#define TELEMETRY_DATA_SIZE 5

typedef struct telemetryData_s {
    uint8_t dataLength;
    uint8_t data[TELEMETRY_DATA_SIZE];
} __attribute__ ((__packed__)) telemetryData_t;

typedef struct telemetryBuffer_s {
    telemetryData_t data;
    uint8_t needsProcessing;
} telemetryBuffer_t;

#define TELEMETRY_FRAME_SIZE  sizeof(telemetryData_t)

typedef struct telemetryPayload_s {
    uint8_t packetConst;
    uint8_t rssiA1;
    telemetrySequenceMarker_t sequence;
    telemetryData_t data;
    uint8_t crc[2];
} __attribute__ ((__packed__)) telemetryPayload_t;

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
static telemetryData_t telemetryTxBuffer[TELEMETRY_SEQUENCE_LENGTH];
#endif

static telemetryBuffer_t telemetryRxBuffer[TELEMETRY_SEQUENCE_LENGTH];

static telemetrySequenceMarker_t responseToSend;

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
static uint8_t frame[20];

#if defined(USE_TELEMETRY_SMARTPORT)
static uint8_t telemetryOutWriter;

static uint8_t telemetryOutBuffer[TELEMETRY_OUT_BUFFER_SIZE];

static bool telemetryEnabled = false;

static uint8_t remoteToProcessId = 0;
static uint8_t remoteToProcessIndex = 0;
#endif
#endif // USE_RX_FRSKY_SPI_TELEMETRY

static uint8_t packetLength;
static uint16_t telemetryDelayUs;

static uint16_t crcTable(uint8_t val) {
    uint16_t word;
    word = (*(&crcTable_Short[val & 0x0f]));
    val /= 16;
    return word ^ (0x1081 * val);
}

static uint16_t calculateCrc(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0;
    for (unsigned i = 0; i < len; i++) {
        crc = (crc << 8) ^ crcTable((uint8_t)(crc >> 8) ^ *data++);
    }
    return crc;
}

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
#if defined(USE_TELEMETRY_SMARTPORT)
static uint8_t appendSmartPortData(uint8_t *buf)
{
    static uint8_t telemetryOutReader = 0;

    uint8_t index;
    for (index = 0; index < TELEMETRY_DATA_SIZE; index++) { // max 5 bytes in a frame
        if (telemetryOutReader == telemetryOutWriter) { // no new data
            break;
        }
        buf[index] = telemetryOutBuffer[telemetryOutReader];
        telemetryOutReader = (telemetryOutReader + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    }

    return index;
}
#endif

static void buildTelemetryFrame(uint8_t *packet)
{
    static uint8_t localPacketId;

    static bool evenRun = false;

    frame[0] = 0x0E;//length
    frame[1] = rxCc2500SpiConfig()->bindTxId[0];
    frame[2] = rxCc2500SpiConfig()->bindTxId[1];
    frame[3] = rxCc2500SpiConfig()->bindTxId[2];

    if (evenRun) {
        frame[4] = (uint8_t)cc2500getRssiDbm() | 0x80;
    } else {
        uint8_t a1Value;
        switch (rxCc2500SpiConfig()->a1Source) {
        case FRSKY_SPI_A1_SOURCE_EXTADC:
            a1Value = (uint8_t)((adcGetChannel(ADC_EXTERNAL1) & 0xfe0) >> 5);
            break;
        case FRSKY_SPI_A1_SOURCE_CONST:
            a1Value = A1_CONST_X & 0x7f;
            break;
        case FRSKY_SPI_A1_SOURCE_VBAT:
        default:
            a1Value = getLegacyBatteryVoltage() & 0x7f;
            break;
        }
        frame[4] = a1Value;
    }
    evenRun = !evenRun;

    telemetrySequenceMarker_t *inFrameMarker = (telemetrySequenceMarker_t *)&packet[21];
    telemetrySequenceMarker_t *outFrameMarker = (telemetrySequenceMarker_t *)&frame[5];
    if (inFrameMarker->data.initRequest) { // check syncronization at startup ok if not no sport telemetry
        outFrameMarker-> raw = 0;
        outFrameMarker->data.initRequest = 1;
        outFrameMarker->data.initResponse = 1;

        localPacketId = 0;
    } else {
        if (inFrameMarker->data.retransmissionRequested) {
            uint8_t retransmissionFrameId = inFrameMarker->data.ackSequenceId;
            outFrameMarker->raw = responseToSend.raw & SEQUENCE_MARKER_REMOTE_PART;
            outFrameMarker->data.packetSequenceId = retransmissionFrameId;

            memcpy(&frame[6], &telemetryTxBuffer[retransmissionFrameId], TELEMETRY_FRAME_SIZE);
        } else {
            uint8_t localAckId = inFrameMarker->data.ackSequenceId;
            if (localPacketId != (localAckId + 1) % TELEMETRY_SEQUENCE_LENGTH) {
                outFrameMarker->raw = responseToSend.raw & SEQUENCE_MARKER_REMOTE_PART;
                outFrameMarker->data.packetSequenceId = localPacketId;

                frame[6] = appendSmartPortData(&frame[7]);
                memcpy(&telemetryTxBuffer[localPacketId], &frame[6], TELEMETRY_FRAME_SIZE);

                localPacketId = (localPacketId + 1) % TELEMETRY_SEQUENCE_LENGTH;
            }
        }
    }

    uint16_t lcrc = calculateCrc(&frame[3], 10);
    frame[13]=lcrc>>8;
    frame[14]=lcrc;
}

static bool frSkyXReadyToSend(void)
{
    return true;
}

#if defined(USE_TELEMETRY_SMARTPORT)
static void frSkyXTelemetrySendByte(uint8_t c) {
    if (c == FSSP_DLE || c == FSSP_START_STOP) {
        telemetryOutBuffer[telemetryOutWriter] = FSSP_DLE;
        telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
        telemetryOutBuffer[telemetryOutWriter] = c ^ FSSP_DLE_XOR;
        telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    } else {
        telemetryOutBuffer[telemetryOutWriter] = c;
        telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    }
}

static void frSkyXTelemetryWriteFrame(const smartPortPayload_t *payload)
{
    telemetryOutBuffer[telemetryOutWriter] = FSSP_START_STOP;
    telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    telemetryOutBuffer[telemetryOutWriter] = FSSP_SENSOR_ID1 & 0x1f;
    telemetryOutWriter = (telemetryOutWriter + 1) % TELEMETRY_OUT_BUFFER_SIZE;
    uint8_t *data = (uint8_t *)payload;
    for (unsigned i = 0; i < sizeof(smartPortPayload_t); i++) {
        frSkyXTelemetrySendByte(*data++);
    }
}
#endif
#endif // USE_RX_FRSKY_SPI_TELEMETRY


void frSkyXSetRcData(uint16_t *rcData, const uint8_t *packet)
{
    uint16_t c[8];
    // ignore failsafe packet
    if (packet[7] != 0) {
        return;
    }
    c[0] = (uint16_t)((packet[10] << 8) & 0xF00) | packet[9];
    c[1] = (uint16_t)((packet[11] << 4) & 0xFF0) | (packet[10] >> 4);
    c[2] = (uint16_t)((packet[13] << 8) & 0xF00) | packet[12];
    c[3] = (uint16_t)((packet[14] << 4) & 0xFF0) | (packet[13] >> 4);
    c[4] = (uint16_t)((packet[16] << 8) & 0xF00) | packet[15];
    c[5] = (uint16_t)((packet[17] << 4) & 0xFF0) | (packet[16] >> 4);
    c[6] = (uint16_t)((packet[19] << 8) & 0xF00) | packet[18];
    c[7] = (uint16_t)((packet[20] << 4) & 0xFF0) | (packet[19] >> 4);

    for (unsigned i = 0; i < 8; i++) {
        const bool channelIsShifted = c[i] & 0x800;
        const uint16_t channelValue = c[i] & 0x7FF;
        rcData[channelIsShifted ? i + 8 : i] = ((channelValue - 64) * 2 + 860 * 3) / 3;
    }
}

bool isValidPacket(const uint8_t *packet)
{
    if (spiProtocol == RX_SPI_FRSKY_X_V2 || spiProtocol == RX_SPI_FRSKY_X_LBT_V2) {
        if (!(packet[packetLength - 1] & 0x80)) {
            return false;
        }
    }
    uint16_t lcrc = calculateCrc(&packet[3], (packetLength - 7));
    if ((lcrc >> 8) == packet[packetLength - 4] && (lcrc & 0x00FF) == packet[packetLength - 3] &&
        (packet[0] == packetLength - 3) &&
        (packet[1] == rxCc2500SpiConfig()->bindTxId[0]) &&
        (packet[2] == rxCc2500SpiConfig()->bindTxId[1]) &&
        (packet[3] == rxCc2500SpiConfig()->bindTxId[2]) &&
        (rxCc2500SpiConfig()->rxNum == 0 || packet[6] == 0 || packet[6] == rxCc2500SpiConfig()->rxNum)) {
        return true;
    }
    return false;
}

rx_spi_received_e frSkyXHandlePacket(uint8_t * const packet, uint8_t * const protocolState)
{
    static unsigned receiveTelemetryRetryCount = 0;
    static bool skipChannels = true;

    static uint8_t remoteAckId = 0;

    static timeUs_t packetTimerUs;

    static bool frameReceived;
    static timeDelta_t receiveDelayUs;
    static uint8_t channelsToSkip = 1;
    static uint32_t packetErrors = 0;

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
    static bool telemetryReceived = false;
#endif

    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    switch (*protocolState) {
    case STATE_STARTING:
        listLength = 47;
        initialiseData(false);
        *protocolState = STATE_UPDATE;
        nextChannel(1);
        cc2500Strobe(CC2500_SRX);
        break;
    case STATE_UPDATE:
        packetTimerUs = micros();
        *protocolState = STATE_DATA;
        frameReceived = false; // again set for receive
        receiveDelayUs = 5300;
        if (rxSpiCheckBindRequested(false)) {
            packetTimerUs = 0;
            timeoutUs = 50;
            missingPackets = 0;
            *protocolState = STATE_INIT;

            break;
        }

        FALLTHROUGH;
        // here FS code could be
    case STATE_DATA:
        if (rxSpiGetExtiState() && (!frameReceived)) {
            uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (ccLen >= packetLength) {
                cc2500ReadFifo(packet, packetLength);
                if (isValidPacket(packet)) {
                    missingPackets = 0;
                    timeoutUs = 1;
                    receiveDelayUs = 0;
                    rxSpiLedOn();
                    if (skipChannels) {
                        channelsToSkip = (packet[5] << 2) | (packet[4] >> 6);
                        telemetryReceived = true; // now telemetry can be sent
                        skipChannels = false;
                    }
                    cc2500setRssiDbm(packet[packetLength - 2]);

                    telemetrySequenceMarker_t *inFrameMarker = (telemetrySequenceMarker_t *)&packet[21];

                    uint8_t remoteNewPacketId = inFrameMarker->data.packetSequenceId;
                    memcpy(&telemetryRxBuffer[remoteNewPacketId].data, &packet[22], TELEMETRY_FRAME_SIZE);
                    telemetryRxBuffer[remoteNewPacketId].needsProcessing = true;

                    responseToSend.raw = 0;
                    uint8_t remoteToAckId = (remoteAckId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                    if (remoteNewPacketId != remoteToAckId) {
                        while (remoteToAckId != remoteNewPacketId) {
                            if (!telemetryRxBuffer[remoteToAckId].needsProcessing) {
                                responseToSend.data.ackSequenceId = remoteToAckId;
                                responseToSend.data.retransmissionRequested = 1;

                                receiveTelemetryRetryCount++;

                                break;
                            }

                            remoteToAckId = (remoteToAckId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                        }
                    }

                    if (!responseToSend.data.retransmissionRequested) {
                        receiveTelemetryRetryCount = 0;

                        remoteToAckId = (remoteAckId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                        uint8_t remoteNextAckId = remoteToAckId;
                        while (telemetryRxBuffer[remoteToAckId].needsProcessing && remoteToAckId != remoteAckId) {
                            remoteNextAckId = remoteToAckId;
                            remoteToAckId = (remoteToAckId + 1) % TELEMETRY_SEQUENCE_LENGTH;
                        }
                        remoteAckId = remoteNextAckId;
                        responseToSend.data.ackSequenceId = remoteAckId;
                    }

                    if (receiveTelemetryRetryCount >= 5) {
#if defined(USE_RX_FRSKY_SPI_TELEMETRY) && defined(USE_TELEMETRY_SMARTPORT)
                         remoteToProcessId = 0;
                         remoteToProcessIndex = 0;
#endif
                         remoteAckId = TELEMETRY_SEQUENCE_LENGTH - 1;
                         for (unsigned i = 0; i < TELEMETRY_SEQUENCE_LENGTH; i++) {
                             telemetryRxBuffer[i].needsProcessing = false;
                         }

                         receiveTelemetryRetryCount = 0;
                     }

                    packetTimerUs = micros();
                    frameReceived = true; // no need to process frame again.
                }
                if (!frameReceived) {
                    packetErrors++;
                    DEBUG_SET(DEBUG_RX_FRSKY_SPI, DEBUG_DATA_BAD_FRAME, packetErrors);
                    cc2500Strobe(CC2500_SFRX);
                }
            }
        }
        if (telemetryReceived) {
            if (cmpTimeUs(micros(), packetTimerUs) > receiveDelayUs) { // if received or not received in this time sent telemetry data
                *protocolState = STATE_TELEMETRY;
                buildTelemetryFrame(packet);
            }
        }
        if (cmpTimeUs(micros(), packetTimerUs) > timeoutUs * SYNC_DELAY_MAX) {
            rxSpiLedToggle();

            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
            nextChannel(1);
            cc2500Strobe(CC2500_SRX);
            *protocolState = STATE_UPDATE;
        }
        if (frameReceived) {
            ret |= RX_SPI_RECEIVED_DATA;

            frameReceived = false;
        }

        break;
#ifdef USE_RX_FRSKY_SPI_TELEMETRY
    case STATE_TELEMETRY:
        if (cmpTimeUs(micros(), packetTimerUs) >= receiveDelayUs + telemetryDelayUs) { // if received or not received in this time sent telemetry data
            cc2500Strobe(CC2500_SIDLE);
            cc2500SetPower(6);
            cc2500Strobe(CC2500_SFRX);
            delayMicroseconds(30);
#if defined(USE_RX_CC2500_SPI_PA_LNA)
            cc2500TxEnable();
#endif
            cc2500Strobe(CC2500_SIDLE);
            cc2500WriteFifo(frame, frame[0] + 1);

#if defined(USE_TELEMETRY_SMARTPORT)
            if (telemetryEnabled) {
                ret |= RX_SPI_ROCESSING_REQUIRED;
            }
#endif
            *protocolState = STATE_RESUME;
        }

        break;
#endif // USE_RX_FRSKY_SPI_TELEMETRY
    case STATE_RESUME:
        if (cmpTimeUs(micros(), packetTimerUs) > receiveDelayUs + 3700) {
            packetTimerUs = micros();
            receiveDelayUs = 5300;
            frameReceived = false; // again set for receive
            nextChannel(channelsToSkip);
            cc2500Strobe(CC2500_SRX);
#ifdef USE_RX_CC2500_SPI_PA_LNA
            cc2500TxDisable();
#if defined(USE_RX_CC2500_SPI_DIVERSITY)
            if (missingPackets >= 2) {
                cc2500switchAntennae();
            }
#endif
#endif // USE_RX_CC2500_SPI_PA_LNA
            if (missingPackets > MAX_MISSING_PKT) {
                timeoutUs = 50;
                skipChannels = true;
                telemetryReceived = false;
                *protocolState = STATE_UPDATE;
                break;
            }
            missingPackets++;
            DEBUG_SET(DEBUG_RX_FRSKY_SPI, DEBUG_DATA_MISSING_PACKETS, missingPackets);
            *protocolState = STATE_DATA;
        }
        break;
    }

    return ret;
}

#if defined(USE_RX_FRSKY_SPI_TELEMETRY) && defined(USE_TELEMETRY_SMARTPORT)
rx_spi_received_e frSkyXProcessFrame(uint8_t * const packet)
{
    static timeMs_t pollingTimeMs = 0;

    UNUSED(packet);

    bool clearToSend = false;
    timeMs_t now = millis();
    smartPortPayload_t *payload = NULL;
    if ((now - pollingTimeMs) > 24) {
        pollingTimeMs = now;

        clearToSend = true;
    } else {
        while (telemetryRxBuffer[remoteToProcessId].needsProcessing && !payload) {
            if (remoteToProcessIndex >= telemetryRxBuffer[remoteToProcessId].data.dataLength) {
                remoteToProcessIndex = 0;
                telemetryRxBuffer[remoteToProcessId].needsProcessing = false;
                remoteToProcessId = (remoteToProcessId + 1) % TELEMETRY_SEQUENCE_LENGTH;

                if (!telemetryRxBuffer[remoteToProcessId].needsProcessing) {
                    break;
                }
            }

            while (remoteToProcessIndex < telemetryRxBuffer[remoteToProcessId].data.dataLength && !payload) {
                payload = smartPortDataReceive(telemetryRxBuffer[remoteToProcessId].data.data[remoteToProcessIndex], &clearToSend, frSkyXReadyToSend, false);
                remoteToProcessIndex = remoteToProcessIndex + 1;
            }
        }
    }

    processSmartPortTelemetry(payload, &clearToSend, NULL);

    return RX_SPI_RECEIVED_NONE;
}
#endif

uint8_t frSkyXInit(void)
{
    switch(spiProtocol) {
    case RX_SPI_FRSKY_X:
        packetLength = FRSKY_RX_D16FCC_LENGTH;
        telemetryDelayUs = 400;
        break;
    case RX_SPI_FRSKY_X_LBT:
        packetLength = FRSKY_RX_D16LBT_LENGTH;
        telemetryDelayUs = 1400;
        break;
    case RX_SPI_FRSKY_X_V2:
        packetLength = FRSKY_RX_D16v2_LENGTH;
        telemetryDelayUs = 400;
        break;
    case RX_SPI_FRSKY_X_LBT_V2:
        packetLength = FRSKY_RX_D16v2_LENGTH;
        telemetryDelayUs = 1500;
        break;
    default:
        break;
    }
#if defined(USE_TELEMETRY_SMARTPORT)
     if (featureIsEnabled(FEATURE_TELEMETRY)) {
         telemetryEnabled = initSmartPortTelemetryExternal(frSkyXTelemetryWriteFrame);
     }
#endif
     return packetLength;
}

#endif
