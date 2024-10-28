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

#ifdef USE_SERIALRX_GHST

#include "build/build_config.h"
#include "build/debug.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "pg/rx.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/ghst.h"

#include "telemetry/ghst.h"

#define GHST_PORT_OPTIONS               (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_BIDIR | SERIAL_BIDIR_PP)
#define GHST_PORT_MODE                  MODE_RXTX   // bidirectional on single pin

#define GHST_MAX_FRAME_TIME_US          500         // 14 bytes @ 420k = ~450us
#define GHST_TIME_BETWEEN_FRAMES_US     2000        // fastest frame rate = 500Hz, or 2000us

#define GHST_RSSI_DBM_MIN (-117)            // Long Range mode value
#define GHST_RSSI_DBM_MAX (-60)             // Typical RSSI with typical power levels, typical antennas, and a few feet/meters between Tx and Rx

// define the time window after the end of the last received packet where telemetry packets may be sent
// NOTE: This allows the Rx to double-up on Rx packets to transmit data other than servo data, but
// only if sent < 1ms after the servo data packet.
#define GHST_RX_TO_TELEMETRY_MIN_US     1000
#define GHST_RX_TO_TELEMETRY_MAX_US     2000

#define GHST_PAYLOAD_OFFSET offsetof(ghstFrameDef_t, type)

#define GHST_RC_CTR_VAL_12BIT_PRIMARY 2048
#define GHST_RC_CTR_VAL_12BIT_AUX     (128 << 2)

STATIC_UNIT_TESTED volatile bool ghstFrameAvailable = false;
STATIC_UNIT_TESTED volatile bool ghstValidatedFrameAvailable = false;
STATIC_UNIT_TESTED volatile bool ghstTransmittingTelemetry = false;

STATIC_UNIT_TESTED ghstFrame_t ghstFrameBuffer[2];

ghstFrame_t *ghstIncomingFrame  = &ghstFrameBuffer[0];  // incoming frame, raw, not CRC checked, destination address not checked
ghstFrame_t *ghstValidatedFrame = &ghstFrameBuffer[1];  // validated frame, CRC is ok, destination address is ok, ready for decode

STATIC_UNIT_TESTED uint16_t ghstChannelData[GHST_MAX_NUM_CHANNELS];
static ghstRfProtocol_e ghstRfProtocol = GHST_RF_PROTOCOL_UNDEFINED;

enum {
    DEBUG_GHST_CRC_ERRORS = 0,
    DEBUG_GHST_UNKNOWN_FRAMES,
    DEBUG_GHST_RX_RSSI,
    DEBUG_GHST_RX_LQ,
};

static serialPort_t *serialPort;
static timeUs_t ghstRxFrameStartAtUs = 0;
static timeUs_t ghstRxFrameEndAtUs = 0;
static uint8_t telemetryBuf[GHST_FRAME_SIZE];
static uint8_t telemetryBufLen = 0;

/* GHST Protocol
 * Ghost uses 420k baud single-wire, half duplex connection, connected to a FC UART 'Tx' pin
 * Each control packet is interleaved with one or more corresponding downlink packets
 *
 * Uplink packet format (Control packets)
 * <Addr><Len><Type><Payload><CRC>
 *
 * Addr:        u8      Destination address
 * Len          u8      Length includes the packet ID, but not the CRC
 * CRC          u8
 *
 * Ghost packets are designed to be as short as possible, for minimum latency.
 *
 * Note that the GHST protocol does not handle, itself, failsafe conditions. Packets are passed from
 * the Ghost receiver to Betaflight as and when they arrive. Betaflight itself is responsible for
 * determining when a failsafe is necessary based on dropped packets.
 *
 */


// called from telemetry/ghst.c
void ghstRxWriteTelemetryData(const void *const data, const int len)
{
    telemetryBufLen = MIN(len, (int)sizeof(telemetryBuf));
    memcpy(telemetryBuf, data, telemetryBufLen);
}

void ghstRxSendTelemetryData(void)
{
    // if there is telemetry data to write
    if (telemetryBufLen > 0) {
        if (serialPort != NULL) {
            serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);
        }
        telemetryBufLen = 0; // reset telemetry buffer
    }
}

uint8_t ghstRxGetTelemetryBufLen(void)
{
    return telemetryBufLen;
}

STATIC_UNIT_TESTED uint8_t ghstFrameCRC(const ghstFrame_t *const pGhstFrame)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, pGhstFrame->frame.type);
    for (int i = 0; i < pGhstFrame->frame.len - GHST_FRAME_LENGTH_TYPE - GHST_FRAME_LENGTH_CRC; ++i) {
        crc = crc8_dvb_s2(crc, pGhstFrame->frame.payload[i]);
    }
    return crc;
}

static void rxSwapFrameBuffers(void)
{
    ghstFrame_t *const tmp = ghstIncomingFrame;
    ghstIncomingFrame = ghstValidatedFrame;
    ghstValidatedFrame = tmp;
}

// Receive ISR callback, called back from serial port
STATIC_UNIT_TESTED void ghstDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    static uint8_t ghstFrameIdx = 0;
    const timeUs_t currentTimeUs = microsISR();

    if (cmpTimeUs(currentTimeUs, ghstRxFrameStartAtUs) > GHST_MAX_FRAME_TIME_US) {
        // Character received after the max. frame time, assume that this is a new frame
        ghstFrameIdx = 0;
    }

    if (ghstFrameIdx == 0) {
        // timestamp the start of the frame, to allow us to detect frame sync issues
        ghstRxFrameStartAtUs = currentTimeUs;
    }

    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    const int fullFrameLength = ghstFrameIdx < 3 ? 5 : ghstIncomingFrame->frame.len + GHST_FRAME_LENGTH_ADDRESS + GHST_FRAME_LENGTH_FRAMELENGTH;

    if (ghstFrameIdx < fullFrameLength && ghstFrameIdx < sizeof(ghstFrame_t)) {
        ghstIncomingFrame->bytes[ghstFrameIdx++] = (uint8_t)c;
        if (ghstFrameIdx >= fullFrameLength) {
            ghstFrameIdx = 0;

            // NOTE: this data is not yet CRC checked, nor do we know whether we are the correct recipient, this is
            // handled in ghstFrameStatus

            // Not CRC checked but we are interested just in frame for us
            // eg. telemetry frames are read back here also, skip them
            if (ghstIncomingFrame->frame.addr == GHST_ADDR_FC) {

                rxSwapFrameBuffers();
                ghstFrameAvailable = true;

                // remember what time the incoming (Rx) packet ended, so that we can ensure a quite bus before sending telemetry
                ghstRxFrameEndAtUs = microsISR();
            }
        }
    }
}

#ifdef USE_TELEMETRY_GHST
static bool shouldSendTelemetryFrame(void)
{
    const timeUs_t now = micros();
    const timeDelta_t timeSinceRxFrameEndUs = cmpTimeUs(now, ghstRxFrameEndAtUs);
    return telemetryBufLen > 0 && timeSinceRxFrameEndUs > GHST_RX_TO_TELEMETRY_MIN_US && timeSinceRxFrameEndUs < GHST_RX_TO_TELEMETRY_MAX_US;
}
#endif

STATIC_UNIT_TESTED uint8_t ghstFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    static int16_t crcErrorCount = 0;
    uint8_t status = RX_FRAME_PENDING;

    if (ghstFrameAvailable) {
        ghstFrameAvailable = false;

        const uint8_t crc = ghstFrameCRC(ghstValidatedFrame);
        const int fullFrameLength = ghstValidatedFrame->frame.len + GHST_FRAME_LENGTH_ADDRESS + GHST_FRAME_LENGTH_FRAMELENGTH;
        if (crc == ghstValidatedFrame->bytes[fullFrameLength - 1]) {
            ghstValidatedFrameAvailable = true;
            rxRuntimeState->lastRcFrameTimeUs = ghstRxFrameEndAtUs;
            status = RX_FRAME_COMPLETE | RX_FRAME_PROCESSING_REQUIRED;      // request callback through ghstProcessFrame to do the decoding work
        } else {
            DEBUG_SET(DEBUG_GHST, DEBUG_GHST_CRC_ERRORS, ++crcErrorCount);
            status = RX_FRAME_DROPPED;                            // frame was invalid
        }
    } else {
#ifdef USE_TELEMETRY_GHST
        if (checkGhstTelemetryState() && shouldSendTelemetryFrame()) {
            status = RX_FRAME_PROCESSING_REQUIRED;
        }
#endif
    }

    return status;
}

static bool ghstProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    // Assume that the only way we get here is if ghstFrameStatus returned RX_FRAME_PROCESSING_REQUIRED, which indicates that the CRC
    // is correct, and the message was actually for us.

    UNUSED(rxRuntimeState);
    static int16_t unknownFrameCount = 0;

#ifdef USE_TELEMETRY_GHST
    // do we have a telemetry buffer to send?
    if (checkGhstTelemetryState() && shouldSendTelemetryFrame()) {
        ghstTransmittingTelemetry = true;
        ghstRxSendTelemetryData();
    }
#endif

    if (ghstValidatedFrameAvailable) {
        ghstValidatedFrameAvailable = false;

        const uint8_t ghstFrameType = ghstValidatedFrame->frame.type;
        const bool scalingLegacy = ghstFrameType >= GHST_UL_RC_CHANS_HS4_FIRST && ghstFrameType <= GHST_UL_RC_CHANS_HS4_LAST;
        const bool scaling12bit = ghstFrameType >= GHST_UL_RC_CHANS_HS4_12_FIRST && ghstFrameType <= GHST_UL_RC_CHANS_HS4_12_LAST;

        if ( scaling12bit || scalingLegacy ) {

            int startIdx = 0;

            switch (ghstFrameType) {
                case GHST_UL_RC_CHANS_HS4_RSSI:
                case GHST_UL_RC_CHANS_HS4_12_RSSI: {
                    const ghstPayloadPulsesRssi_t* const rssiFrame = (ghstPayloadPulsesRssi_t*)&ghstValidatedFrame->frame.payload;

                    DEBUG_SET(DEBUG_GHST, DEBUG_GHST_RX_RSSI, -rssiFrame->rssi);
                    DEBUG_SET(DEBUG_GHST, DEBUG_GHST_RX_LQ, rssiFrame->lq);

                    ghstRfProtocol = rssiFrame->rfProtocol;

#ifdef USE_TELEMETRY_GHST
                    // Enable telemetry just for modes that support it
                    setGhstTelemetryState(ghstRfProtocol == GHST_RF_PROTOCOL_NORMAL
                                       || ghstRfProtocol == GHST_RF_PROTOCOL_RACE
                                       || ghstRfProtocol == GHST_RF_PROTOCOL_LONGRANGE
                                       || ghstRfProtocol == GHST_RF_PROTOCOL_RACE250);
#endif

                    if (rssiSource == RSSI_SOURCE_RX_PROTOCOL) {
                        // rssi sent sign-inverted
                        uint16_t rssiPercentScaled = scaleRange(-rssiFrame->rssi, GHST_RSSI_DBM_MIN, GHST_RSSI_DBM_MAX, 0, RSSI_MAX_VALUE);
                        rssiPercentScaled = MIN(rssiPercentScaled, RSSI_MAX_VALUE);
                        setRssi(rssiPercentScaled, RSSI_SOURCE_RX_PROTOCOL);
                    }

#ifdef USE_RX_RSSI_DBM
                    setRssiDbm(-rssiFrame->rssi, RSSI_SOURCE_RX_PROTOCOL);
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
                    if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_GHST) {
                        setLinkQualityDirect(rssiFrame->lq);
                    }
#endif
                    break;
                }

                case GHST_UL_RC_CHANS_HS4_5TO8:
                case GHST_UL_RC_CHANS_HS4_12_5TO8:
                    startIdx = 4;
                    break;

                case GHST_UL_RC_CHANS_HS4_9TO12:
                case GHST_UL_RC_CHANS_HS4_12_9TO12:
                    startIdx = 8;
                    break;

                case GHST_UL_RC_CHANS_HS4_13TO16:
                case GHST_UL_RC_CHANS_HS4_12_13TO16:
                    startIdx = 12;
                    break;

                default:
                    DEBUG_SET(DEBUG_GHST, DEBUG_GHST_UNKNOWN_FRAMES, ++unknownFrameCount);
                    break;
            }

            // We need to wait for the first RSSI frame to know ghstRfProtocol
            if (ghstRfProtocol != GHST_RF_PROTOCOL_UNDEFINED) {
                const ghstPayloadPulses_t* const rcChannels = (ghstPayloadPulses_t*)&ghstValidatedFrame->frame.payload;

                // all uplink frames contain CH1..4 data (12 bit)
                ghstChannelData[0] = rcChannels->ch1to4.ch1;
                ghstChannelData[1] = rcChannels->ch1to4.ch2;
                ghstChannelData[2] = rcChannels->ch1to4.ch3;
                ghstChannelData[3] = rcChannels->ch1to4.ch4;

                if (startIdx > 0) {
                    // remainder of uplink frame contains 4 more channels (8 bit), sent in a round-robin fashion
                    ghstChannelData[startIdx++] = rcChannels->cha << 2;
                    ghstChannelData[startIdx++] = rcChannels->chb << 2;
                    ghstChannelData[startIdx++] = rcChannels->chc << 2;
                    ghstChannelData[startIdx++] = rcChannels->chd << 2;
                }

                // Recalculate old scaling to the new one
                if (scalingLegacy) {
                    for (int i = 0; i < 4; i++) {
                        ghstChannelData[i] = ((5 * ghstChannelData[i]) >> 2) - 430;  // Primary
                        if (startIdx > 4) {
                            --startIdx;
                            ghstChannelData[startIdx] = 5 * (ghstChannelData[startIdx] >> 2) - 108; // Aux
                        }
                    }
                }

            }
        } else {
            switch(ghstFrameType) {

#if defined(USE_TELEMETRY_GHST) && defined(USE_MSP_OVER_TELEMETRY)
            case GHST_UL_MSP_REQ:
            case GHST_UL_MSP_WRITE: {
                static uint8_t mspFrameCounter = 0;
                DEBUG_SET(DEBUG_GHST_MSP, 0, ++mspFrameCounter);
                if (handleMspFrame(ghstValidatedFrame->frame.payload, ghstValidatedFrame->frame.len - GHST_FRAME_LENGTH_CRC - GHST_FRAME_LENGTH_TYPE, NULL)) {
                    ghstScheduleMspResponse();
                }
                break;
            }
#endif
            default:
                DEBUG_SET(DEBUG_GHST, DEBUG_GHST_UNKNOWN_FRAMES, ++unknownFrameCount);
                break;
            }
        }
    }

    return true;
}

STATIC_UNIT_TESTED float ghstReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);

    // Scaling 12bit channels (8bit channels in brackets)
    //      OpenTx          RC   PWM (BF)
    // min  -1024        0(  0)     988us
    // ctr      0     2048(128)    1500us
    // max   1024     4096(256)    2012us
    //

    // Scaling legacy (nearly 10bit)
    // derived from original SBus scaling, with slight correction for offset
    // now symmetrical around OpenTx 0 value
    // scaling is:
    //      OpenTx         RC   PWM (BF)
    // min  -1024     172( 22)     988us
    // ctr      0     992(124)    1500us
    // max   1024    1811(226)    2012us
    //

    float pwm = ghstChannelData[chan];

    if (chan < 4) {
        pwm = 0.25f * pwm;
    }

    return pwm + 988;
}

// UART idle detected (inter-packet)
static void ghstIdle(void)
{
    if (ghstTransmittingTelemetry) {
        ghstTransmittingTelemetry = false;
    }
}

bool ghstRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    rxRuntimeState->channelCount = GHST_MAX_NUM_CHANNELS;
    rxRuntimeState->rcReadRawFn = ghstReadRawRC;
    rxRuntimeState->rcFrameStatusFn = ghstFrameStatus;
    rxRuntimeState->rcProcessFrameFn = ghstProcessFrame;

    for (int iChan = 0; iChan < rxRuntimeState->channelCount; ++iChan) {
        if (iChan < 4 ) {
            ghstChannelData[iChan] = GHST_RC_CTR_VAL_12BIT_PRIMARY;
        } else {
            ghstChannelData[iChan] = GHST_RC_CTR_VAL_12BIT_AUX;
        }
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    serialPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        ghstDataReceive,
        NULL,
        GHST_RX_BAUDRATE,
        GHST_PORT_MODE,
        GHST_PORT_OPTIONS | (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0)
        );
    serialPort->idleCallback = ghstIdle;

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }

#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource == LQ_SOURCE_NONE) {
        linkQualitySource = LQ_SOURCE_RX_PROTOCOL_GHST;
    }
#endif

    return serialPort != NULL;
}

bool ghstRxIsActive(void)
{
    return serialPort != NULL;
}
#endif
