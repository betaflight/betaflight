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
#include <string.h>

#include "platform.h"

#ifdef USE_RX_INAV

#include "build/build_config.h"

#include "drivers/rx_nrf24l01.h"
#include "drivers/system.h"

#include "rx/nrf24.h"
#include "rx/nrf24_inav.h"

#include "telemetry/ltm.h"
#include "telemetry/nrf24_ltm.h"


/*
 * iNav Protocol
 * No auto acknowledgment
 * Data rate is 250Kbps - lower data rate for better reliability and range
 * Payload size is 16, static, small payload is read more quickly (marginal benefit)
 *
 * Bind Phase
 * uses address {0x4b,0x5c,0x6d,0x7e,0x8f}
 * uses channel 0x4c
 *
 * Data Phase
 * uses the address received in bind packet
 * hops between 4 RF channels generated from the address received in bind packet
 *
 * There are 16 channels: eight 10-bit analog channels, two 8-bit analog channels, and six digital channels as follows:
 *
 * Channels 0 to 3, are the AETR channels, values 1000 to 2000 with resolution of 1 (10-bit channels)
 * Channel AUX1 by deviation convention is used for rate, values 1000, 1500, 2000
 * Channels AUX2 to AUX6 are binary channels, values 1000 or 2000,
 *     by deviation convention used for flip, picture, video, headless, and return to home
 * Channels AUX7 to AUX10 are analog channels, values 1000 to 2000 with resolution of 1 (10-bit channels)
 * Channels AUX11 and AUX12 are analog channels, values 1000 to 2000 with resolution of 4 (8-bit channels)
 */

#define RC_CHANNEL_COUNT 16

enum {
    RATE_LOW = 0,
    RATE_MID = 1,
    RATE_HIGH = 2,
};

enum {
    FLAG_FLIP     = 0x01,
    FLAG_PICTURE  = 0x02,
    FLAG_VIDEO    = 0x04,
    FLAG_RTH      = 0x08,
    FLAG_HEADLESS = 0x10,
};

typedef enum {
    STATE_BIND = 0,
    STATE_DATA
} protocol_state_t;

STATIC_UNIT_TESTED protocol_state_t protocolState;

STATIC_UNIT_TESTED uint8_t ackPayload[NRF24L01_MAX_PAYLOAD_SIZE];

#define INAV_PROTOCOL_PAYLOAD_SIZE 16
STATIC_UNIT_TESTED const uint8_t payloadSize = INAV_PROTOCOL_PAYLOAD_SIZE;
uint8_t receivedPowerSnapshot;

#define RX_TX_ADDR_LEN 5
// set rxTxAddr to the bind address
STATIC_UNIT_TESTED uint8_t rxTxAddr[RX_TX_ADDR_LEN] = {0x4b,0x5c,0x6d,0x7e,0x8f};
uint32_t *nrf24rxIdPtr;

// radio channels for frequency hopping
#define INAV_RF_CHANNEL_COUNT 4
STATIC_UNIT_TESTED const uint8_t inavRfChannelCount = INAV_RF_CHANNEL_COUNT;
STATIC_UNIT_TESTED uint8_t inavRfChannelIndex;
STATIC_UNIT_TESTED uint8_t inavRfChannels[INAV_RF_CHANNEL_COUNT];
#define INAV_RF_BIND_CHANNEL 0x4c

static uint32_t timeOfLastHop;
static const uint32_t hopTimeout = 5000; // 5ms

STATIC_UNIT_TESTED bool inavCheckBindPacket(const uint8_t *payload)
{
    bool bindPacket = false;
    if (payload[0] == 0xae  && payload[1] == 0xc9) {
        bindPacket = true;
        if (protocolState ==STATE_BIND) {
            rxTxAddr[0] = payload[2];
            rxTxAddr[1] = payload[3];
            rxTxAddr[2] = payload[4];
            rxTxAddr[3] = payload[5];
            rxTxAddr[4] = payload[6];
            if (nrf24rxIdPtr != NULL && *nrf24rxIdPtr == 0) {
                // copy the rxTxAddr so it can be saved
                memcpy(nrf24rxIdPtr, rxTxAddr, sizeof(uint32_t));
            }
        }
    }
    return bindPacket;
}

void inavNrf24SetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    // the AETR channels have 10 bit resolution
    uint8_t lowBits = payload[6]; // least significant bits for AETR
    rcData[NRF24_ROLL]     = PWM_RANGE_MIN + ((payload[2] << 2) | (lowBits & 0x03)); // Aileron
    lowBits >>= 2;
    rcData[NRF24_PITCH]    = PWM_RANGE_MIN + ((payload[3] << 2) | (lowBits & 0x03)); // Elevator
    lowBits >>= 2;
    rcData[NRF24_THROTTLE] = PWM_RANGE_MIN + ((payload[4] << 2) | (lowBits & 0x03)); // Throttle
    lowBits >>= 2;
    rcData[NRF24_YAW]      = PWM_RANGE_MIN + ((payload[5] << 2) | (lowBits & 0x03)); // Rudder

    // channel AUX1 is used for rate, as per the deviation convention
    const uint8_t rate = payload[7];
    if (rate == RATE_HIGH) {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MAX;
    } else if (rate == RATE_MID) {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIDDLE;
    } else {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIN;
    }

    // channels AUX2 to AUX7 use the deviation convention
    const uint8_t flags = payload[8];
    rcData[RC_CHANNEL_FLIP]= (flags & FLAG_FLIP) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_PICTURE]= (flags & FLAG_PICTURE) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_VIDEO]= (flags & FLAG_VIDEO) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_HEADLESS]= (flags & FLAG_HEADLESS) ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_RTH]= (flags & FLAG_RTH) ? PWM_RANGE_MAX : PWM_RANGE_MIN;

    // channels AUX7 to AUX10 have 10 bit resolution
    lowBits = payload[13]; // least significant bits for AUX7 to AUX10
    rcData[NRF24_AUX7] = PWM_RANGE_MIN + ((payload[9] << 2) | (lowBits & 0x03));
    lowBits >>= 2;
    rcData[NRF24_AUX8] = PWM_RANGE_MIN + ((payload[10] << 2) | (lowBits & 0x03));
    lowBits >>= 2;
    rcData[NRF24_AUX9] = PWM_RANGE_MIN + ((payload[11] << 2) | (lowBits & 0x03));
    lowBits >>= 2;
    rcData[NRF24_AUX10] = PWM_RANGE_MIN + ((payload[12] << 2) | (lowBits & 0x03));
    lowBits >>= 2;

    // channels AUX11 and AUX12 have 8 bit resolution
    rcData[NRF24_AUX11] = PWM_RANGE_MIN + (payload[14] << 2);
    rcData[NRF24_AUX12] = PWM_RANGE_MIN + (payload[15] << 2);
}

static void inavHopToNextChannel(void)
{
    ++inavRfChannelIndex;
    if (inavRfChannelIndex >= inavRfChannelCount) {
        inavRfChannelIndex = 0;
    }
    NRF24L01_SetChannel(inavRfChannels[inavRfChannelIndex]);
}

// The hopping channels are determined by the low bits of rxTxAddr
STATIC_UNIT_TESTED void inavSetHoppingChannels(uint8_t addr)
{
    addr &= 0x07;
    inavRfChannels[0] = 0x10 + addr;
    inavRfChannels[1] = 0x1C + addr;
    inavRfChannels[2] = 0x28 + addr;
    inavRfChannels[3] = 0x34 + addr;
}

static void inavSetBound(void)
{
    protocolState = STATE_DATA;
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, RX_TX_ADDR_LEN);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rxTxAddr, RX_TX_ADDR_LEN);
    inavSetHoppingChannels(rxTxAddr[0]);
    timeOfLastHop = micros();
    inavRfChannelIndex = 0;
    NRF24L01_SetChannel(inavRfChannels[0]);
}

/*
 * This is called periodically by the scheduler.
 * Returns NRF24L01_RECEIVED_DATA if a data packet was received.
 */
nrf24_received_t inavNrf24DataReceived(uint8_t *payload)
{
#if defined(TELEMETRY_NRF24_LTM)
    static ltm_frame_e ltmFrameType = LTM_FRAME_START;
#endif

    nrf24_received_t ret = NRF24_RECEIVED_NONE;
    uint32_t timeNowUs;
    switch (protocolState) {
    case STATE_BIND:
        if (NRF24L01_ReadPayloadIfAvailable(payload, payloadSize)) {
            const bool bindPacket = inavCheckBindPacket(payload);
            if (bindPacket) {
                // send back the payload with the first two bytes set to zero as the ack
                payload[0] = 0;
                payload[1] = 0;
                NRF24L01_WriteAckPayload(payload, payloadSize, NRF24L01_PIPE0);
                ret = NRF24_RECEIVED_BIND;
                // got a bind packet, so set the hopping channels and the rxTxAddr and start listening for data
                inavSetBound();
            }
        }
        break;
    case STATE_DATA:
        timeNowUs = micros();
        // read the payload, processing of payload is deferred
        if (NRF24L01_ReadPayloadIfAvailable(payload, payloadSize)) {
            receivedPowerSnapshot = NRF24L01_ReadReg(NRF24L01_09_RPD); // set to 1 if received power > -64dBm
            const bool bindPacket = inavCheckBindPacket(payload);
            if (bindPacket) {
                // transmitter may still continue to transmit bind packets after we have switched to data mode
                // send back the payload with the first two bytes set to zero as the ack
                payload[0] = 0;
                payload[1] = 0;
                NRF24L01_WriteAckPayload(payload, payloadSize, NRF24L01_PIPE0);
                ret = NRF24_RECEIVED_BIND;
            } else {
                ret = NRF24_RECEIVED_DATA;
#if defined(TELEMETRY_NRF24_LTM)
                // set up telemetry data, send back telemetry data in the ACK packet
                const int ackPayloadSize = getNrf24LtmDatagram(ackPayload, ltmFrameType);
                ++ltmFrameType;
                if (ltmFrameType > LTM_FRAME_COUNT) {
                    ltmFrameType = LTM_FRAME_START;
                }
                NRF24L01_WriteAckPayload(ackPayload, ackPayloadSize, NRF24L01_PIPE0);
#endif
            }
        }
        if ((ret == NRF24_RECEIVED_DATA) || (timeNowUs > timeOfLastHop + hopTimeout)) {
            inavHopToNextChannel();
            timeOfLastHop = timeNowUs;
        }
        break;
    }
    return ret;
}

static void inavNrf24Setup(nrf24_protocol_t protocol, const uint32_t *nrf24rx_id)
{
    UNUSED(protocol);

    NRF24L01_Initialize(BV(NRF24L01_00_CONFIG_EN_CRC) | BV( NRF24L01_00_CONFIG_CRCO)); // sets PWR_UP, EN_CRC, CRCO - 2 byte CRC
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, NRF24L01_01_EN_AA_ALL_PIPES); // auto acknowledgment on all pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0));
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // auto retransmit disabled

    nrf24rxIdPtr = (uint32_t*)nrf24rx_id;
    if (nrf24rx_id == NULL || *nrf24rx_id == 0) {
        protocolState = STATE_BIND;
        NRF24L01_SetChannel(INAV_RF_BIND_CHANNEL);
    } else {
        memcpy(rxTxAddr, nrf24rx_id, sizeof(uint32_t));
        rxTxAddr[4] = 0xD2;
        inavSetBound();
    }
    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_250Kbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    // RX_ADDR for pipes P1-P5 are left at default values
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, RX_TX_ADDR_LEN);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rxTxAddr, RX_TX_ADDR_LEN);

    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, payloadSize);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, NRF24L01_1C_DYNPD_ALL_PIPES); // dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, BV(NRF24L01_1D_FEATURE_EN_DPL) | BV(NRF24L01_1D_FEATURE_EN_ACK_PAY));

    NRF24L01_SetRxMode(); // enter receive mode to start listening for packets
    // put a null packet in the transmit buffer to be sent as ACK on first receive
    NRF24L01_WriteAckPayload(ackPayload, payloadSize, NRF24L01_PIPE0);
}

void inavNrf24Init(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT;
    inavNrf24Setup((nrf24_protocol_t)rxConfig->nrf24rx_protocol, &rxConfig->nrf24rx_id);
}
#endif

