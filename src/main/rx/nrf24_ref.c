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
#include "build_config.h"

#ifdef USE_RX_REF

#include "drivers/rx_nrf24l01.h"
#include "drivers/system.h"

#include "rx/nrf24.h"
#include "rx/nrf24_ref.h"



/*
 * Reference Protocol
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

#define REF_PROTOCOL_PAYLOAD_SIZE 16
STATIC_UNIT_TESTED const uint8_t payloadSize = REF_PROTOCOL_PAYLOAD_SIZE;

#define RX_TX_ADDR_LEN 5
// set rxTxAddr to the bind values
STATIC_UNIT_TESTED uint8_t rxTxAddr[RX_TX_ADDR_LEN] = {0x4b,0x5c,0x6d,0x7e,0x8f};

// radio channels for frequency hopping
#define REF_RF_CHANNEL_COUNT 4
STATIC_UNIT_TESTED const uint8_t refRfChannelCount = REF_RF_CHANNEL_COUNT;
STATIC_UNIT_TESTED uint8_t refRfChannelIndex;
STATIC_UNIT_TESTED uint8_t refRfChannels[REF_RF_CHANNEL_COUNT];
#define REF_RF_BIND_CHANNEL 0x4c

//static uint32_t packetCount = 0;
static uint32_t timeOfLastHop;
static const uint32_t hopTimeout = 5000; // 5ms

STATIC_UNIT_TESTED bool refCheckBindPacket(const uint8_t *payload)
{
    bool bindPacket = false;
    if (payload[0] == 0xae  && payload[1] == 0xc9) {
        bindPacket = true;
        rxTxAddr[0] = payload[2];
        rxTxAddr[1] = payload[3];
        rxTxAddr[2] = payload[4];
        rxTxAddr[3] = payload[5];
        rxTxAddr[4] = payload[6];
    }
    return bindPacket;
}

void refSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
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

static void refHopToNextChannel(void)
{
    ++refRfChannelIndex;
    if (refRfChannelIndex >= refRfChannelCount) {
        refRfChannelIndex = 0;
    }
    NRF24L01_SetChannel(refRfChannels[refRfChannelIndex]);
}

// The hopping channels are determined by the low bits of rxTxAddr
STATIC_UNIT_TESTED void refSetHoppingChannels(uint32_t addr)
{
    addr = addr & 0x1f;
    const uint32_t inc = (addr << 24) | (addr << 16) | (addr << 8) | addr;
    uint32_t * const prfChannels = (uint32_t *)refRfChannels;
    *prfChannels = 0x10314259 + inc;
}

void refSetBound(const uint8_t* rxTxAddr)
{
    protocolState = STATE_DATA;
    refSetHoppingChannels(rxTxAddr[0]);
    timeOfLastHop = micros();
    refRfChannelIndex = 0;
    NRF24L01_SetChannel(refRfChannels[0]);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, RX_TX_ADDR_LEN);
}

/*
 * This is called periodically by the scheduler.
 * Returns NRF24L01_RECEIVED_DATA if a data packet was received.
 */
nrf24_received_t refDataReceived(uint8_t *payload)
{
    nrf24_received_t ret = NRF24_RECEIVED_NONE;
    uint32_t timeNowUs;
    switch (protocolState) {
    case STATE_BIND:
        if (NRF24L01_ReadPayloadIfAvailable(payload, payloadSize)) {
            const bool bindPacket = refCheckBindPacket(payload);
            if (bindPacket) {
                ret = NRF24_RECEIVED_BIND;
                // got a bind packet, so set the hopping channels and the rxTxAddr and start listening for data
                refSetBound(rxTxAddr);
            }
        }
        break;
    case STATE_DATA:
        timeNowUs = micros();
        // read the payload, processing of payload is deferred
        if (NRF24L01_ReadPayloadIfAvailable(payload, payloadSize)) {
            ret = NRF24_RECEIVED_DATA;
        }
        if ((ret == NRF24_RECEIVED_DATA) || (timeNowUs > timeOfLastHop + hopTimeout)) {
            refHopToNextChannel();
            timeOfLastHop = timeNowUs;
        }
        break;
    }
    return ret;
}

void refNrf24Init(nrf24_protocol_t protocol, const uint8_t* nrf24_id)
{
    UNUSED(protocol);

    NRF24L01_Initialize(BV(NRF24L01_00_CONFIG_EN_CRC) | BV( NRF24L01_00_CONFIG_CRCO)); // sets PWR_UP, EN_CRC, CRCO - 2 byte CRC

    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00); // No auto acknowledgment
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0));
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES);   // 5-byte RX/TX address
    if ((nrf24_id[0] | nrf24_id[1] | nrf24_id[2] | nrf24_id[3] | nrf24_id[4]) == 0) {
        protocolState = STATE_BIND;
        NRF24L01_SetChannel(REF_RF_BIND_CHANNEL);
    } else {
        rxTxAddr[0] = nrf24_id[0];
        rxTxAddr[1] = nrf24_id[1];
        rxTxAddr[2] = nrf24_id[2];
        rxTxAddr[3] = nrf24_id[3];
        rxTxAddr[4] = nrf24_id[4];
        refSetBound(nrf24_id);
    }
    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_250Kbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    // RX_ADDR for pipes P1-P5 are left at default values
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, RX_TX_ADDR_LEN);

    NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00); // Disable dynamic payload length on all pipes

    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, payloadSize);
    NRF24L01_SetRxMode(); // enter receive mode to start listening for packets
}

void refInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT;
    refNrf24Init((nrf24_protocol_t)rxConfig->nrf24rx_protocol, rxConfig->nrf24rx_id);
}
#endif

