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

// This file borrows heavily from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <platform.h>
#include "build_config.h"

#ifdef USE_RX_CX10

#include "drivers/rx_nrf24l01.h"
#include "drivers/system.h"

#include "rx/nrf24.h"
#include "rx/nrf24_cx10.h"

/*
 * Deviation transmitter
 * Bind phase lasts 6 seconds for CX10, for CX10A it lasts until an acknowledgment is received.
 * Other transmitters may vary but should have similar characteristics.
 * For CX10A protocol: after receiving a bind packet, the receiver must send back a data packet with byte[9] = 1 as acknowledgment
 */

/*
 * CX10 Protocol
 * No auto acknowledgment
 * Payload size is 19 and static for CX10A variant, 15 and static for CX10 variant.
 * Data rate is 1Mbps
 * Bind Phase
 * uses address {0xcc, 0xcc, 0xcc, 0xcc, 0xcc}, converted by XN297
 * uses channel 0x02
 * Data phase
 * uses same address as bind phase
 * hops between 4 channels that are set from the txId sent in the bind packet
 */

#define CX10_RC_CHANNEL_COUNT             9

#define CX10_PROTOCOL_PAYLOAD_SIZE       15
#define CX10A_PROTOCOL_PAYLOAD_SIZE      19

#define CX10_RF_BIND_CHANNEL           0x02

#define FLAG_FLIP       0x10 // goes to rudder channel
// flags1
#define FLAG_MODE_MASK  0x03
#define FLAG_HEADLESS   0x04
// flags2
#define FLAG_VIDEO      0x02
#define FLAG_PICTURE    0x04

// XN297 emulation layer
STATIC_UNIT_TESTED uint8_t XN297_WritePayload(uint8_t* data, int len);
STATIC_UNIT_TESTED void XN297_UnscramblePayload(uint8_t* data, int len);

static nrf24_protocol_t cx10Protocol;

typedef enum {
    STATE_BIND = 0,
    STATE_ACK,
    STATE_DATA
} protocol_state_t;

STATIC_UNIT_TESTED protocol_state_t protocolState;



#define ACK_TO_SEND_COUNT 8
static uint8_t payloadSize;

#define RX_TX_ADDR_LEN     5
//STATIC_UNIT_TESTED uint8_t rxTxAddr[RX_TX_ADDR_LEN] = {0xcc, 0xcc, 0xcc, 0xcc, 0xcc};
STATIC_UNIT_TESTED uint8_t txAddr[RX_TX_ADDR_LEN] = {0x55, 0x0F, 0x71, 0x0C, 0x00}; // converted XN297 address, 0xC710F55 (28 bit)
STATIC_UNIT_TESTED uint8_t rxAddr[RX_TX_ADDR_LEN] = {0x49, 0x26, 0x87, 0x7d, 0x2f}; // converted XN297 address
#define TX_ID_LEN 4
STATIC_UNIT_TESTED uint8_t txId[TX_ID_LEN];

STATIC_UNIT_TESTED uint8_t rfChannelIndex = 0;
#define RF_CHANNEL_COUNT 4
STATIC_UNIT_TESTED uint8_t rfChannels[RF_CHANNEL_COUNT]; // channels are set using txId from bind packet

static uint32_t timeOfLastHop;
static const uint32_t hopTimeout = 5000; // 5ms

void cx10Nrf24Init(nrf24_protocol_t protocol)
{
    cx10Protocol = protocol;
    protocolState = STATE_BIND;
    payloadSize = (protocol == NRF24RX_CX10) ? CX10_PROTOCOL_PAYLOAD_SIZE : CX10A_PROTOCOL_PAYLOAD_SIZE;

    NRF24L01_Initialize(0); // sets PWR_UP, no CRC

    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0);
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0));  // Enable data pipe 0 only
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES);   // 5-byte RX/TX address
    NRF24L01_SetChannel(CX10_RF_BIND_CHANNEL);

    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    // RX_ADDR for pipes P2 to P5 are left at default values
    NRF24L01_FlushRx();
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, txAddr, RX_TX_ADDR_LEN);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxAddr, RX_TX_ADDR_LEN);

    NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, payloadSize + 2); // payload + 2 bytes CRC

    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00); // Disable dynamic payload length on all pipes

    NRF24L01_SetRxMode(); // enter receive mode to start listening for packets
}

/*
 * Returns true if it is a bind packet.
 */
STATIC_UNIT_TESTED bool checkBindPacket(const uint8_t *packet)
{
    const bool bindPacket = (packet[0] == 0xaa);
    if (bindPacket) {
        txId[0] = packet[1];
        txId[1] = packet[2];
        txId[2] = packet[3];
        txId[3] = packet[4];
        return true;
    }
    return false;
}

STATIC_UNIT_TESTED uint16_t convertToPwmUnsigned(const uint8_t* pVal)
{
    uint16_t ret = (*(pVal + 1)) & 0x7f; // mask out top bit which is used for a flag for the rudder
    ret = (ret << 8) | *pVal;
    return ret;
}

void cx10SetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    const uint8_t offset = (cx10Protocol == NRF24RX_CX10) ? 0 : 4;
    rcData[NRF24_ROLL] = (PWM_RANGE_MAX + PWM_RANGE_MIN) - convertToPwmUnsigned(&payload[5 + offset]);  // aileron
    rcData[NRF24_PITCH] = (PWM_RANGE_MAX + PWM_RANGE_MIN) - convertToPwmUnsigned(&payload[7 + offset]); // elevator
    rcData[NRF24_THROTTLE] = convertToPwmUnsigned(&payload[9 + offset]); // throttle
    rcData[NRF24_YAW] = convertToPwmUnsigned(&payload[11 + offset]);  // rudder
    const uint8_t flags1 = payload[13 + offset];
    const uint8_t rate = flags1 & FLAG_MODE_MASK; // takes values 0, 1, 2
    if (rate == 0) {
        rcData[NRF24_AUX1] = PWM_RANGE_MIN;
    } else if (rate == 1) {
        rcData[NRF24_AUX1] = PWM_RANGE_MIDDLE;
    } else {
        rcData[NRF24_AUX1] = PWM_RANGE_MAX;
    }
    // flip flag is in YAW byte
    rcData[NRF24_AUX2] = payload[12 + offset] & FLAG_FLIP ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    const uint8_t flags2 = payload[14 + offset];
    rcData[NRF24_AUX3] = flags2 & FLAG_PICTURE ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[NRF24_AUX4] = flags2 & FLAG_VIDEO ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[NRF24_AUX5] = flags1 & FLAG_HEADLESS ? PWM_RANGE_MAX : PWM_RANGE_MIN;
}

static void hopToNextChannel(void)
{
    ++rfChannelIndex;
    if (rfChannelIndex >= RF_CHANNEL_COUNT) {
        rfChannelIndex = 0;
    }
    NRF24L01_SetChannel(rfChannels[rfChannelIndex]);
}

// The hopping channels are determined by the txId
STATIC_UNIT_TESTED void setHoppingChannels(const uint8_t* txId)
{
    rfChannelIndex = 0;
    rfChannels[0] = 0x03 + (txId[0] & 0x0F);
    rfChannels[1] = 0x16 + (txId[0] >> 4);
    rfChannels[2] = 0x2D + (txId[1] & 0x0F);
    rfChannels[3] = 0x40 + (txId[1] >> 4);
}

/*
 * This is called periodically by the scheduler.
 * Returns NRF24_RECEIVED_DATA if a data packet was received.
 */
nrf24_received_t cx10DataReceived(uint8_t *payload)
{
    static uint8_t ackCount;
    nrf24_received_t ret = NRF24_RECEIVED_NONE;
    int totalDelayUs;
    uint32_t timeNowUs;

    switch (protocolState) {
    case STATE_BIND:
        if (NRF24L01_ReadPayloadIfAvailable(payload, payloadSize + 2)) {
            XN297_UnscramblePayload(payload, payloadSize + 2);
            const bool bindPacket = checkBindPacket(payload);
            if (bindPacket) {
                // set the hopping channels as determined by the txId received in the bind packet
                setHoppingChannels(txId);
                ret = NRF24_RECEIVED_BIND;
                protocolState = STATE_ACK;
                ackCount = 0;
            }
        }
        break;
    case STATE_ACK:
        // transmit an ACK packet
        ++ackCount;
        totalDelayUs = 0;
        // send out an ACK on the bind channel, required by deviationTx
        payload[9] = 0x01;
        NRF24L01_SetChannel(CX10_RF_BIND_CHANNEL);
        NRF24L01_FlushTx();
        XN297_WritePayload(payload, payloadSize);
        NRF24L01_SetTxMode();// enter transmit mode to send the packet
        // wait for the ACK packet to send before changing channel
        static const int fifoDelayUs = 100;
        while (!(NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BV(NRF24L01_17_FIFO_STATUS_TX_EMPTY))) {
            delayMicroseconds(fifoDelayUs);
            totalDelayUs += fifoDelayUs;
        }
        // send out an ACK on each of the hopping channels, required by CX10 transmitter
        for (uint8_t ii = 0; ii < RF_CHANNEL_COUNT; ++ii) {
            NRF24L01_SetChannel(rfChannels[ii]);
            XN297_WritePayload(payload, payloadSize);
            NRF24L01_SetTxMode();// enter transmit mode to send the packet
            // wait for the ACK packet to send before changing channel
            while (!(NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BV(NRF24L01_17_FIFO_STATUS_TX_EMPTY))) {
                delayMicroseconds(fifoDelayUs);
                totalDelayUs += fifoDelayUs;
            }
        }
        static const int delayBetweenPacketsUs = 1000;
        if (totalDelayUs < delayBetweenPacketsUs) {
            delayMicroseconds(delayBetweenPacketsUs - totalDelayUs);
        }
        NRF24L01_SetRxMode();//reenter receive mode after sending ACKs
        if (ackCount > ACK_TO_SEND_COUNT) {
            NRF24L01_SetChannel(rfChannels[0]);
            // and go into data state to wait for first data packet
            protocolState = STATE_DATA;
        }
        break;
    case STATE_DATA:
        timeNowUs = micros();
        // read the payload, processing of payload is deferred
        if (NRF24L01_ReadPayloadIfAvailable(payload, payloadSize + 2)) {
            XN297_UnscramblePayload(payload, payloadSize + 2);
            hopToNextChannel();
            timeOfLastHop = timeNowUs;
            ret = NRF24_RECEIVED_DATA;
        }
        if (timeNowUs > timeOfLastHop + hopTimeout) {
            hopToNextChannel();
            timeOfLastHop = timeNowUs;
        }
    }
    return ret;
}

void cx10Init(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfig->channelCount = CX10_RC_CHANNEL_COUNT;
    cx10Nrf24Init((nrf24_protocol_t)rxConfig->nrf24rx_protocol);
}


// XN297 emulation layer

static const uint8_t xn297_data_scramble[30] = {
    0xbc, 0xe5, 0x66, 0x0d, 0xae, 0x8c, 0x88, 0x12,
    0x69, 0xee, 0x1f, 0xc7, 0x62, 0x97, 0xd5, 0x0b,
    0x79, 0xca, 0xcc, 0x1b, 0x5d, 0x19, 0x10, 0x24,
    0xd3, 0xdc, 0x3f, 0x8e, 0xc5, 0x2f
};

static uint8_t bitReverse(uint8_t bIn)
{
    uint8_t bOut = 0;
    for (int ii = 0; ii < 8; ++ii) {
        bOut = (bOut << 1) | (bIn & 1);
        bIn >>= 1;
    }
    return bOut;
}

static uint16_t crc16_update(uint16_t crc, unsigned char a)
{
    static const uint16_t crcPolynomial = 0x1021;
    crc ^= a << 8;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ crcPolynomial;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

STATIC_UNIT_TESTED uint8_t XN297_WritePayload(uint8_t* data, int len)
{
    uint8_t packet[NRF24L01_MAX_PAYLOAD_SIZE];
    uint16_t crc = 0xb5d2;
    for (int ii = 0; ii < RX_TX_ADDR_LEN; ++ii) {
        packet[ii] = rxAddr[RX_TX_ADDR_LEN - 1 - ii];
        crc = crc16_update(crc, packet[ii]);
    }
    for (int ii = 0; ii < len; ++ii) {
        // bit-reverse bytes in packet
        const uint8_t bOut = bitReverse(data[ii]);
        packet[ii + RX_TX_ADDR_LEN] = bOut ^ xn297_data_scramble[ii];
        crc = crc16_update(crc, packet[ii + RX_TX_ADDR_LEN]);
    }
    const uint16_t crcXor = (len == CX10_PROTOCOL_PAYLOAD_SIZE) ? 0x9BA7 : 0x61B1;
    crc ^= crcXor;
    packet[RX_TX_ADDR_LEN + len] = crc >> 8;
    packet[RX_TX_ADDR_LEN + len + 1] = crc & 0xff;
    return NRF24L01_WritePayload(packet, RX_TX_ADDR_LEN + len + 2);
}

STATIC_UNIT_TESTED void XN297_UnscramblePayload(uint8_t* data, int len)
{
    for (uint8_t ii = 0; ii < len; ++ii) {
        data[ii] = bitReverse(data[ii] ^ xn297_data_scramble[ii]);
    }
}

// End of XN297 emulation

#endif

