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

#include "drivers/rx_nrf24l01.h"
#include "common/maths.h"


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

void XN297_UnscramblePayload(uint8_t* data, int len)
{
    for (uint8_t ii = 0; ii < len; ++ii) {
        data[ii] = bitReverse(data[ii] ^ xn297_data_scramble[ii]);
    }
}


#define RX_TX_ADDR_LEN     5
#define PROTOCOL_PAYLOAD_SIZE_15       15
#define PROTOCOL_PAYLOAD_SIZE_19       19

uint8_t XN297_WritePayload(uint8_t *data, int len, const uint8_t *rxAddr)
{
    uint8_t packet[NRF24L01_MAX_PAYLOAD_SIZE];
    uint16_t crc = 0xb5d2;
    for (int ii = 0; ii < RX_TX_ADDR_LEN; ++ii) {
        packet[ii] = rxAddr[RX_TX_ADDR_LEN - 1 - ii];
        crc = crc16_ccitt(crc, packet[ii]);
    }
    for (int ii = 0; ii < len; ++ii) {
        // bit-reverse bytes in packet
        const uint8_t bOut = bitReverse(data[ii]);
        packet[ii + RX_TX_ADDR_LEN] = bOut ^ xn297_data_scramble[ii];
        crc = crc16_ccitt(crc, packet[ii + RX_TX_ADDR_LEN]);
    }
    const uint16_t crcXor = (len == PROTOCOL_PAYLOAD_SIZE_15) ? 0x9BA7 : 0x61B1;
    crc ^= crcXor;
    packet[RX_TX_ADDR_LEN + len] = crc >> 8;
    packet[RX_TX_ADDR_LEN + len + 1] = crc & 0xff;
    return NRF24L01_WritePayload(packet, RX_TX_ADDR_LEN + len + 2);
}

