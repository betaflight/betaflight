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

#include "rx_spi.h"
#include "rx_nrf24l01.h"
#include "common/crc.h"


static const uint8_t xn297_data_scramble[30] = {
    0xbc, 0xe5, 0x66, 0x0d, 0xae, 0x8c, 0x88, 0x12,
    0x69, 0xee, 0x1f, 0xc7, 0x62, 0x97, 0xd5, 0x0b,
    0x79, 0xca, 0xcc, 0x1b, 0x5d, 0x19, 0x10, 0x24,
    0xd3, 0xdc, 0x3f, 0x8e, 0xc5, 0x2f
};

static const uint16_t xn297_crc_xorout[26] = {
    0x9BA7, 0x8BBB, 0x85E1, 0x3E8C, 0x451E, 0x18E6, 0x6B24, 0xE7AB,
    0x3828, 0x814B, 0xD461, 0xF494, 0x2503, 0x691D, 0xFE8B, 0x9BA7,
    0x8B17, 0x2920, 0x8B5F, 0x61B1, 0xD391, 0x7401, 0x2138, 0x129F,
    0xB3A0, 0x2988
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


#define RX_TX_ADDR_LEN 5

uint16_t XN297_UnscramblePayload(uint8_t *data, int len, const uint8_t *rxAddr)
{
    uint16_t crc = 0xb5d2;
    if (rxAddr) {
        for (int ii = 0; ii < RX_TX_ADDR_LEN; ++ii) {
            crc = crc16_ccitt(crc, rxAddr[RX_TX_ADDR_LEN - 1 - ii]);
        }
    }
    for (int ii = 0; ii < len; ++ii) {
        crc = crc16_ccitt(crc, data[ii]);
        data[ii] = bitReverse(data[ii] ^ xn297_data_scramble[ii]);
    }
    crc ^= xn297_crc_xorout[len];
    return crc;
}

uint8_t XN297_WritePayload(uint8_t *data, int len, const uint8_t *rxAddr)
{
    uint8_t packet[NRF24L01_MAX_PAYLOAD_SIZE];
    uint16_t crc = 0xb5d2;
    for (int ii = 0; ii < RX_TX_ADDR_LEN; ++ii) {
        packet[ii] = rxAddr[RX_TX_ADDR_LEN - 1 - ii];
        crc = crc16_ccitt(crc, packet[ii]);
    }
    for (int ii = 0; ii < len; ++ii) {
        const uint8_t bOut = bitReverse(data[ii]);
        packet[ii + RX_TX_ADDR_LEN] = bOut ^ xn297_data_scramble[ii];
        crc = crc16_ccitt(crc, packet[ii + RX_TX_ADDR_LEN]);
    }
    crc ^= xn297_crc_xorout[len];
    packet[RX_TX_ADDR_LEN + len] = crc >> 8;
    packet[RX_TX_ADDR_LEN + len + 1] = crc & 0xff;
    return NRF24L01_WritePayload(packet, RX_TX_ADDR_LEN + len + 2);
}

