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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define GHST_RX_BAUDRATE                420000

#define GHST_TX_BAUDRATE_FAST           400000
#define GHST_TX_BAUDRATE_SLOW           115200
#define GHST_BYTE_TIME_FAST_US          ((1000000/GHST_TX_BAUDRATE_FAST)*10)      // 10 bit words (8 data, 1 start, 1 stop)
#define GHST_BYTE_TIME_SLOW_US          ((1000000/GHST_TX_BAUDRATE_SLOW)*10)
#define GHST_UART_WORDLENGTH            UART_WORDLENGTH_8B

typedef enum {
    GHST_ADDR_RADIO             = 0x80,
    GHST_ADDR_TX_MODULE_SYM     = 0x81,     // symmetrical, 400k pulses, 400k telemetry
    GHST_ADDR_TX_MODULE_ASYM    = 0x88,     // asymmetrical, 400k pulses, 115k telemetry
    GHST_ADDR_FC                = 0x82,
    GHST_ADDR_GOGGLES           = 0x83,
    GHST_ADDR_QUANTUM_TEE1      = 0x84,     // phase 2
    GHST_ADDR_QUANTUM_TEE2      = 0x85,
    GHST_ADDR_QUANTUM_GW1       = 0x86,
    GHST_ADDR_5G_CLK            = 0x87,     // phase 3
    GHST_ADDR_RX                = 0x89
} ghstAddr_e;

typedef enum {
    GHST_UL_RC_CHANS_HS4_5TO8   = 0x10,     // High Speed 4 channel, plus CH5-8
    GHST_UL_RC_CHANS_HS4_9TO12  = 0x11,     // High Speed 4 channel, plus CH9-12
    GHST_UL_RC_CHANS_HS4_13TO16 = 0x12      // High Speed 4 channel, plus CH13-16
} ghstUl_e;

#define GHST_UL_RC_CHANS_SIZE       12      // 1 (type) + 10 (data) + 1 (crc)

typedef enum {
    GHST_DL_OPENTX_SYNC         = 0x20,
    GHST_DL_LINK_STAT           = 0x21,
    GHST_DL_VTX_STAT            = 0x22,
    GHST_DL_PACK_STAT           = 0x23,     // Battery (Pack) Status
} ghstDl_e;

#define GHST_RC_CTR_VAL_12BIT       0x7C0   // servo center for 12 bit values (0x3e0 << 1)
#define GHST_RC_CTR_VAL_8BIT        0x7C    // servo center for 8 bit values

#define GHST_FRAME_SIZE             14      // including addr, type, len, crc, and payload

#define GHST_PAYLOAD_SIZE_MAX           14

#define GHST_FRAME_SIZE_MAX             24

typedef struct ghstFrameDef_s {
    uint8_t addr;
    uint8_t len;
    uint8_t type;
    uint8_t payload[GHST_PAYLOAD_SIZE_MAX + 1];         // CRC adds 1
} ghstFrameDef_t;

typedef union ghstFrame_u {
    uint8_t bytes[GHST_FRAME_SIZE];
    ghstFrameDef_t frame;
} ghstFrame_t;

/* Pulses payload (channel data). Includes 4x high speed control channels, plus 4 channels from CH5-CH12 */
typedef struct ghstPayloadPulses_s {
    // 80 bits, or 10 bytes
    unsigned int ch1: 12;
    unsigned int ch2: 12;
    unsigned int ch3: 12;
    unsigned int ch4: 12;

    unsigned int cha: 8;
    unsigned int chb: 8;
    unsigned int chc: 8;
    unsigned int chd: 8;
} __attribute__ ((__packed__)) ghstPayloadPulses_t;
