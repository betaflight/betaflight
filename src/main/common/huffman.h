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

#define HUFFMAN_TABLE_SIZE 257 // 256 characters plus EOF
typedef struct huffmanTable_s {
    uint8_t     codeLen;
    uint16_t    code;
} huffmanTable_t;

typedef struct huffmanState_s {
    uint16_t    bytesWritten;
    uint8_t     *outByte;
    uint16_t    outBufLen;
    uint8_t     outBit;
} huffmanState_t;

extern const huffmanTable_t huffmanTable[HUFFMAN_TABLE_SIZE];

struct huffmanInfo_s {
    uint16_t uncompressedByteCount;
};

#define HUFFMAN_INFO_SIZE sizeof(struct huffmanInfo_s)

int huffmanEncodeBuf(uint8_t *outBuf, int outBufLen, const uint8_t *inBuf, int inLen, const huffmanTable_t *huffmanTable);
int huffmanEncodeBufStreaming(huffmanState_t *state, const uint8_t *inBuf, int inLen, const huffmanTable_t *huffmanTable);
