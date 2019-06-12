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

#include <stdint.h>

#include "platform.h"

#include "sdcard_standard.h"
#include "common/maths.h"

/**
 * Read a bitfield from an array of bits (the bit at index 0 being the most-significant bit of the first byte in
 * the buffer).
 */
uint32_t readBitfield(uint8_t *buffer, unsigned bitIndex, unsigned bitLen)
{
    uint32_t result = 0;
    unsigned bitInByteOffset = bitIndex % 8;
    uint8_t bufferByte;

    buffer += bitIndex / 8;

    // Align the bitfield to be read to the top of the buffer
    bufferByte = *buffer << bitInByteOffset;

    while (bitLen > 0) {
        unsigned bitsThisLoop = MIN(8 - bitInByteOffset, bitLen);

        result = (result << bitsThisLoop) | (bufferByte >> (8 - bitsThisLoop));

        buffer++;
        bufferByte = *buffer;

        bitLen -= bitsThisLoop;
        bitInByteOffset = 0;
    }

    return result;
}
