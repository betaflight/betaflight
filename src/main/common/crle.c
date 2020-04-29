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

#include "common/crle.h"

// Composite run length encoder
// ----------------------------
// Data is encoded in-place at the beginning of the input buffer.  The resulting
// encoded length is returned to the caller.
//
// Whether or not the encoder encounters any run lengths, the resulting buffer is
// guaranteed to <= bufLen.
//
// A bitmask/prefix is applied to repeating bytes followed a numeric run length value up
// to 256 (1 byte).  Consequently, the prefix limits the available byte size to <= 0x7D.
// Run lengths longer than 256 bytes are broken into multiple encoded sets.
//
// Non-repeating characters are not considered by the encoder and are returned in their
// original form without a trailing run length of 1 (as found in traditiona RLE).
//
// This combination creates composite form of run length encoding with with an overall
// smaller size than standard run length encoding.
//
// This is ideal for encoding short arrays of values lower than 128 bits, making it a
// good fit for serial transmission for CMS and possibly other things that are text-like
// in nature.

size_t cRleEncode(uint8_t *buf, const size_t bufLen)
{
    uint8_t* cursor = buf;
    for(unsigned int i = 0; i < bufLen; i++) {
        size_t runLength = 1;
        const uint8_t c = buf[i];
        for (unsigned int j = i + 1; j < bufLen; j++) {
            if (buf[j] == c) {
                runLength++;
                i++;
            } else {
                break;
            }
        }
        if (runLength > 1) {
            uint8_t maxBatches = runLength / CRLE_MAX_RUN_LENGTH;
            while (maxBatches--) {
                *cursor++ = RLE_CHAR_REPEATED_MASK | c;
                *cursor++ = (uint8_t)CRLE_MAX_RUN_LENGTH;
            }
            const uint8_t remainder = runLength % CRLE_MAX_RUN_LENGTH;
            if (remainder > 0) {
                *cursor++ = RLE_CHAR_REPEATED_MASK | c;
                *cursor++ = remainder;
            }
        } else {
            *cursor++ = c;
        }
    }
    return (cursor - buf);
}

size_t cRleDecode(const uint8_t *source, uint8_t *dest, const size_t sourceBufLen) {
    const uint8_t* destStart = dest;
    for (unsigned int i = 0; i < sourceBufLen; i++) {
        const uint8_t c = source[i] & RLE_CHAR_VALUE_MASK;
        if (source[i] & RLE_CHAR_REPEATED_MASK) {
            uint8_t rep = source[++i];
            while(rep--) {
                *dest++ = c;
            }
        } else {
            *dest++ = c;
        }
    }
    return dest - destStart;
}
