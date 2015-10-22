#include <stdint.h>

#include "sdcard_standard.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

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
