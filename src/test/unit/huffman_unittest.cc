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

#include <stdint.h>

extern "C" {
    #include "common/huffman.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define OUTBUF_LEN 128
static uint8_t outBuf[OUTBUF_LEN];

/*
 * Huffman Tree, used to decompress a bytestream.
 *
 * The leaf nodes of the Huffman tree are stored in an array.
 */

#define HUFFMAN_EOF (-1)

#define HUFFMAN_TREE_SIZE 257 // 256 characters plus EOF

typedef struct huffmanTree_s {
    int16_t     value;
    uint16_t    codeLen;
    uint16_t    code;
} huffmanTree_t;

static const huffmanTree_t huffmanTree[HUFFMAN_TREE_SIZE] = {
//    Char  Len    Code         Bitcode
    { 0x00,   2, 0x0003 },  //  11
    { 0x01,   3, 0x0005 },  //  101
    { 0x02,   4, 0x0009 },  //  1001
    { 0x03,   5, 0x0011 },  //  10001
    { 0x04,   5, 0x0010 },  //  10000
    { 0x50,   5, 0x000F },  //  01111
    { 0x05,   6, 0x001D },  //  011101
    { 0x06,   6, 0x001C },  //  011100
    { 0x07,   6, 0x001B },  //  011011
    { 0x08,   6, 0x001A },  //  011010
    { 0x10,   6, 0x0019 },  //  011001
    { 0x09,   7, 0x0031 },  //  0110001
    { 0x0A,   7, 0x0030 },  //  0110000
    { 0x0B,   7, 0x002F },  //  0101111
    { 0x0C,   7, 0x002E },  //  0101110
    { 0x0D,   7, 0x002D },  //  0101101
    { 0x0E,   7, 0x002C },  //  0101100
    { 0x0F,   7, 0x002B },  //  0101011
    { 0x11,   7, 0x002A },  //  0101010
    { 0x12,   7, 0x0029 },  //  0101001
    { 0x13,   8, 0x0051 },  //  01010001
    { 0x14,   8, 0x0050 },  //  01010000
    { 0x15,   8, 0x004F },  //  01001111
    { 0x16,   8, 0x004E },  //  01001110
    { 0x17,   8, 0x004D },  //  01001101
    { 0x18,   8, 0x004C },  //  01001100
    { 0x19,   8, 0x004B },  //  01001011
    { 0x1A,   8, 0x004A },  //  01001010
    { 0x1B,   8, 0x0049 },  //  01001001
    { 0x1C,   8, 0x0048 },  //  01001000
    { 0x1D,   8, 0x0047 },  //  01000111
    { 0x1E,   8, 0x0046 },  //  01000110
    { 0x1F,   8, 0x0045 },  //  01000101
    { 0x20,   8, 0x0044 },  //  01000100
    { 0x21,   8, 0x0043 },  //  01000011
    { 0x22,   8, 0x0042 },  //  01000010
    { 0x23,   8, 0x0041 },  //  01000001
    { 0x24,   8, 0x0040 },  //  01000000
    { 0x30,   8, 0x003F },  //  00111111
    { 0x40,   8, 0x003E },  //  00111110
    { 0xF0,   8, 0x003D },  //  00111101
    { 0x25,   9, 0x0079 },  //  001111001
    { 0x26,   9, 0x0078 },  //  001111000
    { 0x27,   9, 0x0077 },  //  001110111
    { 0x28,   9, 0x0076 },  //  001110110
    { 0x29,   9, 0x0075 },  //  001110101
    { 0x2A,   9, 0x0074 },  //  001110100
    { 0x2B,   9, 0x0073 },  //  001110011
    { 0x2C,   9, 0x0072 },  //  001110010
    { 0x2D,   9, 0x0071 },  //  001110001
    { 0x2E,   9, 0x0070 },  //  001110000
    { 0x2F,   9, 0x006F },  //  001101111
    { 0x31,   9, 0x006E },  //  001101110
    { 0x32,   9, 0x006D },  //  001101101
    { 0x33,   9, 0x006C },  //  001101100
    { 0x34,   9, 0x006B },  //  001101011
    { 0x35,   9, 0x006A },  //  001101010
    { 0x36,   9, 0x0069 },  //  001101001
    { 0x37,   9, 0x0068 },  //  001101000
    { 0x38,   9, 0x0067 },  //  001100111
    { 0x39,   9, 0x0066 },  //  001100110
    { 0x3A,   9, 0x0065 },  //  001100101
    { 0x3B,   9, 0x0064 },  //  001100100
    { 0x3C,   9, 0x0063 },  //  001100011
    { 0x3D,   9, 0x0062 },  //  001100010
    { 0x3E,   9, 0x0061 },  //  001100001
    { 0x3F,   9, 0x0060 },  //  001100000
    { 0x41,   9, 0x005F },  //  001011111
    { 0x42,   9, 0x005E },  //  001011110
    { 0x43,   9, 0x005D },  //  001011101
    { 0x44,   9, 0x005C },  //  001011100
    { 0x45,   9, 0x005B },  //  001011011
    { 0x46,   9, 0x005A },  //  001011010
    { 0x47,   9, 0x0059 },  //  001011001
    { 0x48,   9, 0x0058 },  //  001011000
    { 0x49,   9, 0x0057 },  //  001010111
    { 0x4C,   9, 0x0056 },  //  001010110
    { 0x4F,   9, 0x0055 },  //  001010101
    { 0x51,   9, 0x0054 },  //  001010100
    { 0x80,   9, 0x0053 },  //  001010011
    { 0xE0,   9, 0x0052 },  //  001010010
    { 0xF1,   9, 0x0051 },  //  001010001
    { 0xFF,   9, 0x0050 },  //  001010000
    { 0x4A,  10, 0x009F },  //  0010011111
    { 0x4B,  10, 0x009E },  //  0010011110
    { 0x4D,  10, 0x009D },  //  0010011101
    { 0x4E,  10, 0x009C },  //  0010011100
    { 0x52,  10, 0x009B },  //  0010011011
    { 0x53,  10, 0x009A },  //  0010011010
    { 0x54,  10, 0x0099 },  //  0010011001
    { 0x55,  10, 0x0098 },  //  0010011000
    { 0x56,  10, 0x0097 },  //  0010010111
    { 0x57,  10, 0x0096 },  //  0010010110
    { 0x58,  10, 0x0095 },  //  0010010101
    { 0x59,  10, 0x0094 },  //  0010010100
    { 0x5A,  10, 0x0093 },  //  0010010011
    { 0x5B,  10, 0x0092 },  //  0010010010
    { 0x5C,  10, 0x0091 },  //  0010010001
    { 0x5D,  10, 0x0090 },  //  0010010000
    { 0x5E,  10, 0x008F },  //  0010001111
    { 0x5F,  10, 0x008E },  //  0010001110
    { 0x60,  10, 0x008D },  //  0010001101
    { 0x61,  10, 0x008C },  //  0010001100
    { 0x62,  10, 0x008B },  //  0010001011
    { 0x63,  10, 0x008A },  //  0010001010
    { 0x64,  10, 0x0089 },  //  0010001001
    { 0x65,  10, 0x0088 },  //  0010001000
    { 0x66,  10, 0x0087 },  //  0010000111
    { 0x67,  10, 0x0086 },  //  0010000110
    { 0x68,  10, 0x0085 },  //  0010000101
    { 0x69,  10, 0x0084 },  //  0010000100
    { 0x6A,  10, 0x0083 },  //  0010000011
    { 0x6B,  10, 0x0082 },  //  0010000010
    { 0x6C,  10, 0x0081 },  //  0010000001
    { 0x6D,  10, 0x0080 },  //  0010000000
    { 0x6E,  10, 0x007F },  //  0001111111
    { 0x6F,  10, 0x007E },  //  0001111110
    { 0x70,  10, 0x007D },  //  0001111101
    { 0x71,  10, 0x007C },  //  0001111100
    { 0x72,  10, 0x007B },  //  0001111011
    { 0x73,  10, 0x007A },  //  0001111010
    { 0x74,  10, 0x0079 },  //  0001111001
    { 0x75,  10, 0x0078 },  //  0001111000
    { 0x76,  10, 0x0077 },  //  0001110111
    { 0x77,  10, 0x0076 },  //  0001110110
    { 0x78,  10, 0x0075 },  //  0001110101
    { 0x79,  10, 0x0074 },  //  0001110100
    { 0x7A,  10, 0x0073 },  //  0001110011
    { 0x7B,  10, 0x0072 },  //  0001110010
    { 0x7C,  10, 0x0071 },  //  0001110001
    { 0x7D,  10, 0x0070 },  //  0001110000
    { 0x7E,  10, 0x006F },  //  0001101111
    { 0x7F,  10, 0x006E },  //  0001101110
    { 0x81,  10, 0x006D },  //  0001101101
    { 0x82,  10, 0x006C },  //  0001101100
    { 0x83,  10, 0x006B },  //  0001101011
    { 0x84,  10, 0x006A },  //  0001101010
    { 0x85,  10, 0x0069 },  //  0001101001
    { 0x86,  10, 0x0068 },  //  0001101000
    { 0x87,  10, 0x0067 },  //  0001100111
    { 0x88,  10, 0x0066 },  //  0001100110
    { 0x89,  10, 0x0065 },  //  0001100101
    { 0x8A,  10, 0x0064 },  //  0001100100
    { 0x8B,  10, 0x0063 },  //  0001100011
    { 0x8C,  10, 0x0062 },  //  0001100010
    { 0x8D,  10, 0x0061 },  //  0001100001
    { 0x8E,  10, 0x0060 },  //  0001100000
    { 0x8F,  10, 0x005F },  //  0001011111
    { 0x90,  10, 0x005E },  //  0001011110
    { 0x91,  10, 0x005D },  //  0001011101
    { 0x92,  10, 0x005C },  //  0001011100
    { 0x93,  10, 0x005B },  //  0001011011
    { 0x94,  10, 0x005A },  //  0001011010
    { 0x95,  10, 0x0059 },  //  0001011001
    { 0x96,  10, 0x0058 },  //  0001011000
    { 0x97,  10, 0x0057 },  //  0001010111
    { 0x98,  10, 0x0056 },  //  0001010110
    { 0x99,  10, 0x0055 },  //  0001010101
    { 0x9A,  10, 0x0054 },  //  0001010100
    { 0x9B,  10, 0x0053 },  //  0001010011
    { 0x9C,  10, 0x0052 },  //  0001010010
    { 0x9D,  10, 0x0051 },  //  0001010001
    { 0x9E,  10, 0x0050 },  //  0001010000
    { 0x9F,  10, 0x004F },  //  0001001111
    { 0xA0,  10, 0x004E },  //  0001001110
    { 0xA1,  10, 0x004D },  //  0001001101
    { 0xA2,  10, 0x004C },  //  0001001100
    { 0xA3,  10, 0x004B },  //  0001001011
    { 0xA4,  10, 0x004A },  //  0001001010
    { 0xA5,  10, 0x0049 },  //  0001001001
    { 0xA6,  10, 0x0048 },  //  0001001000
    { 0xA7,  10, 0x0047 },  //  0001000111
    { 0xA8,  10, 0x0046 },  //  0001000110
    { 0xA9,  10, 0x0045 },  //  0001000101
    { 0xAA,  10, 0x0044 },  //  0001000100
    { 0xAB,  10, 0x0043 },  //  0001000011
    { 0xAC,  10, 0x0042 },  //  0001000010
    { 0xAD,  10, 0x0041 },  //  0001000001
    { 0xAE,  10, 0x0040 },  //  0001000000
    { 0xAF,  10, 0x003F },  //  0000111111
    { 0xB0,  10, 0x003E },  //  0000111110
    { 0xB1,  10, 0x003D },  //  0000111101
    { 0xB2,  10, 0x003C },  //  0000111100
    { 0xB3,  10, 0x003B },  //  0000111011
    { 0xB4,  10, 0x003A },  //  0000111010
    { 0xB5,  10, 0x0039 },  //  0000111001
    { 0xB6,  10, 0x0038 },  //  0000111000
    { 0xB7,  10, 0x0037 },  //  0000110111
    { 0xB8,  10, 0x0036 },  //  0000110110
    { 0xB9,  10, 0x0035 },  //  0000110101
    { 0xBA,  10, 0x0034 },  //  0000110100
    { 0xBB,  10, 0x0033 },  //  0000110011
    { 0xBC,  10, 0x0032 },  //  0000110010
    { 0xBD,  10, 0x0031 },  //  0000110001
    { 0xBE,  10, 0x0030 },  //  0000110000
    { 0xBF,  10, 0x002F },  //  0000101111
    { 0xC0,  10, 0x002E },  //  0000101110
    { 0xC1,  10, 0x002D },  //  0000101101
    { 0xC2,  10, 0x002C },  //  0000101100
    { 0xC3,  10, 0x002B },  //  0000101011
    { 0xC4,  10, 0x002A },  //  0000101010
    { 0xC5,  10, 0x0029 },  //  0000101001
    { 0xC6,  10, 0x0028 },  //  0000101000
    { 0xC7,  10, 0x0027 },  //  0000100111
    { 0xC8,  10, 0x0026 },  //  0000100110
    { 0xC9,  10, 0x0025 },  //  0000100101
    { 0xCA,  10, 0x0024 },  //  0000100100
    { 0xCB,  10, 0x0023 },  //  0000100011
    { 0xCC,  10, 0x0022 },  //  0000100010
    { 0xCD,  10, 0x0021 },  //  0000100001
    { 0xCE,  10, 0x0020 },  //  0000100000
    { 0xCF,  10, 0x001F },  //  0000011111
    { 0xD0,  10, 0x001E },  //  0000011110
    { 0xD1,  10, 0x001D },  //  0000011101
    { 0xD2,  10, 0x001C },  //  0000011100
    { 0xD3,  10, 0x001B },  //  0000011011
    { 0xD4,  10, 0x001A },  //  0000011010
    { 0xD6,  10, 0x0019 },  //  0000011001
    { 0xD7,  10, 0x0018 },  //  0000011000
    { 0xD8,  10, 0x0017 },  //  0000010111
    { 0xD9,  10, 0x0016 },  //  0000010110
    { 0xDA,  10, 0x0015 },  //  0000010101
    { 0xDB,  10, 0x0014 },  //  0000010100
    { 0xDC,  10, 0x0013 },  //  0000010011
    { 0xDE,  10, 0x0012 },  //  0000010010
    { 0xDF,  10, 0x0011 },  //  0000010001
    { 0xE1,  10, 0x0010 },  //  0000010000
    { 0xE2,  10, 0x000F },  //  0000001111
    { 0xE4,  10, 0x000E },  //  0000001110
    { 0xEF,  10, 0x000D },  //  0000001101
    { 0xD5,  11, 0x0019 },  //  00000011001
    { 0xDD,  11, 0x0018 },  //  00000011000
    { 0xE3,  11, 0x0017 },  //  00000010111
    { 0xE5,  11, 0x0016 },  //  00000010110
    { 0xE6,  11, 0x0015 },  //  00000010101
    { 0xE7,  11, 0x0014 },  //  00000010100
    { 0xE8,  11, 0x0013 },  //  00000010011
    { 0xE9,  11, 0x0012 },  //  00000010010
    { 0xEA,  11, 0x0011 },  //  00000010001
    { 0xEB,  11, 0x0010 },  //  00000010000
    { 0xEC,  11, 0x000F },  //  00000001111
    { 0xED,  11, 0x000E },  //  00000001110
    { 0xEE,  11, 0x000D },  //  00000001101
    { 0xF2,  11, 0x000C },  //  00000001100
    { 0xF3,  11, 0x000B },  //  00000001011
    { 0xF4,  11, 0x000A },  //  00000001010
    { 0xF5,  11, 0x0009 },  //  00000001001
    { 0xF6,  11, 0x0008 },  //  00000001000
    { 0xF7,  11, 0x0007 },  //  00000000111
    { 0xF8,  11, 0x0006 },  //  00000000110
    { 0xFA,  11, 0x0005 },  //  00000000101
    { 0xFB,  11, 0x0004 },  //  00000000100
    { 0xFC,  11, 0x0003 },  //  00000000011
    { 0xFD,  11, 0x0002 },  //  00000000010
    { 0xFE,  11, 0x0001 },  //  00000000001
    { 0xF9,  12, 0x0001 },  //  000000000001
    { HUFFMAN_EOF,  12, 0x0000 },  //  000000000000
};

int huffManLenIndex[HUFFMAN_TREE_SIZE];

void huffmanInitDecodeLenIndex(void)
{
    // create an index of first code at each possible length
    for (int ii = 0; ii < HUFFMAN_TREE_SIZE; ++ii) {
        huffManLenIndex[ii] = -1;
    }
    for (int ii = 0; ii < HUFFMAN_TREE_SIZE; ++ii) {
        if (huffManLenIndex[huffmanTree[ii].codeLen] == -1) {
            huffManLenIndex[huffmanTree[ii].codeLen] = ii;
        }
    }
}

int huffmanDecodeBuf(uint8_t *outBuf, int outBufLen, const uint8_t *inBuf, int inBufLen, int inBufCharacterCount, const huffmanTree_t *huffmanTree)
{
    static bool initialized = false;
    if (!initialized) {
        huffmanInitDecodeLenIndex();
        initialized = true;
    }

    if (inBufCharacterCount > outBufLen) {
        return -1;
    }
    uint16_t code = 0;
    int codeLen = 0;
    int outCount = 0;
    int inCount = 0;
    uint8_t testBit = 0x80;
    bool eof = false;
    while (!eof && outCount < outBufLen && inCount < inBufLen) {
        if (outCount == inBufCharacterCount) {
            // we've exhausted the input stream, discard any odd bits on the end
            return outCount;
        }
        if (inCount >= inBufLen) {
            return -1;
        }
        // get the next bit from the input buffer
        code <<= 1;
        ++codeLen;
        if (*inBuf & testBit) {
            code |= 0x01;
        }
        testBit >>= 1;
        if (testBit == 0) {
            testBit = 0x80;
            ++inBuf;
            ++inCount;
        }
        // check if the code is a leaf node or an interior node
        if (huffManLenIndex[codeLen] != -1) {
            // look for the code in the tree, only leaf nodes are stored in the tree
            for(int ii = huffManLenIndex[codeLen]; (ii < HUFFMAN_TREE_SIZE) && (huffmanTree[ii].codeLen == codeLen); ++ii) {
                if (huffmanTree[ii].code == code) {
                    // we've found the code, so it is a leaf node
                    const int16_t value = huffmanTree[ii].value;
                    if (value == HUFFMAN_EOF) {
                        eof = true;
                    } else {
                        // output the value
                        *outBuf = (uint8_t)value;
                        ++outBuf;
                        ++outCount;
                    }
                    // reset the code to continue decompressing the input buffer
                    code = 0;
                    codeLen = 0;
                    break;
                }
            }
        }
    }
    return outCount;
}

TEST(HuffmanUnittest, TestHuffmanEncode)
{
    #define INBUF_LEN1 3
    const uint8_t inBuf1[INBUF_LEN1] = {0,1,1};
    // 11 101 101
    // 1110 1101
    // e    d
    int len = huffmanEncodeBuf(outBuf, OUTBUF_LEN, inBuf1, INBUF_LEN1, huffmanTable);
    EXPECT_EQ(1, len);
    EXPECT_EQ(0xed, (int)outBuf[0]);

    #define INBUF_LEN2 4
    const uint8_t inBuf2[INBUF_LEN2] = {0,1,2,3};
    // 11 101 1001 10001
    // 1110 1100 1100 01
    // e    c    c    8
    len = huffmanEncodeBuf(outBuf, OUTBUF_LEN, inBuf2, INBUF_LEN2, huffmanTable);
    EXPECT_EQ(2, len);
    EXPECT_EQ(0xec, (int)outBuf[0]);
    EXPECT_EQ(0xc4, (int)outBuf[1]);

    #define INBUF_LEN3 8
    const uint8_t inBuf3[INBUF_LEN3] = {0,1,2,3,4,5,6,7};
    // 11 101 1001 10001 10000 011101 011100 011011
    // 1110 1100 1100 0110 0000 1110 1011 1000 1101 1
    // e    c    c    6    0    e    b    8    d    8
    len = huffmanEncodeBuf(outBuf, OUTBUF_LEN, inBuf3, INBUF_LEN3, huffmanTable);
    EXPECT_EQ(5, len);
    EXPECT_EQ(0xec, (int)outBuf[0]);
    EXPECT_EQ(0xc6, (int)outBuf[1]);
    EXPECT_EQ(0x0e, (int)outBuf[2]);
    EXPECT_EQ(0xb8, (int)outBuf[3]);
    EXPECT_EQ(0xd8, (int)outBuf[4]);
}

TEST(HuffmanUnittest, TestHuffmanEncodeStreaming)
{
    #define INBUF_LEN1  3
    #define INBUF_LEN1_CHUNK1 2
    #define INBUF_LEN1_CHUNK2 (INBUF_LEN1 - INBUF_LEN1_CHUNK1)
    const uint8_t inBuf1[INBUF_LEN1] = {0,1,1};
    // 11 101 101
    // 1110 1101
    // e    d
    huffmanState_t state1 = {
        .bytesWritten = 0,
        .outByte = outBuf,
        .outBufLen = OUTBUF_LEN,
        .outBit = 0x80,
    };
    *state1.outByte = 0;
    int status = huffmanEncodeBufStreaming(&state1, inBuf1, INBUF_LEN1_CHUNK1, huffmanTable);
    EXPECT_EQ(0, status);
    status = huffmanEncodeBufStreaming(&state1, inBuf1 + INBUF_LEN1_CHUNK1, INBUF_LEN1_CHUNK2, huffmanTable);
    EXPECT_EQ(0, status);
    if (state1.outBit != 0x80) {
        ++state1.bytesWritten;
    }

    EXPECT_EQ(1, state1.bytesWritten);
    EXPECT_EQ(0xed, (int)outBuf[0]);

    #define INBUF_LEN2 4
    #define INBUF_LEN2_CHUNK1 1
    #define INBUF_LEN2_CHUNK2 1
    #define INBUF_LEN2_CHUNK3 2
    const uint8_t inBuf2[INBUF_LEN2] = {0,1,2,3};
    // 11 101 1001 10001
    // 1110 1100 1100 01
    // e    c    c    8
    huffmanState_t state2 = {
        .bytesWritten = 0,
        .outByte = outBuf,
        .outBufLen = OUTBUF_LEN,
        .outBit = 0x80,
    };
    *state2.outByte = 0;
    status = huffmanEncodeBufStreaming(&state2, inBuf2, INBUF_LEN2_CHUNK1, huffmanTable);
    EXPECT_EQ(0, status);
    status = huffmanEncodeBufStreaming(&state2, inBuf2 + INBUF_LEN2_CHUNK1, INBUF_LEN2_CHUNK2, huffmanTable);
    EXPECT_EQ(0, status);
    status = huffmanEncodeBufStreaming(&state2, inBuf2 + INBUF_LEN2_CHUNK1 + INBUF_LEN2_CHUNK2, INBUF_LEN2_CHUNK3, huffmanTable);
    EXPECT_EQ(0, status);
    if (state2.outBit != 0x80) {
        ++state2.bytesWritten;
    }

    EXPECT_EQ(2, state2.bytesWritten);
    EXPECT_EQ(0xec, (int)outBuf[0]);
    EXPECT_EQ(0xc4, (int)outBuf[1]);

    #define INBUF_LEN3 8
    #define INBUF_LEN3_CHUNK1 4
    #define INBUF_LEN3_CHUNK2 (INBUF_LEN3 - INBUF_LEN3_CHUNK1)
    const uint8_t inBuf3[INBUF_LEN3] = {0,1,2,3,4,5,6,7};
    // 11 101 1001 10001 10000 011101 011100 011011
    // 1110 1100 1100 0110 0000 1110 1011 1000 1101 1
    // e    c    c    6    0    e    b    8    d    8
    huffmanState_t state3 = {
        .bytesWritten = 0,
        .outByte = outBuf,
        .outBufLen = OUTBUF_LEN,
        .outBit = 0x80,
    };
    *state3.outByte = 0;
    status = huffmanEncodeBufStreaming(&state3, inBuf3, INBUF_LEN3_CHUNK1, huffmanTable);
    EXPECT_EQ(0, status);
    status = huffmanEncodeBufStreaming(&state3, inBuf3 + INBUF_LEN3_CHUNK1, INBUF_LEN3_CHUNK2, huffmanTable);
    EXPECT_EQ(0, status);
    if (state3.outBit != 0x80) {
        ++state3.bytesWritten;
    }

    EXPECT_EQ(5, state3.bytesWritten);
    EXPECT_EQ(0xec, (int)outBuf[0]);
    EXPECT_EQ(0xc6, (int)outBuf[1]);
    EXPECT_EQ(0x0e, (int)outBuf[2]);
    EXPECT_EQ(0xb8, (int)outBuf[3]);
    EXPECT_EQ(0xd8, (int)outBuf[4]);
}

TEST(HuffmanUnittest, TestHuffmanDecode)
{
    int len;

    #define HUFF_BUF_LEN1 1
    #define HUFF_BUF_COUNT1 1
    const uint8_t inBuf1[HUFF_BUF_LEN1] = {0xc0}; // 11
    len = huffmanDecodeBuf(outBuf, OUTBUF_LEN, inBuf1, HUFF_BUF_LEN1, HUFF_BUF_COUNT1, huffmanTree);
    EXPECT_EQ(1, len);
    EXPECT_EQ(0x00, (int)outBuf[0]);
    EXPECT_EQ(-1, huffManLenIndex[0]);
    EXPECT_EQ(-1, huffManLenIndex[1]);
    EXPECT_EQ(0, huffManLenIndex[2]);
    EXPECT_EQ(1, huffManLenIndex[3]);
    EXPECT_EQ(2, huffManLenIndex[4]);
    EXPECT_EQ(3, huffManLenIndex[5]);
    EXPECT_EQ(6, huffManLenIndex[6]);
    EXPECT_EQ(11, huffManLenIndex[7]);

    #define HUFF_BUF_LEN2 1
    #define HUFF_BUF_COUNT2 3
    const uint8_t inBuf2[HUFF_BUF_LEN2] = {0xed}; // 11 101 101
    len = huffmanDecodeBuf(outBuf, OUTBUF_LEN, inBuf2, HUFF_BUF_LEN2, HUFF_BUF_COUNT2, huffmanTree);
    EXPECT_EQ(3, len);
    EXPECT_EQ(0x00, (int)outBuf[0]);
    EXPECT_EQ(0x01, (int)outBuf[1]);
    EXPECT_EQ(0x01, (int)outBuf[2]);

    #define HUFF_BUF_LEN3 5
    #define HUFF_BUF_COUNT3 8
    const uint8_t inBuf3[HUFF_BUF_LEN3] = {0xec, 0xc6, 0x0e, 0xb8, 0xd8};
    len = huffmanDecodeBuf(outBuf, OUTBUF_LEN, inBuf3, HUFF_BUF_LEN3, HUFF_BUF_COUNT3, huffmanTree);
    EXPECT_EQ(8, len);
    EXPECT_EQ(0x00, (int)outBuf[0]);
    EXPECT_EQ(0x01, (int)outBuf[1]);
    EXPECT_EQ(0x02, (int)outBuf[2]);
    EXPECT_EQ(0x03, (int)outBuf[3]);
    EXPECT_EQ(0x04, (int)outBuf[4]);
    EXPECT_EQ(0x05, (int)outBuf[5]);
    EXPECT_EQ(0x06, (int)outBuf[6]);
    EXPECT_EQ(0x07, (int)outBuf[7]);
}

// STUBS

extern "C" {
}
