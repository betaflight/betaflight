/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

extern "C" {
    #include "common/uvarint.h"
    #include "common/utils.h"
}

#include "gtest/gtest.h"

typedef struct uvarintExpectation_s {
    uint32_t value;
    int length;
    uint8_t bytes[5];
} uvarintExpectation_t;

// The uvarint format of the FrSky OSD protocol: each byte holds seven bits
// of the value, the least significant group first. Bit 7 is set in each
// byte except the last byte.
static const uvarintExpectation_t expectations[] = {
    {          0, 1, {0x00}},
    {          1, 1, {0x01}},
    {        127, 1, {0x7F}},
    {        128, 2, {0x80, 0x01}},
    {        129, 2, {0x81, 0x01}},
    {        255, 2, {0xFF, 0x01}},
    {      16383, 2, {0xFF, 0x7F}},
    {      16384, 3, {0x80, 0x80, 0x01}},
    {      16511, 3, {0xFF, 0x80, 0x01}},
    {    2097152, 4, {0x80, 0x80, 0x80, 0x01}},
    { 0xFFFFFFFF, 5, {0xFF, 0xFF, 0xFF, 0xFF, 0x0F}},
};

TEST(UvarintTest, EncodeTest)
{
    for (unsigned i = 0; i < ARRAYLEN(expectations); i++) {
        const uvarintExpectation_t *expectation = &expectations[i];
        uint8_t buf[5] = {};

        // expect
        EXPECT_EQ(expectation->length, uvarintEncode(expectation->value, buf, sizeof(buf))) << "value " << expectation->value;
        for (int j = 0; j < expectation->length; j++) {
            EXPECT_EQ(expectation->bytes[j], buf[j]) << "value " << expectation->value << " byte " << j;
        }
    }
}

TEST(UvarintTest, EncodeBufferTooSmallTest)
{
    for (unsigned i = 0; i < ARRAYLEN(expectations); i++) {
        const uvarintExpectation_t *expectation = &expectations[i];
        uint8_t buf[5] = {};

        // expect
        // the buffer is one byte too small for the value
        EXPECT_EQ(-1, uvarintEncode(expectation->value, buf, expectation->length - 1)) << "value " << expectation->value;
    }
}

TEST(UvarintTest, RoundTripTest)
{
    for (unsigned i = 0; i < ARRAYLEN(expectations); i++) {
        const uvarintExpectation_t *expectation = &expectations[i];
        uint8_t buf[5] = {};
        uint32_t decoded = 0;

        // given
        const int length = uvarintEncode(expectation->value, buf, sizeof(buf));

        // expect
        EXPECT_EQ(length, uvarintDecode(&decoded, buf, length)) << "value " << expectation->value;
        EXPECT_EQ(expectation->value, decoded);
    }
}
