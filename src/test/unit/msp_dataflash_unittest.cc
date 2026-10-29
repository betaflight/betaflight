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

#include <cstring>

extern "C" {
#include "platform.h"

#include "common/streambuf.h"
#include "io/flashfs.h"

void testSerializeDataflashReadReply(sbuf_t *dst, uint32_t address, uint16_t size, bool allowCompression);

static const uint32_t volumeSize = 4096;
static unsigned readCalls;
static unsigned goodReadsLeft;

uint32_t flashfsGetSize(void) { return volumeSize; }

int flashfsReadAbs(uint32_t address, uint8_t *buffer, unsigned int length)
{
    UNUSED(address);
    // a bounded stub: an unfixed caller keeps calling, the guard turns that into a failure
    if (++readCalls > 1000) {
        throw readCalls;
    }
    if (goodReadsLeft == 0) {
        return 0;
    }
    --goodReadsLeft;
    memset(buffer, 0, length);
    return length;
}
}

#include "gtest/gtest.h"

static uint8_t replyBuffer[512];

static void serialize(bool allowCompression)
{
    sbuf_t dst;
    sbufInit(&dst, replyBuffer, replyBuffer + sizeof(replyBuffer));
    testSerializeDataflashReadReply(&dst, 0, 128, allowCompression);
}

TEST(MspDataflashReadTest, HuffmanReadReturnsWhenFirstReadFails)
{
    readCalls = 0;
    goodReadsLeft = 0;
    EXPECT_NO_THROW(serialize(true));
    EXPECT_EQ(1u, readCalls);
}

TEST(MspDataflashReadTest, HuffmanReadReturnsWhenLaterReadFails)
{
    readCalls = 0;
    goodReadsLeft = 1;
    EXPECT_NO_THROW(serialize(true));
    EXPECT_EQ(2u, readCalls);
}

TEST(MspDataflashReadTest, HuffmanReadFinishesWhileReadsSucceed)
{
    readCalls = 0;
    goodReadsLeft = 1000;
    EXPECT_NO_THROW(serialize(true));
    EXPECT_GT(readCalls, 0u);
}

TEST(MspDataflashReadTest, UncompressedReadReturnsWhenReadFails)
{
    readCalls = 0;
    goodReadsLeft = 0;
    EXPECT_NO_THROW(serialize(false));
    EXPECT_EQ(1u, readCalls);
}
