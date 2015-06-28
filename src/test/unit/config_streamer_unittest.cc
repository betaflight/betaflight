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
    #include "config/config_streamer.h"
    #include "platform.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

struct MockFlash
{
    static const uintptr_t Base = 0x08000000;
    static const int PageSize = 1024;

    uint8_t data[PageSize*8];
    int64_t wroteTo;
    bool unlocked;

    int erases;
    int writes;

    void init() { memset(this, 0, sizeof(*this)); memset(data, 0xAE, sizeof(data)); }
    int64_t toOffset(uint32_t address) const { return (int64_t)address - Base; }
    bool inBounds(uint32_t address, int count) const
    {
        auto offset = toOffset(address);
        return offset >= 0 && (offset + count) < (int)sizeof(data);
    }

    FLASH_Status erase(uint32_t address)
    {
        EXPECT_TRUE(unlocked);
        EXPECT_EQ(0, address % PageSize);
        EXPECT_TRUE(inBounds(address, PageSize));
        erases++;

        return FLASH_COMPLETE;
    }

    FLASH_Status program(uint32_t address, uint32_t value)
    {
        EXPECT_TRUE(unlocked);
        EXPECT_EQ(0, address % sizeof(value));
        EXPECT_TRUE(inBounds(address, sizeof(value)));

        auto offset = toOffset(address);

        for (uint i = 0; i < sizeof(value); i++) {
            data[offset + i] = (uint8_t)value;
            value >>= 8;
        }
        wroteTo = std::max(wroteTo, (int64_t)(offset + sizeof(value)));
        writes++;

        return FLASH_COMPLETE;
    }
};

static MockFlash mockFlash;

void FLASH_Unlock(void)
{
    EXPECT_EQ(false, mockFlash.unlocked);
    mockFlash.unlocked = true;
}

void FLASH_Lock(void)
{
    EXPECT_EQ(true, mockFlash.unlocked);
    mockFlash.unlocked = false;
}

FLASH_Status FLASH_ErasePage(uint32_t address)
{
    return mockFlash.erase(address);
}

FLASH_Status FLASH_ProgramWord(uint32_t address, uint32_t data)
{
    return mockFlash.program(address, data);
}

config_streamer_t setup()
{
    mockFlash.init();

    config_streamer_t c;
    config_streamer_init(&c);
    config_streamer_start(&c, 0x08000000);

    return c;
}

TEST(configStreamerTest, TestInit)
{
    mockFlash.init();

    config_streamer_t c;
    config_streamer_init(&c);
}

TEST(configStreamerTest, TestWriteSmall)
{
    config_streamer_t c = setup();

    uint8_t ch = 0x12;
    EXPECT_EQ(0, config_streamer_write(&c, &ch, sizeof(ch)));
    EXPECT_EQ(0, config_streamer_finish(&c));
    EXPECT_EQ(ch, mockFlash.data[0]);

    EXPECT_EQ(1, mockFlash.erases);
    EXPECT_EQ(1, mockFlash.writes);
    EXPECT_EQ(4, mockFlash.wroteTo);
}

TEST(configStreamerTest, TestWriteCrossWord)
{
    config_streamer_t c = setup();

    uint8_t data[] = { 1, 2, 3, 4, 5 };
    EXPECT_EQ(0, config_streamer_write(&c, data, sizeof(data)));
    EXPECT_EQ(0, config_streamer_finish(&c));
    for (auto i = 0U; i < sizeof(data); i++) {
        EXPECT_EQ(data[i], mockFlash.data[i]);
    }

    EXPECT_EQ(1, mockFlash.erases);
    EXPECT_EQ(2, mockFlash.writes);
    EXPECT_EQ(8, mockFlash.wroteTo);
}

TEST(configStreamerTest, TestWriteWordMultiple)
{
    config_streamer_t c = setup();

    uint8_t data[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    EXPECT_EQ(0, config_streamer_write(&c, data, sizeof(data)));
    EXPECT_EQ(0, config_streamer_finish(&c));
    for (auto i = 0U; i < sizeof(data); i++) {
        EXPECT_EQ(data[i], mockFlash.data[i]);
    }

    EXPECT_EQ(1, mockFlash.erases);
    EXPECT_EQ(2, mockFlash.writes);
    EXPECT_EQ(8, mockFlash.wroteTo);
}

TEST(configStreamerTest, TestWriteThreeChunks)
{
    config_streamer_t c = setup();

    // Write the same data three times.  Expect appending.
    uint8_t data[] = { 1, 2, 3, 4, 5, 6, 7 };
    EXPECT_EQ(0, config_streamer_write(&c, data, sizeof(data)));
    EXPECT_EQ(0, config_streamer_write(&c, data, sizeof(data)));
    EXPECT_EQ(0, config_streamer_write(&c, data, sizeof(data)));
    EXPECT_EQ(0, config_streamer_finish(&c));

    for (auto i = 0U; i < sizeof(data); i++) {
        EXPECT_EQ(data[i], mockFlash.data[i + 0*sizeof(data)]);
        EXPECT_EQ(data[i], mockFlash.data[i + 1*sizeof(data)]);
        EXPECT_EQ(data[i], mockFlash.data[i + 2*sizeof(data)]);
    }

    EXPECT_EQ(1, mockFlash.erases);
    EXPECT_EQ(6, mockFlash.writes);
    EXPECT_EQ(7*3+3, mockFlash.wroteTo);
}
