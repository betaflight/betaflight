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
    #include "platform.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint8_t __config_start[8192] __attribute__((aligned(1024)));

struct MockFlash
{
    static const int Capacity = sizeof(__config_start);
    static const int PageSize = 1024;

    int64_t wroteTo;
    bool unlocked;

    int erases;
    int writes;

    uint8_t *data() { return __config_start; }
    void init()
    {
        memset(this, 0, sizeof(*this));
        memset(data(), 0xAE, Capacity);
    }
    int64_t toOffset(uint32_t address) const
    {
        int32_t offset = address - (uint32_t)(uintptr_t)__config_start;
        return offset;
    }
    bool inBounds(uint32_t address, int count) const
    {
        auto offset = toOffset(address);
        return offset >= 0 && (offset + count) < Capacity;
    }

    FLASH_Status erase(uint32_t address)
    {
        EXPECT_TRUE(unlocked);
        EXPECT_EQ(0, address % PageSize);
        EXPECT_TRUE(inBounds(address, PageSize));
        memset(data() + toOffset(address), 0xFF, PageSize);
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
            data()[offset + i] = (uint8_t)value;
            value >>= 8;
        }
        wroteTo = std::max(wroteTo, (int64_t)(offset + sizeof(value)));
        writes++;

        return FLASH_COMPLETE;
    }
};

MockFlash mockFlash;

void FLASH_Unlock(void)
{
    EXPECT_FALSE(mockFlash.unlocked);
    mockFlash.unlocked = true;
}

void FLASH_Lock(void)
{
    EXPECT_TRUE(mockFlash.unlocked);
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
