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

extern "C" unsigned int testFlashfsReadLength(uint32_t volumeSize, uint32_t address, unsigned int requestedLength);

#include "gtest/gtest.h"

TEST(FlashfsBoundsTest, PreservesReadInsideVolume)
{
    EXPECT_EQ(16U, testFlashfsReadLength(100, 20, 16));
}

TEST(FlashfsBoundsTest, TruncatesReadAtEndOfVolume)
{
    EXPECT_EQ(8U, testFlashfsReadLength(100, 92, 16));
}

TEST(FlashfsBoundsTest, RejectsReadAtOrBeyondEndOfVolume)
{
    EXPECT_EQ(0U, testFlashfsReadLength(100, 100, 16));
    EXPECT_EQ(0U, testFlashfsReadLength(100, 101, 16));
    EXPECT_EQ(0U, testFlashfsReadLength(100, UINT32_MAX, 16));
}
