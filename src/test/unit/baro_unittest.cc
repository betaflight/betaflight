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

int8_t ms5611_crc(uint16_t *prom);

}


#include "unittest_macros.h"
#include "gtest/gtest.h"


TEST(baroTest, TestValidMs5611Crc)
{

    // given
    uint16_t ms5611_prom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x450B};

    // when
    int8_t result = ms5611_crc(ms5611_prom);

    // then
    EXPECT_EQ(0, result);

}

TEST(baroTest, TestInvalidMs5611Crc)
{

    // given
    uint16_t ms5611_prom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500};

    // when
    int8_t result = ms5611_crc(ms5611_prom);

    // then
    EXPECT_EQ(-1, result);

}

TEST(baroTest, TestMs5611AllZeroProm)
{

    // given
    uint16_t ms5611_prom[] = {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};

    // when
    int8_t result = ms5611_crc(ms5611_prom);

    // then
    EXPECT_EQ(-1, result);

}

TEST(baroTest, TestMs5611AllOnesProm)
{

    // given
    uint16_t ms5611_prom[] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};

    // when
    int8_t result = ms5611_crc(ms5611_prom);

    // then
    EXPECT_EQ(-1, result);

}

// STUBS

extern "C" {

void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}
bool i2cWrite(uint8_t, uint8_t, uint8_t) {
    return 1;
}
bool i2cRead(uint8_t, uint8_t, uint8_t, uint8_t) {
    return 1;
}

}
