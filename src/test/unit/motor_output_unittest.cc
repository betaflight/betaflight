/*
 * This file is part of Betaflight.
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
#include <iostream>

extern "C" {
    #include "drivers/dshot.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(MotorOutputUnittest, TestFixMotorOutputReordering)
{
    const unsigned size = 8;

    uint8_t  a1_initial[size] = {0, 1, 2, 3, 4, 5, 6, 7};
    uint8_t a1_expected[size] = {0, 1, 2, 3, 4, 5, 6, 7};
    validateAndfixMotorOutputReordering(a1_initial, size);
    EXPECT_TRUE( 0 == memcmp(a1_expected, a1_initial, sizeof(a1_expected)));

    uint8_t  a2_initial[size] = {3, 2, 1, 4, 5, 6, 7, 0};
    uint8_t a2_expected[size] = {3, 2, 1, 4, 5, 6, 7, 0};
    validateAndfixMotorOutputReordering(a2_initial, size);
    EXPECT_TRUE( 0 == memcmp(a2_expected, a2_initial, sizeof(a2_expected)));

    uint8_t  a3_initial[size] = {3, 2, 1, 100, 5, 6, 7, 0};
    uint8_t a3_expected[size] = {0, 1, 2, 3, 4, 5, 6, 7};
    validateAndfixMotorOutputReordering(a3_initial, size);
    EXPECT_TRUE( 0 == memcmp(a3_expected, a3_initial, sizeof(a3_expected)));

    uint8_t  a4_initial[size] = {3, 2, 1, 100, 5, 6, 200, 0};
    uint8_t a4_expected[size] = {0, 1, 2, 3, 4, 5, 6, 7};
    validateAndfixMotorOutputReordering(a4_initial, size);
    EXPECT_TRUE( 0 == memcmp(a4_expected, a4_initial, sizeof(a4_expected)));

    uint8_t  a5_initial[size] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t a5_expected[size] = {0, 1, 2, 3, 4, 5, 6, 7};
    validateAndfixMotorOutputReordering(a5_initial, size);
    EXPECT_TRUE( 0 == memcmp(a5_expected, a5_initial, sizeof(a5_expected)));

    uint8_t  a6_initial[size] = {0, 0, 0, 1, 0, 99, 0, 0};
    uint8_t a6_expected[size] = {0, 1, 2, 3, 4, 5, 6, 7};
    validateAndfixMotorOutputReordering(a6_initial, size);
    EXPECT_TRUE( 0 == memcmp(a6_expected, a6_initial, sizeof(a5_expected)));

    uint8_t  a7_initial[size] = {1, 5, 3, 4, 5, 6, 7, 0};
    uint8_t a7_expected[size] = {0, 1, 2, 3, 4, 5, 6, 7};
    validateAndfixMotorOutputReordering(a7_initial, size);
    EXPECT_TRUE( 0 == memcmp(a7_expected, a7_initial, sizeof(a7_expected)));

    uint8_t  a8_initial[size] = {1, 5, 1, 5, 3, 3, 3, 3};
    uint8_t a8_expected[size] = {0, 1, 2, 3, 4, 5, 6, 7};
    validateAndfixMotorOutputReordering(a8_initial, size);
    EXPECT_TRUE( 0 == memcmp(a8_expected, a8_initial, sizeof(a8_expected)));

    uint8_t  a9_initial[size] = {7, 6, 5, 4, 3, 2, 1, 0};
    uint8_t a9_expected[size] = {7, 6, 5, 4, 3, 2, 1, 0};
    validateAndfixMotorOutputReordering(a9_initial, size);
    EXPECT_TRUE( 0 == memcmp(a9_expected, a9_initial, sizeof(a9_expected)));
}
