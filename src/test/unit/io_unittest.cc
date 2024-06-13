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
    #include "drivers/io.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// test macro te generate tag, using io internals
#define TAG_MAKE(port, bit) DEFIO_TAG_MAKE(DEFIO_GPIOID__ ## port, (bit))
#define TAG_NONE 0

TEST(IOUnittest, TestSanity)
{
//test TAG_MAKE
    EXPECT_EQ(TAG_MAKE(A, 1), TAG_MAKE(A,1));
    EXPECT_EQ(TAG_MAKE(A, 0),  ((0 + 1) << 4) |  0);
    EXPECT_EQ(TAG_MAKE(A, 15), ((0 + 1) << 4) | 15);
    EXPECT_EQ(TAG_MAKE(I, 0),  ((8 + 1) << 4) |  0);
    EXPECT_EQ(TAG_MAKE(I, 15), ((8 + 1) << 4) | 15);
}


TEST(IOUnittest, TestIO_TAG)
{
#define TESTTAG_NONE NONE
#define TESTTAG_PA0  PA0
#define TESTTAG_PA1  PA1
#define TESTTAG_EMPTY

    // NONE must be false
    EXPECT_EQ(IO_TAG_FIRST(NONE), 0);
    // direct invocation
    EXPECT_EQ(IO_TAG_FIRST(PA1), TAG_MAKE(A, 1));
    // passing #define to macro
    EXPECT_EQ(IO_TAG_FIRST(TESTTAG_NONE), TAG_NONE);
    EXPECT_EQ(IO_TAG_FIRST(TESTTAG_PA0), TAG_MAKE(A, 0));
    // with multiple arguments, use first tag
    EXPECT_EQ(IO_TAG_FIRST(NONE, PA1), TAG_NONE);
    EXPECT_EQ(IO_TAG_FIRST(PA1, PA2, PA3),  TAG_MAKE(A, 1));
    // now with #defines
    EXPECT_EQ(IO_TAG_FIRST(TESTTAG_NONE, TESTTAG_PA0), TAG_NONE);
    EXPECT_EQ(IO_TAG_FIRST(TESTTAG_PA0, TESTTAG_PA1),  TAG_MAKE(A, 0));
    // undefined macro as argument is ignored
    EXPECT_EQ(IO_TAG_FIRST(TESTTAG_UNDEFINED, TESTTAG_NONE, TESTTAG_PA0), TAG_NONE);
    EXPECT_EQ(IO_TAG_FIRST(TESTTAG_UNDEFINED, TESTTAG_PA0, TESTTAG_PA1),  TAG_MAKE(A, 0));
    // empty macro is ingored too
    EXPECT_EQ(IO_TAG_FIRST(TESTTAG_EMPTY, TESTTAG_NONE, TESTTAG_PA0), TAG_NONE);
    EXPECT_EQ(IO_TAG_FIRST(TESTTAG_EMPTY, TESTTAG_PA0, TESTTAG_PA1),  TAG_MAKE(A, 0));
    // test io not defined for target (PD is not defined for unittests)
    {
        // prevent undefined variable in test, normal compilation will result in undefined variable 
        int DEFIO_UNSUPPORTED_I(D, 1, TARGET_MCU) = 1234;
        EXPECT_EQ(IO_TAG_FIRST(PD1),  1234);
    }
    // test undefined IO that is not used
    EXPECT_EQ(IO_TAG_FIRST(PA0, PD1),  TAG_MAKE(A, 0));
    // if no valid arg is passed, generate error
    {
        // prevent undefined variable in test, normal compilation will result in undefined variable 
        int DEFIO_TAG__NOARGVALID = 1234;
        EXPECT_EQ(IO_TAG_FIRST(TESTTAG_EMPTY, TESTTAG_UNDEFINED),  1234);
    }
}

// STUBS

extern "C" {
}
