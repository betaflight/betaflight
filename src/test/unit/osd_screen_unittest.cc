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
#include <math.h>

#define TEST_ROW_COUNT 16
#define TEST_COLUMN_COUNT 30

#define TEST_SCREEN_CHARACTER_COUNT (TEST_ROW_COUNT * TEST_COLUMN_COUNT)

extern "C" {
    #include "build/build_config.h"
    #include "drivers/video_textscreen.h"
    #include "osd/osd_element.h"
    #include "osd/osd_element_render.h"
    #include "osd/osd_screen.h"

    char textScreenBuffer[TEST_SCREEN_CHARACTER_COUNT]; // PAL has more characters than NTSC.

    const uint8_t font_test_asciiToFontMapping[128] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //    0 -  15
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   16 -  31
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x00, //   32 -  47 " !"#$%&'()*+,-./"
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x00, 0x3c, 0x00, 0x3e, 0x00, //   48 -  63 "0123456789:;<=>?"
        0x00, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, //   64 -  79 "@ABCDEFGHIJKLMNO"
        0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, //   80 -  95 "PQRSTUVWXYZ[\]^_"
        0x00, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, //   96 - 111 "`abcdefghijklmno"
        0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00  //  112 - 127 "pqrstuvwxyz{|}~ "
    };

    const uint8_t *asciiToFontMapping = &font_test_asciiToFontMapping[0];

}

#include "unittest_macros.h"
#include "gtest/gtest.h"


class OsdScreenTest : public ::testing::Test {
protected:

    virtual void SetUp() {
        memset(textScreenBuffer, 0xFF, sizeof(textScreenBuffer));

        osdTextScreen.height = TEST_ROW_COUNT;
        osdTextScreen.width = TEST_COLUMN_COUNT;
    }
};

#define SCREEN_BUFFER_OFFSET(x, y) (((y) * TEST_COLUMN_COUNT) + (x))

TEST_F(OsdScreenTest, TestClearScreen)
{
    // given
    uint8_t expectedMappedSpaceCharacter = font_test_asciiToFontMapping[' '];
    // when
    osdClearScreen();

    // then
    for (int y = 0; y < TEST_ROW_COUNT; y++) {
        for (int x = 0; x < TEST_COLUMN_COUNT; x++) {

            int offset = SCREEN_BUFFER_OFFSET(x,y);

            EXPECT_EQ(expectedMappedSpaceCharacter, textScreenBuffer[offset]);
        }
    }
}

static uint8_t fontMapBuffer[256];

static uint8_t* asciiToFontMap(char *ascii) {
    int i = 0;
    do {
        fontMapBuffer[i++] = asciiToFontMapping[(uint8_t)*ascii++];
    } while (*ascii);

    return fontMapBuffer;
}

void compareScreen(uint8_t x, uint8_t y, uint8_t *content, uint8_t contentLength)
{
    for (uint8_t i = 0; i < contentLength; i++) {
        int offset = SCREEN_BUFFER_OFFSET(x + i, y);
        EXPECT_EQ(content[i], textScreenBuffer[offset]);
    }
}

uint32_t testMillis;

TEST_F(OsdScreenTest, TestOsdElementTime)
{
    // given
    testMillis = ((12 * 1000) * 60) + (34 * 1000);
    element_t timeElement = {
        0, 0, true, OSD_ELEMENT_ON_TIME
    };

    // when
    osdDrawTextElement(&timeElement);

    // then
    char expectedTime[] = "12:34";
    uint8_t *expectedMappedTime = asciiToFontMap(expectedTime);

    compareScreen(0, 0, expectedMappedTime, 5 );
}

// STUBS
extern "C" {
    uint32_t millis(void) { return testMillis; }
}

