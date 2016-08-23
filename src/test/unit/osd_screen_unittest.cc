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

    #include <platform.h>
    #include "drivers/adc.h"
    #include "drivers/serial.h"
    #include "drivers/video_textscreen.h"
    #include "osd/fc_state.h"
    #include "osd/osd_element.h"
    #include "osd/osd_element_render.h"
    #include "osd/osd_screen.h"

    TEXT_SCREEN_CHAR textScreenBuffer[TEST_SCREEN_CHARACTER_COUNT]; // PAL has more characters than NTSC.

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

    int32_t mAhDrawn;
    int32_t amperage;
    uint16_t vbat;

    uint16_t testAdcChannels[ADC_CHANNEL_COUNT];

    #define TEST_VOLTAGE_0  0
    uint16_t testVoltages[1];

    fcStatus_t fcStatus;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define TEST_INITIAL_CHARACTER (TEXT_SCREEN_CHAR)(0xFF)

class OsdScreenTest : public ::testing::Test {
protected:

    virtual void SetUp() {
        memset(textScreenBuffer, TEST_INITIAL_CHARACTER, sizeof(textScreenBuffer));
        memset(testVoltages, 0, sizeof(testVoltages));
        memset(testAdcChannels, 0, sizeof(testAdcChannels));

        osdTextScreen.height = TEST_ROW_COUNT;
        osdTextScreen.width = TEST_COLUMN_COUNT;
    }
};

#define SCREEN_BUFFER_OFFSET(x, y) (((y) * TEST_COLUMN_COUNT) + (x))

TEST_F(OsdScreenTest, TestClearScreen)
{
    // given
    uint8_t expectedMappedSpaceCharacter = font_test_asciiToFontMapping[(uint8_t)' '];
    // when
    osdClearScreen();

    // then
    for (int y = 0; y < TEST_ROW_COUNT; y++) {
        for (int x = 0; x < TEST_COLUMN_COUNT; x++) {

            int offset = SCREEN_BUFFER_OFFSET(x,y);

            EXPECT_EQ((TEXT_SCREEN_CHAR)expectedMappedSpaceCharacter, textScreenBuffer[offset]);
        }
    }
}

TEST_F(OsdScreenTest, TestTopLeftCharacter)
{
    // given
    char testChar = ' ';
    uint8_t expectedMappedChar = font_test_asciiToFontMapping[(uint8_t)testChar];

    // when
    osdSetCharacterAtPosition(0, 0, testChar);

    // then
    int offset = SCREEN_BUFFER_OFFSET(0, 0);
    EXPECT_EQ((TEXT_SCREEN_CHAR)expectedMappedChar, textScreenBuffer[offset]);
}

TEST_F(OsdScreenTest, TestBottomRightCharacter)
{
    // given
    char testChar = ' ';
    uint8_t expectedMappedChar = font_test_asciiToFontMapping[(uint8_t)testChar];

    // when
    osdSetCharacterAtPosition(TEST_COLUMN_COUNT - 1, -1, testChar); // -1 on Y axis indicates bottom justified.

    // then
    int offset = SCREEN_BUFFER_OFFSET(TEST_COLUMN_COUNT - 1, TEST_ROW_COUNT - 1);
    EXPECT_EQ((TEXT_SCREEN_CHAR)expectedMappedChar, textScreenBuffer[offset]);
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

void expectUnmodifiedScreen(void)
{
    for (int y = 0; y < TEST_ROW_COUNT; y++) {
        for (int x = 0; x < TEST_COLUMN_COUNT; x++) {

            int offset = SCREEN_BUFFER_OFFSET(x,y);

            EXPECT_EQ(TEST_INITIAL_CHARACTER, textScreenBuffer[offset]);
        }
    }
}

uint32_t testMillis;

TEST_F(OsdScreenTest, TestOsdElement_OnDuration)
{
    // given
    testMillis = ((12 * 1000) * 60) + (34 * 1000);
    element_t element = {
        0, 0, true, OSD_ELEMENT_ON_DURATION
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "12:34";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_ArmedDuration)
{
    // given
    fcStatus.armedDuration = ((43 * 1000) * 60) + (21 * 1000);
    element_t element = {
        0, 0, true, OSD_ELEMENT_ARMED_DURATION
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "43:21";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}


TEST_F(OsdScreenTest, TestOsdElement_MahDrawn)
{
    // given
    mAhDrawn = 99999;

    element_t element = {
        0, 0, true, OSD_ELEMENT_MAH_DRAWN
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "MAH: 99999";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}


TEST_F(OsdScreenTest, TestOsdElement_Amperage)
{
    // given
    amperage = 9876;

    element_t element = {
        0, 0, true, OSD_ELEMENT_AMPERAGE
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "AMP:98.76A";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}


TEST_F(OsdScreenTest, TestOsdElement_Voltage5V)
{
    // given
    testAdcChannels[ADC_POWER_5V] = TEST_VOLTAGE_0;
    testVoltages[TEST_VOLTAGE_0] = 51;

    element_t element = {
        0, 0, true, OSD_ELEMENT_VOLTAGE_5V
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = " 5V:  5.1V";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_Voltage12V)
{
    // given
    testAdcChannels[ADC_POWER_12V] = TEST_VOLTAGE_0;
    testVoltages[TEST_VOLTAGE_0] = 126;

    element_t element = {
        0, 0, true, OSD_ELEMENT_VOLTAGE_12V
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "12V: 12.6V";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_VoltagBattery_FC)
{
    // given
    fcStatus.vbat = 168;

    element_t element = {
        0, 0, true, OSD_ELEMENT_VOLTAGE_BATTERY_FC
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = " FC: 16.8V";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_VoltageBattery)
{
    // given
    vbat = 168;

    element_t element = {
        0, 0, true, OSD_ELEMENT_VOLTAGE_BATTERY
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "BAT: 16.8V";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_FlightMode_Horizon)
{
    // given
    fcStatus.fcState = (1 << FC_STATE_HORIZON);

    element_t element = {
        0, 0, true, OSD_ELEMENT_FLIGHT_MODE
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "HRZN";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_FlightMode_Angle)
{
    // given
    fcStatus.fcState = (1 << FC_STATE_ANGLE);

    element_t element = {
        0, 0, true, OSD_ELEMENT_FLIGHT_MODE
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "ANGL";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_FlightMode_Acro)
{
    // given
    fcStatus.fcState = 0; // no mode flag exists for acro, it's the default mode when ANGLE and HORIZON are OFF.

    element_t element = {
        0, 0, true, OSD_ELEMENT_FLIGHT_MODE
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "ACRO";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_Indicator_Baro_On)
{
    // given
    fcStatus.fcState = (1 << FC_STATE_BARO);

    element_t element = {
        0, 0, true, OSD_ELEMENT_INDICATOR_BARO
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "B";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_Indicator_Baro_Off)
{
    // given
    fcStatus.fcState = 0;

    element_t element = {
        0, 0, true, OSD_ELEMENT_INDICATOR_BARO
    };

    // when
    osdDrawTextElement(&element);

    // then
    expectUnmodifiedScreen();
}

TEST_F(OsdScreenTest, TestOsdElement_Indicator_Mag_On)
{
    // given
    fcStatus.fcState = (1 << FC_STATE_MAG);

    element_t element = {
        0, 0, true, OSD_ELEMENT_INDICATOR_MAG
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "M";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}

TEST_F(OsdScreenTest, TestOsdElement_Indicator_Mag_Off)
{
    // given
    fcStatus.fcState = 0;

    element_t element = {
        0, 0, true, OSD_ELEMENT_INDICATOR_MAG
    };

    // when
    osdDrawTextElement(&element);

    // then
    expectUnmodifiedScreen();
}

TEST_F(OsdScreenTest, TestOsdElement_RSSIFC)
{
    // given
    fcStatus.rssi = 1000;  // 0.1% steps

    element_t element = {
        0, 0, true, OSD_ELEMENT_RSSI_FC
    };

    // when
    osdDrawTextElement(&element);

    // then
    char expectedAscii[] = "RSSI:100";
    uint8_t *expectedContent = asciiToFontMap(expectedAscii);

    compareScreen(0, 0, expectedContent, strlen(expectedAscii));
}



// STUBS
extern "C" {
    uint16_t adcGetChannel(uint8_t channel) {
        return testAdcChannels[channel];
    }

    uint16_t batteryAdcToVoltage(uint16_t src) {
        return testVoltages[src];
    }

    uint32_t millis(void) { return testMillis; }
    bool isSerialTransmitBufferEmpty(serialPort_t *) { return true; }
    void serialWrite(serialPort_t *, uint8_t ) {};


}
