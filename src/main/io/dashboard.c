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

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DASHBOARD

#include "build/version.h"
#include "build/build_config.h"

#include "drivers/time.h"
#include "drivers/display_ug2864hsweg01.h"

#include "cms/cms.h"

#include "common/printf.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/typeconversion.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"

#include "io/dashboard.h"
#include "io/displayport_oled.h"

#ifdef GPS
#include "io/gps.h"
#endif

#include "navigation/navigation.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"

#include "config/feature.h"


controlRateConfig_t *getControlRateConfig(uint8_t profileIndex);

#define MICROSECONDS_IN_A_SECOND (1000 * 1000)

#define DASHBOARD_UPDATE_FREQUENCY (MICROSECONDS_IN_A_SECOND / 5)
#define PAGE_CYCLE_FREQUENCY (MICROSECONDS_IN_A_SECOND * 5)

static timeUs_t nextDisplayUpdateAt = 0;
static bool displayPresent = false;

static displayPort_t *displayPort;

#define PAGE_TITLE_LINE_COUNT 1

static char lineBuffer[SCREEN_CHARACTER_COLUMN_COUNT + 1];

#define HALF_SCREEN_CHARACTER_COLUMN_COUNT (SCREEN_CHARACTER_COLUMN_COUNT / 2)
#define IS_SCREEN_CHARACTER_COLUMN_COUNT_ODD (SCREEN_CHARACTER_COLUMN_COUNT & 1)

#if defined(DASHBOARD_ARMED_BITMAP)
static uint8_t armedBitmapRLE [] = { 128, 64,
        '\x00','\x00','\x04','\x80','\x80','\x03','\xc0','\xc0', // 0x0008
        '\x02','\x80','\x80','\x03','\x00','\x00','\x0b','\x80', // 0x0010
        '\x80','\x02','\xc0','\xc0','\x02','\x80','\x80','\x03', // 0x0018
        '\x00','\x00','\x63','\xfc','\xfe','\xdf','\x03','\x01', // 0x0020
        '\x71','\x79','\xf9','\xf1','\xc1','\x83','\x8f','\xfe', // 0x0028
        '\xfc','\x00','\x00','\x04','\xf8','\xfe','\xff','\x07', // 0x0030
        '\x83','\xf1','\xf1','\x02','\x79','\x71','\x21','\x03', // 0x0038
        '\x87','\xfe','\xfc','\x70','\x00','\x00','\x07','\xff', // 0x0040
        '\xff','\x03','\x00','\x00','\x05','\xff','\xff','\x03', // 0x0048
        '\x3f','\x7c','\xf8','\xf0','\xc0','\x80','\x00','\x00', // 0x0050
        '\x0b','\xfe','\xff','\xff','\x03','\x00','\x00','\x0c', // 0x0058
        '\xc0','\xf0','\xfe','\x7f','\x0f','\x7f','\xfe','\xf8', // 0x0060
        '\xc0','\x00','\x00','\x06','\x07','\x1f','\xff','\xfc', // 0x0068
        '\xf0','\x80','\x00','\x00','\x0e','\xe0','\xf8','\xfe', // 0x0070
        '\x3f','\x0f','\x01','\x00','\x00','\x03','\x01','\x03', // 0x0078
        '\x07','\x0e','\x0e','\x02','\x0c','\x0c','\x03','\x0d', // 0x0080
        '\x0b','\x03','\xff','\xfe','\xfe','\x02','\x86','\x07', // 0x0088
        '\x07','\x02','\x87','\xce','\xfe','\xff','\x23','\x03', // 0x0090
        '\x0d','\x0c','\x0c','\x04','\x0e','\x07','\x03','\x01', // 0x0098
        '\x00','\x00','\x08','\xff','\xff','\x03','\x00','\x00', // 0x00a0
        '\x05','\xff','\xff','\x03','\xfe','\x00','\x00','\x02', // 0x00a8
        '\x03','\x07','\x1f','\x3e','\x7c','\xf8','\xe0','\xc0', // 0x00b0
        '\x00','\x00','\x06','\xff','\xff','\x04','\x00','\x00', // 0x00b8
        '\x09','\xe0','\xf8','\xff','\x3f','\x07','\x01','\x00', // 0x00c0
        '\x00','\x03','\x01','\x07','\x3f','\xff','\xf8','\xe0', // 0x00c8
        '\x80','\x00','\x00','\x05','\x03','\x1f','\x7f','\xfe', // 0x00d0
        '\xf0','\xc0','\x00','\x00','\x08','\xe0','\xfc','\xff', // 0x00d8
        '\x3f','\x07','\x01','\x00','\x00','\x06','\xe0','\xf8', // 0x00e0
        '\x7c','\x1c','\x0e','\xc6','\xc6','\x03','\xe6','\x74', // 0x00e8
        '\x38','\x78','\xe3','\xe7','\x1f','\x3f','\x3f','\x02', // 0x00f0
        '\x1f','\xef','\xe7','\xf9','\x38','\x78','\xf6','\xe6', // 0x00f8
        '\xc6','\xc6','\x02','\x8e','\x0c','\x3c','\xf8','\xf0', // 0x0100
        '\xc0','\x00','\x00','\x07','\xff','\xff','\x03','\x00', // 0x0108
        '\x00','\x05','\xff','\xff','\x04','\x00','\x00','\x07', // 0x0110
        '\x01','\x03','\x07','\x1f','\x3e','\xfc','\xf0','\xe0', // 0x0118
        '\x80','\x0f','\xff','\xff','\x03','\x00','\x00','\x05', // 0x0120
        '\x80','\xe0','\xfc','\xff','\x3f','\x0f','\x0c','\x0c', // 0x0128
        '\x0b','\x0f','\x1f','\xff','\xfc','\xf0','\x80','\x00', // 0x0130
        '\x00','\x05','\x03','\x0f','\x7f','\xfe','\xf8','\xc0', // 0x0138
        '\x00','\x80','\xf0','\xfc','\xff','\x3f','\x07','\x00', // 0x0140
        '\x00','\x0a','\x07','\x1f','\x1e','\x38','\x30','\x71', // 0x0148
        '\x63','\x63','\x02','\x71','\x30','\x38','\x1e','\x0f', // 0x0150
        '\x07','\x00','\x00','\x04','\x03','\x0f','\x1f','\x38', // 0x0158
        '\x30','\x71','\x63','\x63','\x02','\x73','\x31','\x38', // 0x0160
        '\x3c','\x1f','\x0f','\x03','\x00','\x00','\x07','\x1f', // 0x0168
        '\x3f','\x3f','\x02','\x00','\x00','\x05','\x1f','\x3f', // 0x0170
        '\x3f','\x02','\x0f','\x00','\x00','\x0d','\x01','\x03', // 0x0178
        '\x0f','\x1f','\x3f','\x3f','\x02','\x1f','\x00','\x00', // 0x0180
        '\x03','\x10','\x3c','\x3f','\x1f','\x03','\x00','\x00', // 0x0188
        '\x11','\x03','\x1f','\x3f','\x3e','\x18','\x00','\x00', // 0x0190
        '\x06','\x01','\x0f','\x1f','\x3f','\x3f','\x02','\x1f', // 0x0198
        '\x03','\x00','\x00','\xaf','\xc0','\xf0','\xf8','\x38', // 0x01a0
        '\xf8','\xf8','\x02','\xe0','\x00','\x00','\x04','\xf8', // 0x01a8
        '\xf8','\x03','\x18','\x18','\x03','\xf8','\xf8','\x02', // 0x01b0
        '\xf0','\x00','\x00','\x02','\xf8','\xf8','\x03','\x38', // 0x01b8
        '\xf8','\xe0','\x00','\xe0','\xf8','\x38','\xf8','\xf8', // 0x01c0
        '\x03','\x00','\x00','\x02','\xf8','\xf8','\x03','\x98', // 0x01c8
        '\x98','\x05','\x18','\x00','\x00','\x02','\xf8','\xf8', // 0x01d0
        '\x03','\x18','\x18','\x04','\x38','\xf8','\xf0','\xe0', // 0x01d8
        '\x00','\x00','\x42','\x20','\x38','\x3e','\x3f','\x0f', // 0x01e0
        '\x07','\x06','\x06','\x02','\x0f','\x3f','\x3f','\x02', // 0x01e8
        '\x3c','\x20','\x00','\x3f','\x3f','\x03','\x00','\x03', // 0x01f0
        '\x1f','\x3f','\x39','\x20','\x00','\x00','\x02','\x3f', // 0x01f8
        '\x3f','\x03','\x00','\x0f','\x3f','\x3c','\x0f','\x01', // 0x0200
        '\x00','\x3f','\x3f','\x03','\x00','\x00','\x02','\x3f', // 0x0208
        '\x3f','\x03','\x31','\x31','\x05','\x30','\x00','\x00', // 0x0210
        '\x02','\x3f','\x3f','\x03','\x30','\x30','\x04','\x3c', // 0x0218
        '\x1f','\x0f','\x07','\x00','\x00','\x22',
};
#endif

static const char* const pageTitles[] = {
    FC_FIRMWARE_NAME,
    "ARMED",
    "STATUS"
};

static const char* const gpsFixTypeText[] = {
    "NO",
    "2D",
    "3D"
};

static const char* tickerCharacters = "|/-\\"; // use 2/4/8 characters so that the divide is optimal.
#define TICKER_CHARACTER_COUNT (sizeof(tickerCharacters) / sizeof(char))

static timeUs_t nextPageAt;
static bool forcePageChange;
static pageId_e currentPageId;

static void resetDisplay(void)
{
    displayPresent = ug2864hsweg01InitI2C();
}

static void LCDprint(uint8_t i)
{
   i2c_OLED_send_char(i);
}

static void padLineBufferToChar(uint8_t toChar)
{
    uint8_t length = strlen(lineBuffer);
    while (length < toChar - 1) {
        lineBuffer[length++] = ' ';
    }
    lineBuffer[length] = 0;
}

#ifdef GPS
static void padLineBuffer(void)
{
    padLineBufferToChar(sizeof(lineBuffer));
}

static void padHalfLineBuffer(void)
{
    uint8_t halfLineIndex = sizeof(lineBuffer) / 2;
    padLineBufferToChar(halfLineIndex);
}
#endif

// LCDbar(n,v) : draw a bar graph - n number of chars for width, v value in % to display
static void drawHorizonalPercentageBar(uint8_t width,uint8_t percent) {
    uint8_t i, j;

    if (percent > 100)
        percent = 100;

    j = (width * percent) / 100;

    for (i = 0; i < j; i++)
        LCDprint(159); // full

    if (j < width)
        LCDprint(154 + (percent * width * 5 / 100 - 5 * j)); // partial fill

    for (i = j + 1; i < width; i++)
        LCDprint(154); // empty
}

static void updateTicker(void)
{
    static uint8_t tickerIndex = 0;
    i2c_OLED_set_xy(SCREEN_CHARACTER_COLUMN_COUNT - 1, 0);
    i2c_OLED_send_char(tickerCharacters[tickerIndex]);
    tickerIndex++;
    tickerIndex = tickerIndex % TICKER_CHARACTER_COUNT;
}

static void updateRxStatus(void)
{
    i2c_OLED_set_xy(SCREEN_CHARACTER_COLUMN_COUNT - 2, 0);
    char rxStatus = '!';
    if (rxIsReceivingSignal()) {
        rxStatus = 'r';
    } if (rxAreFlightChannelsValid()) {
        rxStatus = 'R';
    }
    i2c_OLED_send_char(rxStatus);
}

static void updateFailsafeStatus(void)
{
    char failsafeIndicator = '?';
    switch (failsafePhase()) {
        case FAILSAFE_IDLE:
            failsafeIndicator = '-';
            break;
        case FAILSAFE_RX_LOSS_DETECTED:
            failsafeIndicator = 'R';
            break;
        case FAILSAFE_RX_LOSS_IDLE:
            failsafeIndicator = 'I';
            break;
#if defined(NAV)
        case FAILSAFE_RETURN_TO_HOME:
            failsafeIndicator = 'H';
            break;
#endif
        case FAILSAFE_LANDING:
            failsafeIndicator = 'l';
            break;
        case FAILSAFE_LANDED:
            failsafeIndicator = 'L';
            break;
        case FAILSAFE_RX_LOSS_MONITORING:
            failsafeIndicator = 'M';
            break;
        case FAILSAFE_RX_LOSS_RECOVERED:
            failsafeIndicator = 'r';
            break;
    }
    i2c_OLED_set_xy(SCREEN_CHARACTER_COLUMN_COUNT - 3, 0);
    i2c_OLED_send_char(failsafeIndicator);
}

static void showTitle(void)
{
#if defined(DASHBOARD_ARMED_BITMAP)
    if (currentPageId != PAGE_ARMED) {
        i2c_OLED_set_line(0);
        i2c_OLED_send_string(pageTitles[currentPageId]);
    }
#else
    i2c_OLED_set_line(0);
    i2c_OLED_send_string(pageTitles[currentPageId]);
#endif
}

static void showWelcomePage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    tfp_sprintf(lineBuffer, "v%s (%s)", FC_VERSION_STRING, shortGitRevision);
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(targetName);
}

#if defined(DASHBOARD_ARMED_BITMAP)
// RLE compressed bitmaps must be 128 width with vertical data orientation, and size included in file.
static void bitmapDecompressAndShow(uint8_t *bitmap)
{
    uint8_t data = 0, count = 0;
    uint16_t i;
    uint8_t width = *bitmap;
    bitmap++;
    uint8_t height = *bitmap;
    bitmap++;
    uint16_t bitmapSize = (width * height) / 8;
    for (i = 0; i < bitmapSize; i++) {
        if (count == 0) {
            data = *bitmap;
            bitmap++;
            if (data == *bitmap) {
                bitmap++;
                count = *bitmap;
                bitmap++;
            }
            else {
                count = 1;
            }
        }
        count--;
        i2c_OLED_send_byte(data);
    }
}

static void showArmedPage(void)
{
    i2c_OLED_set_line(2);
    bitmapDecompressAndShow(armedBitmapRLE);
}
#else
void showArmedPage(void)
{
}
#endif

static void showStatusPage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    if (feature(FEATURE_VBAT)) {
        i2c_OLED_set_line(rowIndex++);
        tfp_sprintf(lineBuffer, "V: %d.%1d ", vbat / 10, vbat % 10);
        padLineBufferToChar(12);
        i2c_OLED_send_string(lineBuffer);

        uint8_t batteryPercentage = calculateBatteryPercentage();
        drawHorizonalPercentageBar(10, batteryPercentage);
    }

    if (feature(FEATURE_CURRENT_METER)) {
        i2c_OLED_set_line(rowIndex++);
        tfp_sprintf(lineBuffer, "mAh: %d", mAhDrawn);
        padLineBufferToChar(12);
        i2c_OLED_send_string(lineBuffer);

        uint8_t capacityPercentage = calculateBatteryCapacityRemainingPercentage();
        drawHorizonalPercentageBar(10, capacityPercentage);
    }

#ifdef GPS
    if (feature(FEATURE_GPS)) {
        tfp_sprintf(lineBuffer, "Sats: %d", gpsSol.numSat);
        padHalfLineBuffer();
        i2c_OLED_set_line(rowIndex);
        i2c_OLED_send_string(lineBuffer);

        tfp_sprintf(lineBuffer, "Fix: %s", gpsFixTypeText[gpsSol.fixType]);
        padHalfLineBuffer();
        i2c_OLED_set_xy(HALF_SCREEN_CHARACTER_COLUMN_COUNT, rowIndex++);
        i2c_OLED_send_string(lineBuffer);

        tfp_sprintf(lineBuffer, "HDOP: %d.%1d", gpsSol.hdop / 100, gpsSol.hdop % 100);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);

        tfp_sprintf(lineBuffer, "La/Lo: %d/%d", gpsSol.llh.lat / GPS_DEGREES_DIVIDER, gpsSol.llh.lon / GPS_DEGREES_DIVIDER);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);

    }
#endif

#ifdef MAG
    if (sensors(SENSOR_MAG)) {
        tfp_sprintf(lineBuffer, "HDG: %d", DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        padHalfLineBuffer();
        i2c_OLED_set_line(rowIndex);
        i2c_OLED_send_string(lineBuffer);
    }
#endif

#ifdef BARO
    if (sensors(SENSOR_BARO)) {
        int32_t alt = baroCalculateAltitude();
        tfp_sprintf(lineBuffer, "Alt: %d", alt / 100);
        padHalfLineBuffer();
        i2c_OLED_set_xy(HALF_SCREEN_CHARACTER_COLUMN_COUNT, rowIndex);
        i2c_OLED_send_string(lineBuffer);
    }
#endif

    rowIndex++;
    char rollTrim[7], pitchTrim[7];
    formatTrimDegrees(rollTrim, boardAlignment()->rollDeciDegrees);
    formatTrimDegrees(pitchTrim, boardAlignment()->pitchDeciDegrees);
    tfp_sprintf(lineBuffer, "Acc: %sR, %sP", rollTrim, pitchTrim );
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);
}

void dashboardUpdate(timeUs_t currentTimeUs)
{
    static uint8_t previousArmedState = 0;
    bool pageChanging;

#ifdef CMS
    static bool wasGrabbed = false;
    if (displayIsGrabbed(displayPort)) {
        wasGrabbed = true;
        return;
    } else if (wasGrabbed) {
        pageChanging = true;
        wasGrabbed = false;
    } else {
        pageChanging = false;
    }
#else
    pageChanging = false;
#endif

    bool updateNow = (int32_t)(currentTimeUs - nextDisplayUpdateAt) >= 0L;

    if (!updateNow) {
        return;
    }

    nextDisplayUpdateAt = currentTimeUs + DASHBOARD_UPDATE_FREQUENCY;

    bool armedState = ARMING_FLAG(ARMED) ? true : false;
    bool armedStateChanged = armedState != previousArmedState;
    previousArmedState = armedState;

    if (armedState) {
        if (!armedStateChanged) {
            return;
        }
        currentPageId = PAGE_ARMED;
        pageChanging = true;
    } else {
        if (armedStateChanged) {
            currentPageId = PAGE_STATUS;
            pageChanging = true;
        }

        if ((currentPageId == PAGE_WELCOME) && ((int32_t)(currentTimeUs - nextPageAt) >= 0L)) {
            currentPageId = PAGE_STATUS;
            pageChanging = true;
        }

        if (forcePageChange) {
            pageChanging = true;
            forcePageChange = false;
        }
    }

    if (pageChanging) {
        // Some OLED displays do not respond on the first initialisation so refresh the display
        // when the page changes in the hopes the hardware responds.  This also allows the
        // user to power off/on the display or connect it while powered.
        if (!displayPresent) {
            resetDisplay();
        }

        if (!displayPresent) {
            return;
        }

        i2c_OLED_clear_display_quick();
        showTitle();
    }

    if (!displayPresent) {
        return;
    }

    switch (currentPageId) {
        case PAGE_WELCOME:
            showWelcomePage();
            break;
        case PAGE_ARMED:
            showArmedPage();
            break;
        case PAGE_STATUS:
            showStatusPage();
            break;
    }

    if (!armedState) {
        updateFailsafeStatus();
        updateRxStatus();
        updateTicker();
    }
}

void dashboardSetPage(pageId_e newPageId)
{
    currentPageId = newPageId;
    forcePageChange = true;
}

void dashboardInit(void)
{
    delay(200);
    resetDisplay();
    delay(200);

    displayPort = displayPortOledInit();
#if defined(CMS)
    cmsDisplayPortRegister(displayPort);
#endif

    dashboardSetPage(PAGE_WELCOME);
    const timeUs_t now = micros();
    dashboardSetNextPageChangeAt(now + 5 * MICROSECONDS_IN_A_SECOND);

    dashboardUpdate(now);
}

void dashboardSetNextPageChangeAt(timeUs_t futureMicros)
{
    nextPageAt = futureMicros;
}

void formatTrimDegrees ( char *formattedTrim, int16_t trimValue ) {
    char trim[6];
    tfp_sprintf(trim, "%d", trimValue);
    int x = strlen(trim)-1;
    strncpy(formattedTrim,trim,x);
    formattedTrim[x] = '\0';
    if (trimValue !=0) {
        strcat(formattedTrim,".");
    }
    strcat(formattedTrim,trim+x);
}

#endif
