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
#include "version.h"

#include "build_config.h"

#include "drivers/serial.h"
#include "common/printf.h"
#include "common/maths.h"

#ifdef DISPLAY

#include "drivers/system.h"
#include "drivers/display_ug2864hsweg01.h"

#include "drivers/sensor.h"
#include "drivers/compass.h"

#include "sensors/battery.h"

#include "common/axis.h"
#include "flight/flight.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"

#ifdef GPS
#include "io/gps.h"
#include "flight/navigation.h"
#endif

#include "rx/rx.h"
#include "io/rc_controls.h"

#include "config/runtime_config.h"

#include "config/config.h"

#include "display.h"

controlRateConfig_t *getControlRateConfig(uint8_t profileIndex);

//#define ENABLE_DEBUG_OLED_PAGE

#define MILLISECONDS_IN_A_SECOND (1000 * 1000)

#define DISPLAY_UPDATE_FREQUENCY (MILLISECONDS_IN_A_SECOND / 10)
#define PAGE_CYCLE_FREQUENCY (MILLISECONDS_IN_A_SECOND * 5)

static uint32_t nextDisplayUpdateAt = 0;

static rxConfig_t *rxConfig;

#define PAGE_TITLE_LINE_COUNT 1

static char lineBuffer[SCREEN_CHARACTER_COLUMN_COUNT + 1];

const char* pageTitles[] = {
    "CLEANFLIGHT",
    "ARMED",
    "BATTERY",
    "SENSORS",
    "RX",
    "PROFILE"
#ifdef GPS
    ,"GPS"
#endif
#ifdef ENABLE_DEBUG_OLED_PAGE
    ,"DEBUG"
#endif
};

#define PAGE_COUNT (PAGE_RX + 1)

const uint8_t cyclePageIds[] = {
    PAGE_PROFILE,
#ifdef GPS
    PAGE_GPS,
#endif
    PAGE_RX,
    PAGE_BATTERY,
    PAGE_SENSORS
#ifdef ENABLE_DEBUG_OLED_PAGE
    ,PAGE_DEBUG,
#endif
};

#define CYCLE_PAGE_ID_COUNT (sizeof(cyclePageIds) / sizeof(cyclePageIds[0]))

static const char* tickerCharacters = "|/-\\";
#define TICKER_CHARACTER_COUNT (sizeof(tickerCharacters) / sizeof(char))

typedef enum {
    PAGE_STATE_FLAG_NONE = 0,
    PAGE_STATE_FLAG_CYCLE_ENABLED = (1 << 0),
    PAGE_STATE_FLAG_FORCE_PAGE_CHANGE = (1 << 1)
} pageFlags_e;

typedef struct pageState_s {
    bool pageChanging;
    pageId_e pageId;
    pageId_e pageIdBeforeArming;
    uint8_t pageFlags;
    uint8_t cycleIndex;
    uint32_t nextPageAt;
} pageState_t;

static pageState_t pageState;

void resetDisplay(void) {
    ug2864hsweg01InitI2C();
}

void LCDprint(uint8_t i) {
   i2c_OLED_send_char(i);
}

void padLineBuffer(void)
{
    uint8_t length = strlen(lineBuffer);
    while (length < sizeof(lineBuffer) - 1) {
        lineBuffer[length++] = ' ';
    }
}

// LCDbar(n,v) : draw a bar graph - n number of chars for width, v value in % to display
void drawHorizonalPercentageBar(uint8_t width,uint8_t percent) {
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

#if 0
void fillScreenWithCharacters()
{
    for (uint8_t row = 0; row < SCREEN_CHARACTER_ROW_COUNT; row++) {
        for (uint8_t column = 0; column < SCREEN_CHARACTER_COLUMN_COUNT; column++) {
            i2c_OLED_set_xy(column, row);
            i2c_OLED_send_char('A' + column);
        }
    }
}
#endif

void updateTicker(void)
{
    static uint8_t tickerIndex = 0;
    i2c_OLED_set_xy(SCREEN_CHARACTER_COLUMN_COUNT - 1, 0);
    i2c_OLED_send_char(tickerCharacters[tickerIndex]);
    tickerIndex++;
    tickerIndex = tickerIndex % TICKER_CHARACTER_COUNT;
}

void showTitle()
{
    i2c_OLED_set_line(0);
    i2c_OLED_send_string(pageTitles[pageState.pageId]);
}

void handlePageChange(void)
{
    // Some OLED displays do not respond on the first initialisation so refresh the display
    // when the page changes in the hopes the hardware responds.  This also allows the
    // user to power off/on the display or connect it while powered.
    resetDisplay();

    i2c_OLED_clear_display_quick();
    showTitle();
}

void drawRxChannel(uint8_t channelIndex, uint8_t width)
{
    uint32_t percentage;

    LCDprint(rcChannelLetters[channelIndex]);

    percentage = (constrain(rcData[channelIndex], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
    drawHorizonalPercentageBar(width - 1, percentage);
}

#define HALF_SCREEN_CHARACTER_COLUMN_COUNT (SCREEN_CHARACTER_COLUMN_COUNT / 2)
#define IS_SCREEN_CHARACTER_COLUMN_COUNT_ODD (SCREEN_CHARACTER_COLUMN_COUNT & 1)

#define RX_CHANNELS_PER_PAGE_COUNT 14
void showRxPage(void)
{

    for (uint8_t channelIndex = 0; channelIndex < rxRuntimeConfig.channelCount && channelIndex < RX_CHANNELS_PER_PAGE_COUNT; channelIndex += 2) {
        i2c_OLED_set_line((channelIndex / 2) + PAGE_TITLE_LINE_COUNT);

        drawRxChannel(channelIndex, HALF_SCREEN_CHARACTER_COLUMN_COUNT);

        if (channelIndex >= rxRuntimeConfig.channelCount) {
            continue;
        }

        if (IS_SCREEN_CHARACTER_COLUMN_COUNT_ODD) {
            LCDprint(' ');
        }

        drawRxChannel(channelIndex + PAGE_TITLE_LINE_COUNT, HALF_SCREEN_CHARACTER_COLUMN_COUNT);
    }
}

void showWelcomePage(void)
{
    tfp_sprintf(lineBuffer, "Rev: %s", shortGitRevision);
    i2c_OLED_set_line(PAGE_TITLE_LINE_COUNT + 0);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Target: %s", targetName);
    i2c_OLED_set_line(PAGE_TITLE_LINE_COUNT + 1);
    i2c_OLED_send_string(lineBuffer);
}

void showArmedPage(void)
{
}

void showProfilePage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    tfp_sprintf(lineBuffer, "Profile: %d", getCurrentProfile());
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    uint8_t currentRateProfileIndex = getCurrentControlRateProfile();
    tfp_sprintf(lineBuffer, "Rate profile: %d", currentRateProfileIndex);
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    controlRateConfig_t *controlRateConfig = getControlRateConfig(currentRateProfileIndex);

    tfp_sprintf(lineBuffer, "RC Expo: %d", controlRateConfig->rcExpo8);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "RC Rate: %d", controlRateConfig->rcRate8);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "R&P Rate: %d", controlRateConfig->rollPitchRate);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Yaw Rate: %d", controlRateConfig->yawRate);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

}
#define SATELLITE_COUNT (sizeof(GPS_svinfo_cno) / sizeof(GPS_svinfo_cno[0]))
#define SATELLITE_GRAPH_LEFT_OFFSET ((SCREEN_CHARACTER_COLUMN_COUNT - SATELLITE_COUNT) / 2)

#ifdef GPS
void showGpsPage() {
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    i2c_OLED_set_xy(max(0, SATELLITE_GRAPH_LEFT_OFFSET), rowIndex++);

    uint32_t index;
    for (index = 0; index < SATELLITE_COUNT && index < SCREEN_CHARACTER_COLUMN_COUNT; index++) {
        uint8_t bargraphValue = ((uint16_t) GPS_svinfo_cno[index] * VERTICAL_BARGRAPH_CHARACTER_COUNT) / (GPS_DBHZ_MAX - 1);
        bargraphValue = min(bargraphValue, VERTICAL_BARGRAPH_CHARACTER_COUNT - 1);
        i2c_OLED_send_char(VERTICAL_BARGRAPH_ZERO_CHARACTER + bargraphValue);
    }

    char fixChar = STATE(GPS_FIX) ? 'Y' : 'N';
    tfp_sprintf(lineBuffer, "Satellites: %d Fix: %c", GPS_numSat, fixChar);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Lat: %d Lon: %d", GPS_coord[LAT] / GPS_DEGREES_DIVIDER, GPS_coord[LON] / GPS_DEGREES_DIVIDER);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Spd: %d cm/s GC: %d", GPS_speed, GPS_ground_course);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "RX: %d Delta: %d", GPS_packetCount, gpsData.lastMessage - gpsData.lastLastMessage);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "ERRs: %d TOs: %d", gpsData.errors, gpsData.timeouts);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    strncpy(lineBuffer, gpsPacketLog, GPS_PACKET_LOG_ENTRY_COUNT);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

#ifdef GPS_PH_DEBUG
    tfp_sprintf(lineBuffer, "Angles: P:%d R:%d", GPS_angle[PITCH], GPS_angle[ROLL]);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);
#endif

#if 0
    tfp_sprintf(lineBuffer, "%d %d %d %d", debug[0], debug[1], debug[2], debug[3]);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);
#endif
}
#endif

void showBatteryPage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    if (feature(FEATURE_VBAT)) {
        tfp_sprintf(lineBuffer, "Volts: %d.%1d Cells: %d", vbat / 10, vbat % 10, batteryCellCount);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);

        uint8_t batteryPercentage = calculateBatteryPercentage();
        i2c_OLED_set_line(rowIndex++);
        drawHorizonalPercentageBar(SCREEN_CHARACTER_COLUMN_COUNT, batteryPercentage);
    }

    if (feature(FEATURE_CURRENT_METER)) {
        tfp_sprintf(lineBuffer, "Amps: %d.%2d mAh: %d", amperage / 100, amperage % 100, mAhDrawn);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);

        uint8_t capacityPercentage = calculateBatteryCapacityRemainingPercentage();
        i2c_OLED_set_line(rowIndex++);
        drawHorizonalPercentageBar(SCREEN_CHARACTER_COLUMN_COUNT, capacityPercentage);
    }
}

void showSensorsPage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string("        X     Y     Z");
    if (sensors(SENSOR_ACC)) {
        tfp_sprintf(lineBuffer, "A = %5d %5d %5d", accSmooth[X], accSmooth[Y], accSmooth[Z]);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }
    if (sensors(SENSOR_GYRO)) {
        tfp_sprintf(lineBuffer, "G = %5d %5d %5d", gyroADC[X], gyroADC[Y], gyroADC[Z]);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }
#ifdef MAG
    if (sensors(SENSOR_MAG)) {
        tfp_sprintf(lineBuffer, "M = %5d %5d %5d", magADC[X], magADC[Y], magADC[Z]);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }
#endif
}

#ifdef ENABLE_DEBUG_OLED_PAGE

void showDebugPage(void)
{
    uint8_t rowIndex;

    for (rowIndex = 0; rowIndex < 4; rowIndex++) {
        tfp_sprintf(lineBuffer, "%d = %5d", rowIndex, debug[rowIndex]);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex + PAGE_TITLE_LINE_COUNT);
        i2c_OLED_send_string(lineBuffer);
    }
}
#endif

void updateDisplay(void)
{
    uint32_t now = micros();
    static uint8_t previousArmedState = 0;

    bool updateNow = (int32_t)(now - nextDisplayUpdateAt) >= 0L;
    if (!updateNow) {
        return;
    }

    nextDisplayUpdateAt = now + DISPLAY_UPDATE_FREQUENCY;

    bool armedState = ARMING_FLAG(ARMED) ? true : false;
    bool armedStateChanged = armedState != previousArmedState;
    previousArmedState = armedState;

    if (armedState) {
        if (!armedStateChanged) {
            return;
        }
        pageState.pageIdBeforeArming = pageState.pageId;
        pageState.pageId = PAGE_ARMED;
        pageState.pageChanging = true;
    } else {
        if (armedStateChanged) {
            pageState.pageFlags |= PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
            pageState.pageId = pageState.pageIdBeforeArming;
        }

        pageState.pageChanging = (pageState.pageFlags & PAGE_STATE_FLAG_FORCE_PAGE_CHANGE) ||
                (((int32_t)(now - pageState.nextPageAt) >= 0L && (pageState.pageFlags & PAGE_STATE_FLAG_CYCLE_ENABLED)));
        if (pageState.pageChanging && (pageState.pageFlags & PAGE_STATE_FLAG_CYCLE_ENABLED)) {
            pageState.cycleIndex++;
            pageState.cycleIndex = pageState.cycleIndex % CYCLE_PAGE_ID_COUNT;
            pageState.pageId = cyclePageIds[pageState.cycleIndex];
        }
    }

    if (pageState.pageChanging) {
        handlePageChange();
        pageState.pageFlags &= ~PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
        pageState.nextPageAt = now + PAGE_CYCLE_FREQUENCY;
    }

    switch(pageState.pageId) {
        case PAGE_WELCOME:
            showWelcomePage();
            break;
        case PAGE_ARMED:
            showArmedPage();
            break;
        case PAGE_BATTERY:
            showBatteryPage();
            break;
        case PAGE_SENSORS:
            showSensorsPage();
            break;
        case PAGE_RX:
            showRxPage();
            break;
        case PAGE_PROFILE:
            showProfilePage();
            break;
#ifdef GPS
        case PAGE_GPS:
            if (feature(FEATURE_GPS)) {
                showGpsPage();
            } else {
                pageState.pageFlags |= PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
            }
            break;
#endif
#ifdef ENABLE_DEBUG_OLED_PAGE
        case PAGE_DEBUG:
            showDebugPage();
            break;
#endif
    }
    if (!armedState) {
        updateTicker();
    }
}

void displaySetPage(pageId_e pageId)
{
    pageState.pageId = pageId;
    pageState.pageFlags |= PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
}

void displayInit(rxConfig_t *rxConfigToUse)
{
    delay(200);
    resetDisplay();
    delay(200);

    rxConfig = rxConfigToUse;

    memset(&pageState, 0, sizeof(pageState));
    displaySetPage(PAGE_WELCOME);

    updateDisplay();

    displaySetNextPageChangeAt(micros() + (1000 * 1000 * 5));
}

void displayShowFixedPage(pageId_e pageId)
{
    displaySetPage(pageId);
    displayDisablePageCycling();
}

void displaySetNextPageChangeAt(uint32_t futureMicros)
{
    pageState.nextPageAt = futureMicros;
}

void displayEnablePageCycling(void)
{
    pageState.pageFlags |= PAGE_STATE_FLAG_CYCLE_ENABLED;
    pageState.cycleIndex = CYCLE_PAGE_ID_COUNT - 1; // start at first page
}

void displayDisablePageCycling(void)
{
    pageState.pageFlags &= ~PAGE_STATE_FLAG_CYCLE_ENABLED;
}

#endif
