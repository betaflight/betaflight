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

#include "sensors/battery.h"

#include "common/axis.h"
#include "flight/flight.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"

#include "rx/rx.h"

#include "config/runtime_config.h"

#include "config/config.h"

#include "display.h"

#define MILLISECONDS_IN_A_SECOND (1000 * 1000)

#define DISPLAY_UPDATE_FREQUENCY (MILLISECONDS_IN_A_SECOND / 10)
#define PAGE_CYCLE_FREQUENCY (MILLISECONDS_IN_A_SECOND * 5)

static uint32_t nextDisplayUpdateAt = 0;

static rxConfig_t *rxConfig;

static char lineBuffer[SCREEN_CHARACTER_COLUMN_COUNT + 1];

typedef enum {
    PAGE_WELCOME,
    PAGE_ARMED,
    PAGE_BATTERY,
    PAGE_SENSORS,
    PAGE_RX,
    PAGE_PROFILE,
} pageId_e;

const char* pageTitles[] = {
    "CLEANFLIGHT",
    "ARMED",
    "BATTERY",
    "SENSORS",
    "RX",
    "PROFILE"
};

#define PAGE_COUNT (PAGE_RX + 1)

const uint8_t cyclePageIds[] = {
    PAGE_PROFILE,
    PAGE_BATTERY,
    PAGE_SENSORS,
    PAGE_RX
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

void showRxPage(void)
{

    for (uint8_t channelIndex = 0; channelIndex < 8; channelIndex += 2) {
        i2c_OLED_set_line((channelIndex / 2) + 1);

        uint8_t width = SCREEN_CHARACTER_COLUMN_COUNT / 2;

        drawRxChannel(channelIndex, width);

        if (width * 2 != SCREEN_CHARACTER_COLUMN_COUNT) {
            LCDprint(' ');
        }

        drawRxChannel(channelIndex + 1, width);
    }
}

void showWelcomePage(void)
{
    tfp_sprintf(lineBuffer, "Rev: %s", shortGitRevision);
    i2c_OLED_set_line(1);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Target: %s", targetName);
    i2c_OLED_set_line(2);
    i2c_OLED_send_string(lineBuffer);
}

void showArmedPage(void)
{
}

void showProfilePage(void)
{
    tfp_sprintf(lineBuffer, "Profile: %d", getCurrentProfile());
    i2c_OLED_set_line(1);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Rate profile: %d", getCurrentControlRateProfile());
    i2c_OLED_set_line(2);
    i2c_OLED_send_string(lineBuffer);
}

void showBatteryPage(void)
{
    tfp_sprintf(lineBuffer, "Volts: %d.%d, Cells: %d", vbat / 10, vbat % 10, batteryCellCount);
    i2c_OLED_set_line(1);
    i2c_OLED_send_string(lineBuffer);
    padLineBuffer();

    uint32_t batteryPercentage = calculateBatteryPercentage();
    i2c_OLED_set_line(2);
    drawHorizonalPercentageBar(SCREEN_CHARACTER_COLUMN_COUNT, batteryPercentage);
}

void showSensorsPage(void)
{
    uint8_t rowIndex = 1;

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

        pageState.pageChanging = (pageState.pageFlags & PAGE_STATE_FLAG_FORCE_PAGE_CHANGE) || ((int32_t)(now - pageState.nextPageAt) >= 0L);
        if (pageState.pageChanging && (pageState.pageFlags & PAGE_STATE_FLAG_CYCLE_ENABLED)) {
            pageState.nextPageAt = now + PAGE_CYCLE_FREQUENCY;
            pageState.cycleIndex++;
            pageState.cycleIndex = pageState.cycleIndex % CYCLE_PAGE_ID_COUNT;
            pageState.pageId = cyclePageIds[pageState.cycleIndex];
        }
    }

    if (pageState.pageChanging) {
        handlePageChange();
        pageState.pageFlags &= ~PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
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
    }
    if (!armedState) {
        updateTicker();
    }
}

void displayInit(rxConfig_t *rxConfigToUse)
{
    delay(20);
    ug2864hsweg01InitI2C();

    rxConfig = rxConfigToUse;

    memset(&pageState, 0, sizeof(pageState));
    pageState.pageId = PAGE_WELCOME;

    updateDisplay();

    displaySetNextPageChangeAt(micros() + (1000 * 1000 * 5));
}

void displaySetNextPageChangeAt(uint32_t futureMicros) {
    pageState.nextPageAt = futureMicros;
}

void displayEnablePageCycling(void) {
    pageState.pageFlags |= PAGE_STATE_FLAG_CYCLE_ENABLED;
    pageState.cycleIndex = CYCLE_PAGE_ID_COUNT - 1; // start at first page
}

void displayDisablePageCycling(void) {
    pageState.pageFlags &= ~PAGE_STATE_FLAG_CYCLE_ENABLED;
}

#endif
