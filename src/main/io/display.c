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

#include "build_config.h"

#include "drivers/serial.h"
#include "common/printf.h"

#ifdef DISPLAY

#include "drivers/system.h"
#include "drivers/display_ug2864hsweg01.h"

#include "sensors/battery.h"

#include "common/axis.h"
#include "flight/flight.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"

#include "config/runtime_config.h"

#include "display.h"

#define MILLISECONDS_IN_A_SECOND (1000 * 1000)

#define DISPLAY_UPDATE_FREQUENCY (MILLISECONDS_IN_A_SECOND / 10)
#define PAGE_CYCLE_FREQUENCY (MILLISECONDS_IN_A_SECOND * 5)

uint32_t nextDisplayUpdateAt = 0;
uint32_t nextPageAt = 0;

char lineBuffer[SCREEN_CHARACTER_COLUMN_COUNT];

typedef enum {
    PAGE_BATTERY,
    PAGE_SENSORS
} pageId_e;

const char* pageTitles[] = {
    "BATTERY",
    "SENSORS"
};

#define PAGE_COUNT (PAGE_SENSORS + 1)

static const char* tickerCharacters = "|/-\\";
#define TICKER_CHARACTER_COUNT (sizeof(tickerCharacters) / sizeof(char))

typedef struct pageState_s {
    bool pageChanging;
    pageId_e pageId;
} pageState_t;

static pageState_t pageState;

void LCDprint(uint8_t i) {
   i2c_OLED_send_char(i);
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

void showBatteryPage(void)
{
    tfp_sprintf(lineBuffer, "volts: %d.%d, cells: %d", vbat / 10, vbat % 10, batteryCellCount);
    i2c_OLED_set_line(1);
    i2c_OLED_send_string(lineBuffer);

    uint32_t batteryPercentage = calculateBatteryPercentage();
    i2c_OLED_set_line(2);
    drawHorizonalPercentageBar(SCREEN_CHARACTER_COLUMN_COUNT, batteryPercentage);
}

void showSensorsPage(void)
{
    uint8_t rowIndex = 1;

    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(   "         X    Y    Z");
    if (sensors(SENSOR_ACC)) {
        tfp_sprintf(lineBuffer, "Acc : %4d %4d %4d", accSmooth[X], accSmooth[Y], accSmooth[Z]);
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }
    if (sensors(SENSOR_GYRO)) {
        tfp_sprintf(lineBuffer, "Gryo: %4d %4d %4d", gyroADC[X], gyroADC[Y], gyroADC[Z]);
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }
#ifdef MAG
    if (sensors(SENSOR_MAG)) {
        tfp_sprintf(lineBuffer, "Comp: %4d %4d %4d", magADC[X], magADC[Y], magADC[Z]);
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }
    #endif
}

void updateDisplay(void)
{
    uint32_t now = micros();

    bool updateNow = (int32_t)(now - nextDisplayUpdateAt) >= 0L;
    if (!updateNow) {
        return;
    }

    nextDisplayUpdateAt = now + DISPLAY_UPDATE_FREQUENCY;

    if (ARMING_FLAG(ARMED)) {
        return;
    }

    pageState.pageChanging = (int32_t)(now - nextPageAt) >= 0L;
    if (pageState.pageChanging) {
        nextPageAt = now + PAGE_CYCLE_FREQUENCY;
        pageState.pageId++;
        pageState.pageId = pageState.pageId % PAGE_COUNT;

        handlePageChange();
    }


    switch(pageState.pageId) {
        case PAGE_BATTERY:
            showBatteryPage();
            break;
        case PAGE_SENSORS:
            showSensorsPage();
            break;
    }

    updateTicker();
}


void displayInit(void)
{
    delay(20);
    ug2864hsweg01InitI2C();
    memset(&pageState, 0, sizeof(pageState));
}

#endif
