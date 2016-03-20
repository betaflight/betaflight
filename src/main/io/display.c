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
#include "drivers/system.h"
#include "drivers/display_ug2864hsweg01.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "common/printf.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/typeconversion.h"

#ifdef DISPLAY

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"

#include "rx/rx.h"

#include "io/rc_controls.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"

#ifdef GPS
#include "io/gps.h"
#include "flight/navigation_rewrite.h"
#endif

#include "config/runtime_config.h"

#include "config/config.h"

#include "display.h"

controlRateConfig_t *getControlRateConfig(uint8_t profileIndex);

#define MICROSECONDS_IN_A_SECOND (1000 * 1000)

#define DISPLAY_UPDATE_FREQUENCY (MICROSECONDS_IN_A_SECOND / 5)
#define PAGE_CYCLE_FREQUENCY (MICROSECONDS_IN_A_SECOND * 5)

static uint32_t nextDisplayUpdateAt = 0;
static bool displayPresent = false;

static rxConfig_t *rxConfig;

#define PAGE_TITLE_LINE_COUNT 1

static char lineBuffer[SCREEN_CHARACTER_COLUMN_COUNT + 1];

#define HALF_SCREEN_CHARACTER_COLUMN_COUNT (SCREEN_CHARACTER_COLUMN_COUNT / 2)
#define IS_SCREEN_CHARACTER_COLUMN_COUNT_ODD (SCREEN_CHARACTER_COLUMN_COUNT & 1)

static const char* const pageTitles[] = {
    FC_NAME
    ,"ARMED"
    ,"STATUS"
#ifdef ENABLE_DEBUG_OLED_PAGE
    ,"DEBUG"
#endif
};

static const char* const gpsFixTypeText[] = {
    "NO",
    "2D",
    "3D"
};

const pageId_e cyclePageIds[] = {
    PAGE_STATUS
#ifdef ENABLE_DEBUG_OLED_PAGE
    ,PAGE_DEBUG
#endif
};

#define CYCLE_PAGE_ID_COUNT (sizeof(cyclePageIds) / sizeof(cyclePageIds[0]))

static const char* tickerCharacters = "|/-\\"; // use 2/4/8 characters so that the divide is optimal.
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
    displayPresent = ug2864hsweg01InitI2C();
}

void LCDprint(uint8_t i) {
   i2c_OLED_send_char(i);
}

void padLineBufferToChar(uint8_t toChar)
{
    uint8_t length = strlen(lineBuffer);
    while (length < toChar - 1) {
        lineBuffer[length++] = ' ';
    }
    lineBuffer[length] = 0;
}

void padLineBuffer(void)
{
    padLineBufferToChar(sizeof(lineBuffer));
}

void padHalfLineBuffer(void)
{
    uint8_t halfLineIndex = sizeof(lineBuffer) / 2;
    padLineBufferToChar(halfLineIndex);
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

void updateTicker(void)
{
    static uint8_t tickerIndex = 0;
    i2c_OLED_set_xy(SCREEN_CHARACTER_COLUMN_COUNT - 1, 0);
    i2c_OLED_send_char(tickerCharacters[tickerIndex]);
    tickerIndex++;
    tickerIndex = tickerIndex % TICKER_CHARACTER_COUNT;
}

void updateRxStatus(void)
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

void updateFailsafeStatus(void)
{
    char failsafeIndicator = '?';
    switch (failsafePhase()) {
        case FAILSAFE_IDLE:
            failsafeIndicator = '-';
            break;
        case FAILSAFE_RX_LOSS_DETECTED:
            failsafeIndicator = 'R';
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

void showTitle()
{
    i2c_OLED_set_line(0);
    i2c_OLED_send_string(pageTitles[pageState.pageId]);
}

void showWelcomePage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    tfp_sprintf(lineBuffer, "v%s (%s)", FC_VERSION_STRING, shortGitRevision);
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(targetName);
}

void showArmedPage(void)
{
}

void showStatusPage(void)
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
    
    rowIndex++;
    
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
                
        uint8_t prevCycleIndex = pageState.cycleIndex;
        uint8_t pageId = pageState.pageId;
                
        if (pageState.pageChanging && (pageState.pageFlags & PAGE_STATE_FLAG_CYCLE_ENABLED)) {
            pageState.cycleIndex++;
            pageState.cycleIndex = pageState.cycleIndex % CYCLE_PAGE_ID_COUNT;
            pageState.pageId = cyclePageIds[pageState.cycleIndex];
            
            if (prevCycleIndex == pageState.cycleIndex && pageId != PAGE_WELCOME) {
                pageState.pageChanging = false;
            }
            
        }
        
    }

    if (pageState.pageChanging) {
        pageState.pageFlags &= ~PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
        pageState.nextPageAt = now + PAGE_CYCLE_FREQUENCY;

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

    switch(pageState.pageId) {
        case PAGE_WELCOME:
            showWelcomePage();
            break;
        case PAGE_ARMED:
            showArmedPage();
            break;
        case PAGE_STATUS:
            showStatusPage();
            break;
#ifdef ENABLE_DEBUG_OLED_PAGE
        case PAGE_DEBUG:
            showDebugPage();
            break;
#endif
    }
    if (!armedState) {
        updateFailsafeStatus();
        updateRxStatus();
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
}

void displayResetPageCycling(void)
{
    pageState.cycleIndex = CYCLE_PAGE_ID_COUNT - 1; // start at first page

}

void displayDisablePageCycling(void)
{
    pageState.pageFlags &= ~PAGE_STATE_FLAG_CYCLE_ENABLED;
}

#endif
