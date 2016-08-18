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
#include <string.h>

#include "platform.h"

#ifdef xxxxx //DISPLAY

#include "version.h"
#include "debug.h"

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

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "io/rc_controls.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"

#ifdef GPS
#include "io/gps.h"
#include "flight/navigation.h"
#endif

#include "config/config.h"
#include "config/runtime_config.h"
#include "config/config_profile.h"

#include "io/display.h"

#include "scheduler/scheduler.h"

extern profile_t *currentProfile;

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
    "CLEANFLIGHT",
    "ARMED",
    "BATTERY",
    "SENSORS",
    "RX",
    "PROFILE"
#ifndef SKIP_TASK_STATISTICS
    ,"TASKS"
#endif
#ifdef GPS
    ,"GPS"
#endif
#ifdef ENABLE_DEBUG_OLED_PAGE
    ,"DEBUG"
#endif
};

#define PAGE_COUNT (PAGE_RX + 1)

const pageId_e cyclePageIds[] = {
    PAGE_PROFILE,
#ifdef GPS
    PAGE_GPS,
#endif
    PAGE_RX,
    PAGE_BATTERY,
    PAGE_SENSORS
#ifndef SKIP_TASK_STATISTICS
    ,PAGE_TASKS
#endif
#ifdef ENABLE_DEBUG_OLED_PAGE
    ,PAGE_DEBUG,
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

void padLineBuffer(void)
{
    uint8_t length = strlen(lineBuffer);
    while (length < sizeof(lineBuffer) - 1) {
        lineBuffer[length++] = ' ';
    }
    lineBuffer[length] = 0;
}

void padHalfLineBuffer(void)
{
    uint8_t halfLineIndex = sizeof(lineBuffer) / 2;
    uint8_t length = strlen(lineBuffer);
    while (length < halfLineIndex - 1) {
        lineBuffer[length++] = ' ';
    }
    lineBuffer[length] = 0;
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

void showProfilePage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    tfp_sprintf(lineBuffer, "Profile: %d", getCurrentProfile());
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    static const char* const axisTitles[3] = {"ROL", "PIT", "YAW"};
    const pidProfile_t *pidProfile = &currentProfile->pidProfile;
    for (int axis = 0; axis < 3; ++axis) {
        tfp_sprintf(lineBuffer, "%s P:%3d I:%3d D:%3d",
            axisTitles[axis],
            pidProfile->P8[axis],
            pidProfile->I8[axis],
            pidProfile->D8[axis]
        );
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }

    const uint8_t currentRateProfileIndex = getCurrentControlRateProfile();
    tfp_sprintf(lineBuffer, "Rate profile: %d", currentRateProfileIndex);
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    const controlRateConfig_t *controlRateConfig = getControlRateConfig(currentRateProfileIndex);
    tfp_sprintf(lineBuffer, "RCE: %d, RCR: %d",
        controlRateConfig->rcExpo8,
        controlRateConfig->rcRate8
    );
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "RR:%d PR:%d YR:%d",
        controlRateConfig->rates[FD_ROLL],
        controlRateConfig->rates[FD_PITCH],
        controlRateConfig->rates[FD_YAW]
    );
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);
}
#define SATELLITE_COUNT (sizeof(GPS_svinfo_cno) / sizeof(GPS_svinfo_cno[0]))
#define SATELLITE_GRAPH_LEFT_OFFSET ((SCREEN_CHARACTER_COLUMN_COUNT - SATELLITE_COUNT) / 2)

#ifdef GPS
void showGpsPage() {
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;

    static uint8_t gpsTicker = 0;
    static uint32_t lastGPSSvInfoReceivedCount = 0;
    if (GPS_svInfoReceivedCount != lastGPSSvInfoReceivedCount) {
        lastGPSSvInfoReceivedCount = GPS_svInfoReceivedCount;
        gpsTicker++;
        gpsTicker = gpsTicker % TICKER_CHARACTER_COUNT;
    }

    i2c_OLED_set_xy(0, rowIndex);
    i2c_OLED_send_char(tickerCharacters[gpsTicker]);

    i2c_OLED_set_xy(MAX(0, SATELLITE_GRAPH_LEFT_OFFSET), rowIndex++);

    uint32_t index;
    for (index = 0; index < SATELLITE_COUNT && index < SCREEN_CHARACTER_COLUMN_COUNT; index++) {
        uint8_t bargraphOffset = ((uint16_t) GPS_svinfo_cno[index] * VERTICAL_BARGRAPH_CHARACTER_COUNT) / (GPS_DBHZ_MAX - 1);
        bargraphOffset = MIN(bargraphOffset, VERTICAL_BARGRAPH_CHARACTER_COUNT - 1);
        i2c_OLED_send_char(VERTICAL_BARGRAPH_ZERO_CHARACTER + bargraphOffset);
    }


    char fixChar = STATE(GPS_FIX) ? 'Y' : 'N';
    tfp_sprintf(lineBuffer, "Sats: %d Fix: %c", GPS_numSat, fixChar);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "La/Lo: %d/%d", GPS_coord[LAT] / GPS_DEGREES_DIVIDER, GPS_coord[LON] / GPS_DEGREES_DIVIDER);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Spd: %d", GPS_speed);
    padHalfLineBuffer();
    i2c_OLED_set_line(rowIndex);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "GC: %d", GPS_ground_course);
    padHalfLineBuffer();
    i2c_OLED_set_xy(HALF_SCREEN_CHARACTER_COLUMN_COUNT, rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "RX: %d", GPS_packetCount);
    padHalfLineBuffer();
    i2c_OLED_set_line(rowIndex);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "ERRs: %d", gpsData.errors, gpsData.timeouts);
    padHalfLineBuffer();
    i2c_OLED_set_xy(HALF_SCREEN_CHARACTER_COLUMN_COUNT, rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "Dt: %d", gpsData.lastMessage - gpsData.lastLastMessage);
    padHalfLineBuffer();
    i2c_OLED_set_line(rowIndex);
    i2c_OLED_send_string(lineBuffer);

    tfp_sprintf(lineBuffer, "TOs: %d", gpsData.timeouts);
    padHalfLineBuffer();
    i2c_OLED_set_xy(HALF_SCREEN_CHARACTER_COLUMN_COUNT, rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    strncpy(lineBuffer, gpsPacketLog, GPS_PACKET_LOG_ENTRY_COUNT);
    padHalfLineBuffer();
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
    static const char *format = "%s %5d %5d %5d";

    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string("        X     Y     Z");

    if (sensors(SENSOR_ACC)) {
        tfp_sprintf(lineBuffer, format, "ACC", accSmooth[X], accSmooth[Y], accSmooth[Z]);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }

    if (sensors(SENSOR_GYRO)) {
        tfp_sprintf(lineBuffer, format, "GYR", gyroADC[X], gyroADC[Y], gyroADC[Z]);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }

#ifdef MAG
    if (sensors(SENSOR_MAG)) {
        tfp_sprintf(lineBuffer, format, "MAG", magADC[X], magADC[Y], magADC[Z]);
        padLineBuffer();
        i2c_OLED_set_line(rowIndex++);
        i2c_OLED_send_string(lineBuffer);
    }
#endif

    tfp_sprintf(lineBuffer, format, "I&H", attitude.values.roll, attitude.values.pitch, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    /*
    uint8_t length;

    ftoa(EstG.A[X], lineBuffer);
    length = strlen(lineBuffer);
    while (length < HALF_SCREEN_CHARACTER_COLUMN_COUNT) {
        lineBuffer[length++] = ' ';
        lineBuffer[length+1] = 0;
    }
    ftoa(EstG.A[Y], lineBuffer + length);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);

    ftoa(EstG.A[Z], lineBuffer);
    length = strlen(lineBuffer);
    while (length < HALF_SCREEN_CHARACTER_COLUMN_COUNT) {
        lineBuffer[length++] = ' ';
        lineBuffer[length+1] = 0;
    }
    ftoa(smallAngle, lineBuffer + length);
    padLineBuffer();
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(lineBuffer);
    */

}

#ifndef SKIP_TASK_STATISTICS
void showTasksPage(void)
{
    uint8_t rowIndex = PAGE_TITLE_LINE_COUNT;
    static const char *format = "%2d%6d%5d%4d%4d";

    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string("Task max  avg mx% av%");
    cfTaskInfo_t taskInfo;
    for (cfTaskId_e taskId = 0; taskId < TASK_COUNT; ++taskId) {
        getTaskInfo(taskId, &taskInfo);
        if (taskInfo.isEnabled && taskId != TASK_SERIAL) {// don't waste a line of the display showing serial taskInfo
            const int taskFrequency = (int)(1000000.0f / ((float)taskInfo.latestDeltaTime));
            const int maxLoad = (taskInfo.maxExecutionTime * taskFrequency + 5000) / 10000;
            const int averageLoad = (taskInfo.averageExecutionTime * taskFrequency + 5000) / 10000;
            tfp_sprintf(lineBuffer, format, taskId, taskInfo.maxExecutionTime, taskInfo.averageExecutionTime, maxLoad, averageLoad);
            padLineBuffer();
            i2c_OLED_set_line(rowIndex++);
            i2c_OLED_send_string(lineBuffer);
            if (rowIndex > SCREEN_CHARACTER_ROW_COUNT) {
                break;
            }
        }
    }
}
#endif

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
        pageState.pageFlags &= ~PAGE_STATE_FLAG_FORCE_PAGE_CHANGE;
        pageState.nextPageAt = now + PAGE_CYCLE_FREQUENCY;

        // Some OLED displays do not respond on the first initialisation so refresh the display
        // when the page changes in the hopes the hardware responds.  This also allows the
        // user to power off/on the display or connect it while powered.
        resetDisplay();

        if (!displayPresent) {
            return;
        }
        handlePageChange();
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
#ifndef SKIP_TASK_STATISTICS
        case PAGE_TASKS:
            showTasksPage();
            break;
#endif
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

#ifdef DISPLAY
//#include <stdbool.h>
//#include <stdint.h>
#include <stdlib.h>
//#include <string.h>
#include <math.h>
#include <ctype.h>

//#include "platform.h"
#include "version.h"
#include "scheduler/scheduler.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/atomic.h"
#include "common/maths.h"
#include "common/typeconversion.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/usb_io.h"
#include "drivers/transponder_ir.h"
#include "drivers/sdcard.h"
#include "drivers/display_ug2864hsweg01.h"

#include "rx/rx.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"
#include "io/osd.h"
#include "io/status.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "build_config.h"
#include "debug.h"

#include "common/printf.h"

#define MAX_MENU_ROWS 4
#define MAX_MENU_COLS 3


extern profile_t *currentProfile;

controlRateConfig_t *getControlRateConfig(uint8_t profileIndex);

#define MICROSECONDS_IN_A_SECOND (1000 * 1000)

#define DISPLAY_UPDATE_FREQUENCY (MICROSECONDS_IN_A_SECOND / 5)
#define PAGE_CYCLE_FREQUENCY (MICROSECONDS_IN_A_SECOND * 5)


static rxConfig_t *rxConfig;

#define PAGE_TITLE_LINE_COUNT 1

static char ScreenBuffer[SCREEN_CHARACTER_COLUMN_COUNT*SCREEN_CHARACTER_ROW_COUNT + 1];

#define HALF_SCREEN_CHARACTER_COLUMN_COUNT (SCREEN_CHARACTER_COLUMN_COUNT / 2)
#define IS_SCREEN_CHARACTER_COLUMN_COUNT_ODD (SCREEN_CHARACTER_COLUMN_COUNT & 1)

#define STICKMIN 10
#define STICKMAX 90

/** Artificial Horizon limits **/
#define AHIPITCHMAX 200             // Specify maximum AHI pitch value displayed. Default 200 = 20.0 degrees
#define AHIROLLMAX  400             // Specify maximum AHI roll value displayed. Default 400 = 40.0 degrees
#define AHISIDEBARWIDTHPOSITION 7
#define AHISIDEBARHEIGHTPOSITION 3


static uint32_t nextDisplayUpdateAt = 0;
static bool displayPresent = false;
static uint32_t armed_seconds      = 0;
static uint32_t armed_at           = 0;
static uint8_t armed               = 0;

static uint8_t current_page        = 0;
static uint8_t sticks[]            = {0,0,0,0};

static uint8_t cursor_row          = 255;
static uint8_t cursor_col          = 0;
static uint8_t in_menu             = 0;
static uint8_t activating_menu     = 0;

static char string_buffer[30];

extern uint16_t rssi;

enum {
    MENU_VALUE_DECREASE = 0,
    MENU_VALUE_INCREASE,
};

void resetDisplay(void) {
    displayPresent = ug2864hsweg01InitI2C();
}

void print_pid(uint16_t pos, uint8_t col, int pid_term) {
    switch(col) {
        case 0:
            tfp_sprintf(string_buffer, "%d", currentProfile->pidProfile.P8[pid_term]);
            memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
            break;
        case 1:
            tfp_sprintf(string_buffer, "%d", currentProfile->pidProfile.I8[pid_term]);
            memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
            break;
        case 2:
            tfp_sprintf(string_buffer, "%d", currentProfile->pidProfile.D8[pid_term]);
            memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
            break;
        default:
            return;
    }
    //max7456_write_string(string_buffer, pos);
}

void print_roll_pid(uint16_t pos, uint8_t col) {
    print_pid(pos, col, ROLL);
}

void print_pitch_pid(uint16_t pos, uint8_t col) {
    print_pid(pos, col, PITCH);
}

void print_yaw_pid(uint16_t pos, uint8_t col) {
    print_pid(pos, col, YAW);
}

void print_roll_rate(uint16_t pos, uint8_t col) {
    if (col == 0) {
        tfp_sprintf(string_buffer, "%d", currentControlRateProfile->rates[FD_ROLL]);
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));

        //max7456_write_string(string_buffer, pos);
    }
}

void print_pitch_rate(uint16_t pos, uint8_t col) {
    if (col == 0) {
        tfp_sprintf(string_buffer, "%d", currentControlRateProfile->rates[FD_PITCH]);
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        //max7456_write_string(string_buffer, pos);
    }
}

void print_yaw_rate(uint16_t pos, uint8_t col) {
    if (col == 0) {
        tfp_sprintf(string_buffer, "%d", currentControlRateProfile->rates[FD_YAW]);
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        //max7456_write_string(string_buffer, pos);
    }
}

void print_tpa_rate(uint16_t pos, uint8_t col) {
    if (col == 0) {
        tfp_sprintf(string_buffer, "%d", currentControlRateProfile->dynThrPID);
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        //max7456_write_string(string_buffer, pos);
    }
}

void print_tpa_brkpt(uint16_t pos, uint8_t col) {
    if (col == 0) {
        tfp_sprintf(string_buffer, "%d", currentControlRateProfile->tpa_breakpoint);
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        //max7456_write_string(string_buffer, pos);
    }
}

void update_int_pid(int value_change_direction, uint8_t col, int pid_term) {
    void* ptr;

    switch(col) {
        case 0:
            ptr = &currentProfile->pidProfile.P8[pid_term];
            break;
        case 1:
            ptr = &currentProfile->pidProfile.I8[pid_term];
            break;
        case 2:
            ptr = &currentProfile->pidProfile.D8[pid_term];
            break;
        default:
            return;
    }

    if (value_change_direction) {
        if (*(uint8_t*)ptr < 200)
            *(uint8_t*)ptr += 1;
    } else {
        if (*(uint8_t*)ptr > 0)
            *(uint8_t*)ptr -= 1;
    }
}

void update_roll_pid(int value_change_direction, uint8_t col) {
    update_int_pid(value_change_direction, col, ROLL);
}

void update_pitch_pid(int value_change_direction, uint8_t col) {
    update_int_pid(value_change_direction, col, PITCH);
}

void update_yaw_pid(int value_change_direction, uint8_t col) {
    update_int_pid(value_change_direction, col, YAW);
}

void update_roll_rate(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->rates[FD_ROLL] < CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX)
            currentControlRateProfile->rates[FD_ROLL]++;
    } else {
        if (currentControlRateProfile->rates[FD_ROLL] > 0)
            currentControlRateProfile->rates[FD_ROLL]--;
    }
}

void update_pitch_rate(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->rates[FD_PITCH] < CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX)
            currentControlRateProfile->rates[FD_PITCH]++;
    } else {
        if (currentControlRateProfile->rates[FD_PITCH] > 0)
            currentControlRateProfile->rates[FD_PITCH]--;
    }
}

void update_yaw_rate(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->rates[FD_YAW] < CONTROL_RATE_CONFIG_YAW_RATE_MAX)
            currentControlRateProfile->rates[FD_YAW]++;
    } else {
        if (currentControlRateProfile->rates[FD_YAW] > 0)
            currentControlRateProfile->rates[FD_YAW]--;
    }
}

void update_tpa_rate(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->dynThrPID < CONTROL_RATE_CONFIG_TPA_MAX)
            currentControlRateProfile->dynThrPID++;
    } else {
        if (currentControlRateProfile->dynThrPID > 0)
            currentControlRateProfile->dynThrPID--;
    }
}

void update_tpa_brkpt(int value_change_direction, uint8_t col) {
    UNUSED(col);

    if (value_change_direction) {
        if (currentControlRateProfile->tpa_breakpoint < PWM_RANGE_MAX)
            currentControlRateProfile->tpa_breakpoint++;
    } else {
        if (currentControlRateProfile->tpa_breakpoint > PWM_RANGE_MIN)
            currentControlRateProfile->tpa_breakpoint--;
    }
}

void print_average_system_load(uint16_t pos, uint8_t col) {
    UNUSED(col);

    tfp_sprintf(string_buffer, "%d", averageSystemLoadPercent);
    memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
    //max7456_write_string(string_buffer, pos);
}

void print_batt_voltage(uint16_t pos, uint8_t col) {
    UNUSED(col);

    tfp_sprintf(string_buffer, "%d.%1d", vbat / 10, vbat % 10);
    memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
    //max7456_write_string(string_buffer, pos);
}

// LCDbar(n,v) : draw a bar graph - n number of chars for width, v value in % to display
void drawHorizonalPercentageBar(uint8_t width,uint8_t percent,char* buffer) {
    uint8_t i, j;

    if (percent > 100)
        percent = 100;

    j = (width * percent) / 100;

    for (i = 0; i < j; i++)
        buffer[i] = 159; // full

    if (j < width)
        buffer[j] = 154 + (percent * width * 5 / 100 - 5 * j); // partial fill

    for (i = j + 1; i < width; i++)
        buffer[i] = 154; // empty
}

void print_channel_15(uint16_t pos, uint8_t col) {
uint32_t percentage;
  switch(col) {
      case 0:
        percentage = (constrain(rcData[0], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        drawHorizonalPercentageBar(8, percentage, ScreenBuffer + pos +2);
        ScreenBuffer[pos] = rcChannelLetters[0];
        break;
      case 1:
        percentage = (constrain(rcData[4], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        drawHorizonalPercentageBar(8, percentage, ScreenBuffer + pos +2);
        ScreenBuffer[pos] = rcChannelLetters[4];
        break;
      default:
        return;

    }
}

void print_channel_26(uint16_t pos, uint8_t col) {
uint32_t percentage;
  switch(col) {
      case 0:
        percentage = (constrain(rcData[1], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        drawHorizonalPercentageBar(8, percentage, ScreenBuffer + pos +2);
        ScreenBuffer[pos] = rcChannelLetters[1];
        break;
      case 1:
        percentage = (constrain(rcData[5], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        drawHorizonalPercentageBar(8, percentage, ScreenBuffer + pos +2);
        ScreenBuffer[pos] = rcChannelLetters[5];
        break;
      default:
        return;

    }
}

void print_channel_37(uint16_t pos, uint8_t col) {
uint32_t percentage;
  switch(col) {
      case 0:
        percentage = (constrain(rcData[2], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        drawHorizonalPercentageBar(8, percentage, ScreenBuffer + pos +2);
        ScreenBuffer[pos] = rcChannelLetters[2];
        break;
      case 1:
        percentage = (constrain(rcData[6], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        drawHorizonalPercentageBar(8, percentage, ScreenBuffer + pos +2);
        ScreenBuffer[pos] = rcChannelLetters[6];
        break;
      default:
        return;

    }
}

void print_channel_48(uint16_t pos, uint8_t col) {
uint32_t percentage;
  switch(col) {
      case 0:
        percentage = (constrain(rcData[3], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        drawHorizonalPercentageBar(8, percentage, ScreenBuffer + pos +2);
        ScreenBuffer[pos] = rcChannelLetters[3];
        break;
      case 1:
        percentage = (constrain(rcData[7], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        drawHorizonalPercentageBar(8, percentage, ScreenBuffer + pos +2);
        ScreenBuffer[pos] = rcChannelLetters[7];
        break;
      default:
        return;

    }
}


void print_gps_sats(uint16_t pos, uint8_t col) {
  char fixChar = STATE(GPS_FIX) ? 'Y' : 'N';
  switch(col) {
      case 0:
        tfp_sprintf(string_buffer, "Sats: %d",GPS_numSat); //logo missing
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        break;
      case 1:
        tfp_sprintf(string_buffer, "Fix: %c",fixChar); //logo missing
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        break;
      default:
        return;

    }
}

void print_gps_lo_la(uint16_t pos, uint8_t col) {
  switch(col) {
      case 0:
        tfp_sprintf(string_buffer, "La: %d", GPS_coord[LAT] / GPS_DEGREES_DIVIDER); //logo missing
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        break;
      case 1:
        tfp_sprintf(string_buffer, "Lo: %d", GPS_coord[LON] / GPS_DEGREES_DIVIDER); //logo missing
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        break;
      default:
        return;

    }
}

void print_gps_spd_gc(uint16_t pos, uint8_t col) {
  switch(col) {
      case 0:
        tfp_sprintf(string_buffer, "Spd: %d", GPS_speed); //logo missing
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        break;
      case 1:
        tfp_sprintf(string_buffer, "GC: %d", GPS_ground_course); //logo missing
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        break;
      default:
        return;
    }
}

void print_gps_packets_errors(uint16_t pos, uint8_t col) {
  switch(col) {
      case 0:
        tfp_sprintf(string_buffer, "RX: %d", GPS_packetCount); //logo missing
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        break;
      case 1:
        tfp_sprintf(string_buffer, "Err: %d", gpsData.errors); //logo missing
        memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
        break;
      default:
        return;
    }
}




//FIXME: add old pages into menu structure
osd_page_t menu_pages[] = {
    {
        .title = "STATUS",
        .cols_number = 1,
        .rows_number = 2,
        .cols = {
            {
                .title = NULL,
                .x_pos = 10
            }
        },
        .rows = {
            {
                .title  = "AVG LOAD",
                .update = NULL,
                .print  = print_average_system_load
            },
            {
                .title  = "BATT",
                .update = NULL,
                .print  = print_batt_voltage
            },
        }
    },
    {
        .title = "CHANNELS 1-8",
        .cols_number = 2,
        .rows_number = 4,
        .cols = {
            {
                .title = NULL,
                .x_pos = 1
            },
            {
                .title = NULL,
                .x_pos = 11
            }
        },
        .rows = {
            {
                .title  = NULL,
                .update = NULL,
                .print  = print_channel_15
            },
            {
                .title  = NULL,
                .update = NULL,
                .print  = print_channel_26
            },
            {
                .title  = NULL,
                .update = NULL,
                .print  = print_channel_37
            },
            {
                .title  = NULL,
                .update = NULL,
                .print  = print_channel_48
            },
        }
    },
    {
        .title = "GPS",
        .cols_number = 2,
        .rows_number = 4,
        .cols = {
            {
                .title = NULL,
                .x_pos = 1
            },
            {
                .title = NULL,
                .x_pos = 11
            }
        },
        .rows = {
            {
                .title  = NULL,
                .update = NULL,
                .print  = print_gps_sats
            },
            {
                .title  = NULL,
                .update = NULL,
                .print  = print_gps_lo_la
            },
            {
                .title  = NULL,
                .update = NULL,
                .print  = print_gps_spd_gc
            },
            {
                .title  = NULL,
                .update = NULL,
                .print  = print_gps_packets_errors
            },
        }
    },
#ifdef USE_RTC6705
    {
        .title       = "VTX SETTINGS",
        .cols_number = 1,
        .rows_number = 4,
        .cols = {
            {
                .title = NULL,
                .x_pos = 10
            }
        },
        .rows = {
            {
                .title  = "BAND",
                .update = update_vtx_band,
                .print  = print_vtx_band
            },
            {
                .title  = "CHANNEL",
                .update = update_vtx_channel,
                .print  = print_vtx_channel
            },
            {
                .title  = "FREQUENCY",
                .update = NULL,
                .print  = print_vtx_freq
            },
            {
                .title  = "POWER",
                .update = update_vtx_power,
                .print  = print_vtx_power
            }
        }
    },
#endif
    {
        .title       = "PID SETTINGS",
        .cols_number = 3,
        .rows_number = 4,
        .cols = {
            {
                .title = "P",
                .x_pos = 10
            },
            {
                .title = "I",
                .x_pos = 14
            },
            {
                .title = "D",
                .x_pos = 18
            }
        },
        .rows = {
            {
                .title  = "ROLL",
                .update = update_roll_pid,
                .print  = print_roll_pid
            },
            {
                .title  = "PITCH",
                .update = update_pitch_pid,
                .print  = print_pitch_pid
            },
            {
                .title  = "YAW",
                .update = update_yaw_pid,
                .print  = print_yaw_pid
            },
            {
                .title  = "ROLL RATE",
                .update = update_roll_rate,
                .print  = print_roll_rate
            },

        }
    },
    {
        .title       = "PID SETTINGS 2",
        .cols_number = 1,
        .rows_number = 4,
        .cols = {
            {
                .title = "P",
                .x_pos = 14
            }
        },
        .rows = {
            {
                .title  = "PITCH RATE",
                .update = update_pitch_rate,
                .print  = print_pitch_rate
            },
            {
                .title  = "YAW RATE",
                .update = update_yaw_rate,
                .print  = print_yaw_rate
            },
            {
                .title  = "TPA RATE",
                .update = update_tpa_rate,
                .print  = print_tpa_rate
            },
            {
                .title  = "TPA BRKPT",
                .update = update_tpa_brkpt,
                .print  = print_tpa_brkpt
            },
        }
    },


};

void show_menu(void) {
    uint8_t line = 0;
    uint16_t pos;
    osd_col_t *col;
    osd_row_t *row;
    int16_t cursor_x = 0;
    int16_t cursor_y = 0;

    if (activating_menu) {
        if (sticks[YAW] < 60 && sticks[YAW] > 40 && sticks[PITCH] > 40 && sticks[PITCH] < 60 && sticks[ROLL] > 40 && sticks[ROLL] < 60)
            activating_menu = false;
        else
            return;
    }

    if (sticks[YAW] > STICKMAX && sticks[ROLL] > STICKMIN && sticks[ROLL] < STICKMAX && sticks[PITCH] > STICKMIN && sticks[PITCH] < STICKMAX) {
        if (cursor_row > MAX_MENU_ROWS) {
            switch(cursor_col) {
                case 0:
                    in_menu = false;
                    break;
                case 1:
                    in_menu = false;
#ifdef USE_RTC6705
                    if (masterConfig.vtx_channel != current_vtx_channel) {
                        masterConfig.vtx_channel = current_vtx_channel;
                        rtc6705_soft_spi_set_channel(vtx_freq[current_vtx_channel]);
                    }
#endif
                    writeEEPROM();
                    break;
                case 2:
                    if (current_page < (sizeof(menu_pages) / sizeof(osd_page_t) - 1))
                        current_page++;
                    else
                        current_page = 0;
            }
        } else {
            if (menu_pages[current_page].rows[cursor_row].update)
                menu_pages[current_page].rows[cursor_row].update(MENU_VALUE_INCREASE, cursor_col);
        }
    }

    if (sticks[YAW] < STICKMIN && sticks[ROLL] > STICKMIN && sticks[ROLL] < STICKMAX && sticks[PITCH] > STICKMIN && sticks[PITCH] < STICKMAX) {
        if (cursor_row > MAX_MENU_ROWS) {
            if (cursor_col == 2 && current_page > 0) {
                current_page--;
            }
        } else {
            if (menu_pages[current_page].rows[cursor_row].update)
                menu_pages[current_page].rows[cursor_row].update(MENU_VALUE_DECREASE, cursor_col);
        }
    }

    if (sticks[PITCH] > STICKMAX && sticks[YAW] > STICKMIN && sticks[YAW] < STICKMAX) {
        if (cursor_row > MAX_MENU_ROWS) {
            cursor_row = menu_pages[current_page].rows_number - 1;
            cursor_col = 0;
        } else {
            if (cursor_row > 0)
                cursor_row--;
        }
    }
    if (sticks[PITCH] < STICKMIN && sticks[YAW] > STICKMIN && sticks[YAW] < STICKMAX) {
        if (cursor_row < (menu_pages[current_page].rows_number - 1))
            cursor_row++;
        else
            cursor_row = 255;
    }
    if (sticks[ROLL] > STICKMAX && sticks[YAW] > STICKMIN && sticks[YAW] < STICKMAX) {
        if (cursor_row > MAX_MENU_ROWS) {
            if (cursor_col < 2)
                cursor_col++;
        } else {
            if (cursor_col < (menu_pages[current_page].cols_number - 1))
                cursor_col++;
        }
    }
    if (sticks[ROLL] < STICKMIN && sticks[YAW] > STICKMIN && sticks[YAW] < STICKMAX) {
        if (cursor_col > 0)
            cursor_col--;
    }

    if (cursor_row > MAX_MENU_ROWS) {
        cursor_row = 255;
        cursor_y = -1;
        switch(cursor_col) {
            case 0:
                cursor_x = 0;
                break;
            case 1:
                cursor_x = SCREEN_CHARACTER_COLUMN_COUNT/3;
                break;
            case 2:
                cursor_x = SCREEN_CHARACTER_COLUMN_COUNT*2/3;
                break;
            default:
                cursor_x = 0;
        }
    }

    tfp_sprintf(string_buffer, "EXIT  SAVE+EXIT  PAGE");
    memcpy(ScreenBuffer+(SCREEN_CHARACTER_ROW_COUNT-1)*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));
    //max7456_write_string(string_buffer, -29);

    pos = (SCREEN_CHARACTER_COLUMN_COUNT - strlen(menu_pages[current_page].title)) / 2 + line * SCREEN_CHARACTER_COLUMN_COUNT;
    tfp_sprintf(string_buffer, "%s", menu_pages[current_page].title); // FIXME: Inverted?
    memcpy(ScreenBuffer+pos,string_buffer,strlen(string_buffer));
    //max7456_write_string(string_buffer, pos);

    line += 1;

    for (int i = 0; i < menu_pages[current_page].cols_number; i++){
        col = &menu_pages[current_page].cols[i];
        if (cursor_col == i && cursor_row < MAX_MENU_ROWS)
            cursor_x = col->x_pos - 1;

        if (col->title) {
            tfp_sprintf(string_buffer , "%s", col->title);
            memcpy(ScreenBuffer+line * SCREEN_CHARACTER_COLUMN_COUNT + col->x_pos,string_buffer,strlen(string_buffer));
            //max7456_write_string(string_buffer, line * OSD_LINE_LENGTH + col->x_pos);
        }
    }

    line++;
    for (int i = 0; i < menu_pages[current_page].rows_number; i++) {
        row = &menu_pages[current_page].rows[i];
        if (cursor_row == i)
            cursor_y = line;

        tfp_sprintf(string_buffer, "%s", row->title);
        memcpy(ScreenBuffer+line * SCREEN_CHARACTER_COLUMN_COUNT + 1,string_buffer,strlen(string_buffer));
        //max7456_write_string(string_buffer, line * OSD_LINE_LENGTH + 1);
        for (int j = 0; j < menu_pages[current_page].cols_number; j++) {
            col = &menu_pages[current_page].cols[j];
            row->print(line * SCREEN_CHARACTER_COLUMN_COUNT + col->x_pos, j);
        }
        line++;
    }

    //max7456_write_string(">", cursor_x + cursor_y * OSD_LINE_LENGTH);
    ScreenBuffer[cursor_x + cursor_y * SCREEN_CHARACTER_COLUMN_COUNT] = '>';
}

// Write the artifical horizon to the screen buffer
void osdDrawArtificialHorizon(int rollAngle, int pitchAngle, uint8_t show_sidebars) {
  /*
    uint16_t position = 194;
    //MAX7456_CHAR_TYPE *ScreenBuffer = max7456_get_screen_buffer();

    if (pitchAngle > AHIPITCHMAX)
        pitchAngle = AHIPITCHMAX;
    if (pitchAngle < -AHIPITCHMAX)
        pitchAngle = -AHIPITCHMAX;
    if (rollAngle > AHIROLLMAX)
        rollAngle = AHIROLLMAX;
    if (rollAngle < -AHIROLLMAX)
        rollAngle = -AHIROLLMAX;

    for (uint8_t X = 0; X <= 8; X++) {
        if (X == 4)
            X = 5;
        int Y = (rollAngle * (4 - X)) / 64;
        Y -= pitchAngle / 8;
        Y += 41;
        if (Y >= 0 && Y <= 81) {
            uint16_t pos = position - 7 + LINE * (Y / 9) + 3 - 4 * LINE + X;
            ScreenBuffer[pos] = MAX7456_CHAR(SYM_AH_BAR9_0 + (Y % 9));
        }
    }
    ScreenBuffer[position - 1] = MAX7456_CHAR(SYM_AH_CENTER_LINE);
    ScreenBuffer[position + 1] = MAX7456_CHAR(SYM_AH_CENTER_LINE_RIGHT);
    ScreenBuffer[position]     = MAX7456_CHAR(SYM_AH_CENTER);

    if (show_sidebars) {
        // Draw AH sides
        int8_t hudwidth  = AHISIDEBARWIDTHPOSITION;
        int8_t hudheight = AHISIDEBARHEIGHTPOSITION;
        for (int8_t X = -hudheight; X <= hudheight; X++) {
            ScreenBuffer[position - hudwidth + (X * LINE)] = MAX7456_CHAR(SYM_AH_DECORATION);
            ScreenBuffer[position + hudwidth + (X * LINE)] = MAX7456_CHAR(SYM_AH_DECORATION);
        }
        // AH level indicators
        ScreenBuffer[position-hudwidth+1] =  MAX7456_CHAR(SYM_AH_LEFT);
        ScreenBuffer[position+hudwidth-1] =  MAX7456_CHAR(SYM_AH_RIGHT);
    }
    */
}


void updateDisplay(void)
{
    uint32_t now = micros();

    bool updateNow = (int32_t)(now - nextDisplayUpdateAt) >= 0L;
    if (!updateNow) {
        return;
    }

    nextDisplayUpdateAt = now + DISPLAY_UPDATE_FREQUENCY;

    static uint8_t blink = 0;

    blink++;

    // Some OLED displays do not respond on the first initialisation so refresh the display
    // when the page changes in the hopes the hardware responds.  This also allows the
    // user to power off/on the display or connect it while powered.
    //
    // Reset every 30 seconds
    //
    static uint32_t reset_now = 0; // display reset timer
    if (now - MICROSECONDS_IN_A_SECOND * 30 >reset_now) {
      resetDisplay();
      reset_now = now;
    }

    if (!displayPresent) {
        return;
    }

    if (ARMING_FLAG(ARMED)) {
        in_menu = false;
        if (!armed) {
            armed = true;
            armed_at = now;
            // clear the display
            i2c_OLED_clear_display_quick();
            i2c_OLED_set_line(3);
            i2c_OLED_send_line_inverted("                      ");
            i2c_OLED_set_line(4);
            i2c_OLED_send_string("####### ARMED ########");
            i2c_OLED_set_line(5);
            i2c_OLED_send_line_inverted("                      ");
        }
        // in armed state we update only status line
        // FIXME: status update speed
        composeStatus(statusLine,sizeof(statusLine));
        i2c_OLED_set_line(0);
        i2c_OLED_send_string_formatted(statusLine);

    } else {
        if (armed) {
            armed = false;
            armed_seconds += ((now - armed_at) / 1000000);
        }
        for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
            sticks[channelIndex] = (constrain(rcData[channelIndex], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
        }
        if (!in_menu && sticks[YAW] > STICKMAX && sticks[THROTTLE] > STICKMIN && sticks[THROTTLE] < STICKMAX && sticks[ROLL] > STICKMIN && sticks[ROLL] < STICKMAX && sticks[PITCH] > STICKMAX) {
            in_menu = true;
            cursor_row = 255;
            cursor_col = 2;
            activating_menu = true;
        }
        // "OSD" is shown only in disarmed state on OLED
        if (in_menu) {
          // most of the things are moved into menu_pages
          // including old OLED display pages
          //FIXME: add old pages into menu structure
          show_menu();

          // buffer to screen
          // first and last lines are inverted
          uint8_t row;
          i2c_OLED_set_line(0);
          i2c_OLED_send_line_inverted(ScreenBuffer);
          for (row = 1; row <SCREEN_CHARACTER_ROW_COUNT-1; row++) {
            i2c_OLED_set_line(row);
            i2c_OLED_send_line(ScreenBuffer+row*SCREEN_CHARACTER_COLUMN_COUNT);
          }
          i2c_OLED_set_line(7);
          i2c_OLED_send_line_inverted(ScreenBuffer+(SCREEN_CHARACTER_ROW_COUNT-1)*SCREEN_CHARACTER_COLUMN_COUNT);
        } else {
          /*
          if (batteryWarningVoltage > vbat && (blink & 1) && masterConfig.osdProfile.item_pos[OSD_VOLTAGE_WARNING] != -1) {
            max7456_write_string("LOW VOLTAGE", masterConfig.osdProfile.item_pos[OSD_VOLTAGE_WARNING]);
          }
          if (arming && (blink & 1) && masterConfig.osdProfile.item_pos[OSD_ARMED] != -1) {
            max7456_write_string("ARMED", masterConfig.osdProfile.item_pos[OSD_ARMED]);
            arming--;
          }
          if (!armed && masterConfig.osdProfile.item_pos[OSD_DISARMED] != -1) {
            max7456_write_string("DISARMED", masterConfig.osdProfile.item_pos[OSD_DISARMED]);
          }

          if (masterConfig.osdProfile.item_pos[OSD_MAIN_BATT_VOLTAGE] != -1) {
            line[0] = SYM_VOLT;
            tfp_sprintf(line+1, "%d.%1d", vbat / 10, vbat % 10);
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_MAIN_BATT_VOLTAGE]);
          }

          if (masterConfig.osdProfile.item_pos[OSD_CURRENT_DRAW] != -1) {
            line[0] = SYM_AMP;
            tfp_sprintf(line+1, "%d.%02d", amperage / 100, amperage % 100);
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_CURRENT_DRAW]);
          }

          if (masterConfig.osdProfile.item_pos[OSD_MAH_DRAWN] != -1) {
            line[0] = SYM_MAH;
            tfp_sprintf(line+1, "%d", mAhDrawn);
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_MAH_DRAWN]);
          }

          if (masterConfig.osdProfile.item_pos[OSD_CRAFT_NAME] != -1) {
            for (uint8_t i = 0; i < MAX_NAME_LENGTH; i++) {
              line[i] = toupper((unsigned char)masterConfig.name[i]);
              if (masterConfig.name[i] == 0)
              break;
            }
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_CRAFT_NAME]);
          }

          if (masterConfig.osdProfile.item_pos[OSD_RSSI_VALUE] != -1) {
            line[0] = SYM_RSSI;
            tfp_sprintf(line+1, "%d", rssi / 10);
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_RSSI_VALUE]);
          }
          if (masterConfig.osdProfile.item_pos[OSD_THROTTLE_POS] != -1) {
            line[0] = SYM_THR;
            line[1] = SYM_THR1;
            tfp_sprintf(line+2, "%3d", (constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN));
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_THROTTLE_POS]);
          }
          if (masterConfig.osdProfile.item_pos[OSD_TIMER] != -1) {
            if (armed) {
              seconds = armed_seconds + ((now-armed_at) / 1000000);
              line[0] = SYM_FLY_M;
              tfp_sprintf(line+1, " %02d:%02d", seconds / 60, seconds % 60);
            } else {
              line[0] = SYM_ON_M;
              seconds = now  / 1000000;
              tfp_sprintf(line+1, " %02d:%02d", seconds / 60, seconds % 60);
            }
            max7456_write_string(line, masterConfig.osdProfile.item_pos[OSD_TIMER]);
          }
          if (masterConfig.osdProfile.item_pos[OSD_CPU_LOAD] != -1) {
            print_average_system_load(masterConfig.osdProfile.item_pos[OSD_CPU_LOAD], 0);
          }

          if (masterConfig.osdProfile.item_pos[OSD_ARTIFICIAL_HORIZON] != -1) {
            osdDrawArtificialHorizon(attitude.values.roll, attitude.values.pitch, masterConfig.osdProfile.item_pos[OSD_HORIZON_SIDEBARS] != -1);
          }
          */
          composeStatus(statusLine,sizeof(statusLine));
          i2c_OLED_set_line(0);
          i2c_OLED_send_string_formatted(statusLine);
          tfp_sprintf(string_buffer,"MENU: THRT MID        ");
          memcpy(ScreenBuffer+3*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));
          tfp_sprintf(string_buffer,"      YAW RIGHT       ");
          memcpy(ScreenBuffer+4*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));
          tfp_sprintf(string_buffer,"      PITCH UP       ");
          memcpy(ScreenBuffer+5*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));
          uint8_t row;
          for (row = 1; row <SCREEN_CHARACTER_ROW_COUNT; row++) {
            i2c_OLED_set_line(row);
            i2c_OLED_send_line(ScreenBuffer+row*SCREEN_CHARACTER_COLUMN_COUNT);
          }
        }

        //max7456_draw_screen();

        // clear screen bufffer for next round
        memset(&ScreenBuffer, ' ', sizeof(ScreenBuffer));
        ScreenBuffer[sizeof(ScreenBuffer)-1]=0;
        // Artificial horizon is graphics drawn over text...
        /*
        if (!in_menu) {
          if (masterConfig.osdProfile.item_pos[OSD_ARTIFICIAL_HORIZON] != -1) {
              osdDrawArtificialHorizon(attitude.values.roll, attitude.values.pitch, masterConfig.osdProfile.item_pos[OSD_HORIZON_SIDEBARS] != -1);
          }
        }
        */
    }
}


void displayInit(rxConfig_t *rxConfigToUse)
{
    delay(200);
    resetDisplay();
    delay(200);

    rxConfig = rxConfigToUse;

    memset(&ScreenBuffer, ' ', sizeof(ScreenBuffer));
    ScreenBuffer[sizeof(ScreenBuffer)-1]=0;

    i2c_OLED_clear_display_quick();
    tfp_sprintf(string_buffer, "Betaflight            "); //logo missing
    memcpy(ScreenBuffer+0*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));
    tfp_sprintf(string_buffer, "           Betaflight "); //logo missing
    memcpy(ScreenBuffer+1*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));
    tfp_sprintf(string_buffer, "Welcome               "); //logo missing
    memcpy(ScreenBuffer+2*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));

    tfp_sprintf(string_buffer, "v%s (%s)", FC_VERSION_STRING, shortGitRevision);
    memcpy(ScreenBuffer+3*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));

    tfp_sprintf(string_buffer, "                     "); //logo missing
    memcpy(ScreenBuffer+4*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));

    tfp_sprintf(string_buffer,"MENU: THROTTLET MID   ");
    memcpy(ScreenBuffer+5*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));
    tfp_sprintf(string_buffer,"      YAW       RIGHT ");
    memcpy(ScreenBuffer+6*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));
    tfp_sprintf(string_buffer,"      PITCH     UP    ");
    memcpy(ScreenBuffer+7*SCREEN_CHARACTER_COLUMN_COUNT,string_buffer,strlen(string_buffer));

    uint8_t row;
    for (row = 0; row <SCREEN_CHARACTER_ROW_COUNT; row++) {
      i2c_OLED_set_line(row);
      i2c_OLED_send_line(ScreenBuffer+row*SCREEN_CHARACTER_COLUMN_COUNT);
    }
    // clear screen bufffer for next round
    memset(&ScreenBuffer, ' ', sizeof(ScreenBuffer));
    ScreenBuffer[sizeof(ScreenBuffer)-1]=0;

    nextDisplayUpdateAt = micros() + 10 * MICROSECONDS_IN_A_SECOND;
}
#endif
