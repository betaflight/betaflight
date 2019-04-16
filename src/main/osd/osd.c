/*
 * This file is part of Cleanflight and Betaflight.
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

/*
 Created by Marcin Baliniak
 some functions based on MinimOSD

 OSD-CMS separation by jflyper
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#ifdef USE_OSD

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "build/build_config.h"
#include "build/version.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/display.h"
#include "drivers/flash.h"
#include "drivers/max7456_symbols.h"
#include "drivers/sdcard.h"
#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gps.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/esc_sensor.h"
#include "sensors/sensors.h"
#if defined(USE_GYRO_DATA_ANALYSE)
#include "sensors/gyroanalyse.h"
#endif

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

const char * const osdTimerSourceNames[] = {
    "ON TIME  ",
    "TOTAL ARM",
    "LAST ARM "
};

// Things in both OSD and CMS

#define IS_HI(X)  (rcData[X] > 1750)
#define IS_LO(X)  (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

timeUs_t osdFlyTime = 0;
#if defined(USE_ACC)
float osdGForce = 0;
#endif

static bool showVisualBeeper = false;

static statistic_t stats;
timeUs_t resumeRefreshAt = 0;
#define REFRESH_1S    1000 * 1000

static uint8_t armState;
#ifdef USE_OSD_PROFILES
static uint8_t osdProfile = 1;
#endif
static displayPort_t *osdDisplayPort;

static bool suppressStatsDisplay = false;

#ifdef USE_ESC_SENSOR
escSensorData_t *osdEscDataCombined;
#endif

PG_REGISTER_WITH_RESET_FN(osdConfig_t, osdConfig, PG_OSD_CONFIG, 5);

void osdStatSetState(uint8_t statIndex, bool enabled)
{
    if (enabled) {
        osdConfigMutable()->enabled_stats |= (1 << statIndex);
    } else {
        osdConfigMutable()->enabled_stats &= ~(1 << statIndex);
    }
}

bool osdStatGetState(uint8_t statIndex)
{
    return osdConfig()->enabled_stats & (1 << statIndex);
}

void osdWarnSetState(uint8_t warningIndex, bool enabled)
{
    if (enabled) {
        osdConfigMutable()->enabledWarnings |= (1 << warningIndex);
    } else {
        osdConfigMutable()->enabledWarnings &= ~(1 << warningIndex);
    }
}

bool osdWarnGetState(uint8_t warningIndex)
{
    return osdConfig()->enabledWarnings & (1 << warningIndex);
}

#ifdef USE_OSD_PROFILES
void setOsdProfile(uint8_t value)
{
    // 1 ->> 001
    // 2 ->> 010
    // 3 ->> 100
    if (value <= OSD_PROFILE_COUNT) {
        if (value == 0) {
            osdProfile = 1;
        } else {
            osdProfile = 1 << (value - 1);
        }
    }
 }

uint8_t getCurrentOsdProfileIndex(void)
{
    return osdConfig()->osdProfileIndex;
}

void changeOsdProfileIndex(uint8_t profileIndex)
{
    if (profileIndex <= OSD_PROFILE_COUNT) {
        osdConfigMutable()->osdProfileIndex = profileIndex;
        setOsdProfile(profileIndex);
        osdAnalyzeActiveElements();
    }
}
#endif

static void osdDrawElements(timeUs_t currentTimeUs)
{
    displayClearScreen(osdDisplayPort);

    // Hide OSD when OSDSW mode is active
    if (IS_RC_MODE_ACTIVE(BOXOSD)) {
        return;
    }

    osdDrawActiveElements(osdDisplayPort, currentTimeUs);
}

void pgResetFn_osdConfig(osdConfig_t *osdConfig)
{
    // Position elements near centre of screen and disabled by default
    for (int i = 0; i < OSD_ITEM_COUNT; i++) {
        osdConfig->item_pos[i] = OSD_POS(10, 7);
    }

    // Always enable warnings elements by default
    uint16_t profileFlags = 0;
    for (unsigned i = 1; i <= OSD_PROFILE_COUNT; i++) {
        profileFlags |= OSD_PROFILE_FLAG(i);
    }
    osdConfig->item_pos[OSD_WARNINGS] = OSD_POS(9, 10) | profileFlags;

    // Default to old fixed positions for these elements
    osdConfig->item_pos[OSD_CROSSHAIRS]         = OSD_POS(13, 6);
    osdConfig->item_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS(14, 2);
    osdConfig->item_pos[OSD_HORIZON_SIDEBARS]   = OSD_POS(14, 6);

    // Enable the default stats
    osdConfig->enabled_stats = 0; // reset all to off and enable only a few initially
    osdStatSetState(OSD_STAT_MAX_SPEED, true);
    osdStatSetState(OSD_STAT_MIN_BATTERY, true);
    osdStatSetState(OSD_STAT_MIN_RSSI, true);
    osdStatSetState(OSD_STAT_MAX_CURRENT, true);
    osdStatSetState(OSD_STAT_USED_MAH, true);
    osdStatSetState(OSD_STAT_BLACKBOX, true);
    osdStatSetState(OSD_STAT_BLACKBOX_NUMBER, true);
    osdStatSetState(OSD_STAT_TIMER_2, true);

    osdConfig->units = OSD_UNIT_METRIC;

    // Enable all warnings by default
    for (int i=0; i < OSD_WARNING_COUNT; i++) {
        osdWarnSetState(i, true);
    }
    // turn off RSSI & Link Quality warnings by default
    osdWarnSetState(OSD_WARNING_RSSI, false);
    osdWarnSetState(OSD_WARNING_LINK_QUALITY, false);

    osdConfig->timers[OSD_TIMER_1] = OSD_TIMER(OSD_TIMER_SRC_ON, OSD_TIMER_PREC_SECOND, 10);
    osdConfig->timers[OSD_TIMER_2] = OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_SECOND, 10);

    osdConfig->overlay_radio_mode = 2;

    osdConfig->rssi_alarm = 20;
    osdConfig->link_quality_alarm = 80;
    osdConfig->cap_alarm  = 2200;
    osdConfig->alt_alarm  = 100; // meters or feet depend on configuration
    osdConfig->esc_temp_alarm = ESC_TEMP_ALARM_OFF; // off by default
    osdConfig->esc_rpm_alarm = ESC_RPM_ALARM_OFF; // off by default
    osdConfig->esc_current_alarm = ESC_CURRENT_ALARM_OFF; // off by default
    osdConfig->core_temp_alarm = 70; // a temperature above 70C should produce a warning, lockups have been reported above 80C

    osdConfig->ahMaxPitch = 20; // 20 degrees
    osdConfig->ahMaxRoll = 40; // 40 degrees

    osdConfig->osdProfileIndex = 1;
    osdConfig->ahInvert = false;
}

static void osdDrawLogo(int x, int y)
{
    // display logo and help
    int fontOffset = 160;
    for (int row = 0; row < 4; row++) {
        for (int column = 0; column < 24; column++) {
            if (fontOffset <= SYM_END_OF_FONT)
                displayWriteChar(osdDisplayPort, x + column, y + row, fontOffset++);
        }
    }
}

void osdInit(displayPort_t *osdDisplayPortToUse)
{
    if (!osdDisplayPortToUse) {
        return;
    }

    STATIC_ASSERT(OSD_POS_MAX == OSD_POS(31,31), OSD_POS_MAX_incorrect);

    osdDisplayPort = osdDisplayPortToUse;
#ifdef USE_CMS
    cmsDisplayPortRegister(osdDisplayPort);
#endif

    armState = ARMING_FLAG(ARMED);

    osdResetAlarms();

    displayClearScreen(osdDisplayPort);

    osdDrawLogo(3, 1);

    char string_buffer[30];
    tfp_sprintf(string_buffer, "V%s", FC_VERSION_STRING);
    displayWrite(osdDisplayPort, 20, 6, string_buffer);
#ifdef USE_CMS
    displayWrite(osdDisplayPort, 7, 8,  CMS_STARTUP_HELP_TEXT1);
    displayWrite(osdDisplayPort, 11, 9, CMS_STARTUP_HELP_TEXT2);
    displayWrite(osdDisplayPort, 11, 10, CMS_STARTUP_HELP_TEXT3);
#endif

#ifdef USE_RTC_TIME
    char dateTimeBuffer[FORMATTED_DATE_TIME_BUFSIZE];
    if (osdFormatRtcDateTime(&dateTimeBuffer[0])) {
        displayWrite(osdDisplayPort, 5, 12, dateTimeBuffer);
    }
#endif

    displayResync(osdDisplayPort);

    resumeRefreshAt = micros() + (4 * REFRESH_1S);
#ifdef USE_OSD_PROFILES
    setOsdProfile(osdConfig()->osdProfileIndex);
#endif
    osdAnalyzeActiveElements();
}

bool osdInitialized(void)
{
    return osdDisplayPort;
}

static void osdResetStats(void)
{
    stats.max_current  = 0;
    stats.max_speed    = 0;
    stats.min_voltage  = 5000;
    stats.min_rssi     = 99; // percent
    stats.max_altitude = 0;
    stats.max_distance = 0;
    stats.armed_time   = 0;
    stats.max_g_force  = 0;
    stats.max_esc_temp = 0;
    stats.max_esc_rpm  = 0;
    stats.min_link_quality = 99; // percent
}

static void osdUpdateStats(void)
{
    int16_t value = 0;

#ifdef USE_GPS
    value = gpsSol.groundSpeed;
    if (stats.max_speed < value) {
        stats.max_speed = value;
    }
#endif

    value = getBatteryVoltage();
    if (stats.min_voltage > value) {
        stats.min_voltage = value;
    }

    value = getAmperage() / 100;
    if (stats.max_current < value) {
        stats.max_current = value;
    }

    value = getRssiPercent();
    if (stats.min_rssi > value) {
        stats.min_rssi = value;
    }

    int32_t altitudeCm = getEstimatedAltitudeCm();
    if (stats.max_altitude < altitudeCm) {
        stats.max_altitude = altitudeCm;
    }

#if defined(USE_ACC)
    if (stats.max_g_force < osdGForce) {
        stats.max_g_force = osdGForce;
    }
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
    value = rxGetLinkQualityPercent();
    if (stats.min_link_quality > value) {
        stats.min_link_quality = value;
    }
#endif

#ifdef USE_GPS
    if (STATE(GPS_FIX) && STATE(GPS_FIX_HOME)) {
        value = GPS_distanceToHome;

        if (stats.max_distance < GPS_distanceToHome) {
            stats.max_distance = GPS_distanceToHome;
        }
    }
#endif
#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        value = osdEscDataCombined->temperature;
        if (stats.max_esc_temp < value) {
            stats.max_esc_temp = value;
        }
        value = calcEscRpm(osdEscDataCombined->rpm);
        if (stats.max_esc_rpm < value) {
            stats.max_esc_rpm = value;
        }
    }
#endif
}

#ifdef USE_BLACKBOX

static void osdGetBlackboxStatusString(char * buff)
{
    bool storageDeviceIsWorking = isBlackboxDeviceWorking();
    uint32_t storageUsed = 0;
    uint32_t storageTotal = 0;

    switch (blackboxConfig()->device) {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        if (storageDeviceIsWorking) {
            storageTotal = sdcard_getMetadata()->numBlocks / 2000;
            storageUsed = storageTotal - (afatfs_getContiguousFreeSpace() / 1024000);
        }
        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        if (storageDeviceIsWorking) {
            const flashGeometry_t *geometry = flashfsGetGeometry();
            storageTotal = geometry->totalSize / 1024;
            storageUsed = flashfsGetOffset() / 1024;
        }
        break;
#endif

    default:
        break;
    }

    if (storageDeviceIsWorking) {
        const uint16_t storageUsedPercent = (storageUsed * 100) / storageTotal;
        tfp_sprintf(buff, "%d%%", storageUsedPercent);
    } else {
        tfp_sprintf(buff, "FAULT");
    }
}
#endif

static void osdDisplayStatisticLabel(uint8_t y, const char * text, const char * value)
{
    displayWrite(osdDisplayPort, 2, y, text);
    displayWrite(osdDisplayPort, 20, y, ":");
    displayWrite(osdDisplayPort, 22, y, value);
}

/*
 * Test if there's some stat enabled
 */
static bool isSomeStatEnabled(void)
{
    return (osdConfig()->enabled_stats != 0);
}

// *** IMPORTANT ***
// The stats display order was previously required to match the enumeration definition so it matched
// the order shown in the configurator. However, to allow reordering this screen without breaking the
// compatibility, this requirement has been relaxed to a best effort approach. Reordering the elements
// on the stats screen will have to be more beneficial than the hassle of not matching exactly to the
// configurator list.

static void osdShowStats(uint16_t endBatteryVoltage)
{
    uint8_t top = 2;
    char buff[OSD_ELEMENT_BUFFER_LENGTH];

    displayClearScreen(osdDisplayPort);
    displayWrite(osdDisplayPort, 2, top++, "  --- STATS ---");

    if (osdStatGetState(OSD_STAT_RTC_DATE_TIME)) {
        bool success = false;
#ifdef USE_RTC_TIME
        success = osdFormatRtcDateTime(&buff[0]);
#endif
        if (!success) {
            tfp_sprintf(buff, "NO RTC");
        }

        displayWrite(osdDisplayPort, 2, top++, buff);
    }

    if (osdStatGetState(OSD_STAT_TIMER_1)) {
        osdFormatTimer(buff, false, (OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1]) == OSD_TIMER_SRC_ON ? false : true), OSD_TIMER_1);
        osdDisplayStatisticLabel(top++, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1])], buff);
    }

    if (osdStatGetState(OSD_STAT_TIMER_2)) {
        osdFormatTimer(buff, false, (OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2]) == OSD_TIMER_SRC_ON ? false : true), OSD_TIMER_2);
        osdDisplayStatisticLabel(top++, osdTimerSourceNames[OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2])], buff);
    }

    if (osdStatGetState(OSD_STAT_MAX_ALTITUDE)) {
        const int alt = osdGetMetersToSelectedUnit(stats.max_altitude) / 10;
        tfp_sprintf(buff, "%d.%d%c", alt / 10, alt % 10, osdGetMetersToSelectedUnitSymbol());
        osdDisplayStatisticLabel(top++, "MAX ALTITUDE", buff);
    }

#ifdef USE_GPS
    if (featureIsEnabled(FEATURE_GPS)) {
        if (osdStatGetState(OSD_STAT_MAX_SPEED)) {
            tfp_sprintf(buff, "%d%c", osdGetSpeedToSelectedUnit(stats.max_speed), osdGetSpeedToSelectedUnitSymbol());
            osdDisplayStatisticLabel(top++, "MAX SPEED", buff);
        }

        if (osdStatGetState(OSD_STAT_MAX_DISTANCE)) {
            tfp_sprintf(buff, "%d%c", osdGetMetersToSelectedUnit(stats.max_distance), osdGetMetersToSelectedUnitSymbol());
            osdDisplayStatisticLabel(top++, "MAX DISTANCE", buff);
        }

        if (osdStatGetState(OSD_STAT_FLIGHT_DISTANCE)) {
            const uint32_t distanceFlown = GPS_distanceFlownInCm / 100;
            tfp_sprintf(buff, "%d%c", osdGetMetersToSelectedUnit(distanceFlown), osdGetMetersToSelectedUnitSymbol());
            osdDisplayStatisticLabel(top++, "FLIGHT DISTANCE", buff);
        }
    }
#endif

    if (osdStatGetState(OSD_STAT_MIN_BATTERY)) {
        tfp_sprintf(buff, "%d.%02d%c", stats.min_voltage / 100, stats.min_voltage % 100, SYM_VOLT);
        osdDisplayStatisticLabel(top++, "MIN BATTERY", buff);
    }

    if (osdStatGetState(OSD_STAT_END_BATTERY)) {
        tfp_sprintf(buff, "%d.%02d%c", endBatteryVoltage / 100, endBatteryVoltage % 100, SYM_VOLT);
        osdDisplayStatisticLabel(top++, "END BATTERY", buff);
    }

    if (osdStatGetState(OSD_STAT_BATTERY)) {
        tfp_sprintf(buff, "%d.%02d%c", getBatteryVoltage() / 100, getBatteryVoltage() % 100, SYM_VOLT);
        osdDisplayStatisticLabel(top++, "BATTERY", buff);
    }

    if (osdStatGetState(OSD_STAT_MIN_RSSI)) {
        itoa(stats.min_rssi, buff, 10);
        strcat(buff, "%");
        osdDisplayStatisticLabel(top++, "MIN RSSI", buff);
    }

    if (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) {
        if (osdStatGetState(OSD_STAT_MAX_CURRENT)) {
            itoa(stats.max_current, buff, 10);
            strcat(buff, "A");
            osdDisplayStatisticLabel(top++, "MAX CURRENT", buff);
        }

        if (osdStatGetState(OSD_STAT_USED_MAH)) {
            tfp_sprintf(buff, "%d%c", getMAhDrawn(), SYM_MAH);
            osdDisplayStatisticLabel(top++, "USED MAH", buff);
        }
    }

#ifdef USE_BLACKBOX
    if (osdStatGetState(OSD_STAT_BLACKBOX) && blackboxConfig()->device && blackboxConfig()->device != BLACKBOX_DEVICE_SERIAL) {
        osdGetBlackboxStatusString(buff);
        osdDisplayStatisticLabel(top++, "BLACKBOX", buff);
    }

    if (osdStatGetState(OSD_STAT_BLACKBOX_NUMBER) && blackboxConfig()->device && blackboxConfig()->device != BLACKBOX_DEVICE_SERIAL) {
        itoa(blackboxGetLogNumber(), buff, 10);
        osdDisplayStatisticLabel(top++, "BB LOG NUM", buff);
    }
#endif

#if defined(USE_ACC)
    if (osdStatGetState(OSD_STAT_MAX_G_FORCE) && sensors(SENSOR_ACC)) {
        const int gForce = lrintf(stats.max_g_force * 10);
        tfp_sprintf(buff, "%01d.%01dG", gForce / 10, gForce % 10);
        osdDisplayStatisticLabel(top++, "MAX G-FORCE", buff);
    }
#endif

#ifdef USE_ESC_SENSOR
    if (osdStatGetState(OSD_STAT_MAX_ESC_TEMP)) {
        tfp_sprintf(buff, "%d%c", osdConvertTemperatureToSelectedUnit(stats.max_esc_temp), osdGetTemperatureSymbolForSelectedUnit());
        osdDisplayStatisticLabel(top++, "MAX ESC TEMP", buff);
    }

    if (osdStatGetState(OSD_STAT_MAX_ESC_RPM)) {
        itoa(stats.max_esc_rpm, buff, 10);
        osdDisplayStatisticLabel(top++, "MAX ESC RPM", buff);
    }
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
    if (osdStatGetState(OSD_STAT_MIN_LINK_QUALITY)) {
        itoa(stats.min_link_quality, buff, 10);
        strcat(buff, "%");
        osdDisplayStatisticLabel(top++, "MIN LINK", buff);
    }
#endif

#if defined(USE_GYRO_DATA_ANALYSE)
    if (osdStatGetState(OSD_STAT_MAX_FFT) && featureIsEnabled(FEATURE_DYNAMIC_FILTER)) {
        int value = getMaxFFT();
        if (value > 0) {
            tfp_sprintf(buff, "%dHZ", value);
            osdDisplayStatisticLabel(top++, "PEAK FFT", buff);
        } else {
            osdDisplayStatisticLabel(top++, "PEAK FFT", "THRT<20%");
        }
    }
#endif
}

static void osdShowArmed(void)
{
    displayClearScreen(osdDisplayPort);
    displayWrite(osdDisplayPort, 12, 7, "ARMED");
}

STATIC_UNIT_TESTED void osdRefresh(timeUs_t currentTimeUs)
{
    static timeUs_t lastTimeUs = 0;
    static bool osdStatsEnabled = false;
    static bool osdStatsVisible = false;
    static timeUs_t osdStatsRefreshTimeUs;
    static uint16_t endBatteryVoltage;

    // detect arm/disarm
    if (armState != ARMING_FLAG(ARMED)) {
        if (ARMING_FLAG(ARMED)) {
            osdStatsEnabled = false;
            osdStatsVisible = false;
            osdResetStats();
            osdShowArmed();
            resumeRefreshAt = currentTimeUs + (REFRESH_1S / 2);
        } else if (isSomeStatEnabled()
                   && !suppressStatsDisplay
                   && (!(getArmingDisableFlags() & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED))
                       || !VISIBLE(osdConfig()->item_pos[OSD_WARNINGS]))) { // suppress stats if runaway takeoff triggered disarm and WARNINGS element is visible
            osdStatsEnabled = true;
            resumeRefreshAt = currentTimeUs + (60 * REFRESH_1S);
            endBatteryVoltage = getBatteryVoltage();
        }

        armState = ARMING_FLAG(ARMED);
    }


    if (ARMING_FLAG(ARMED)) {
        osdUpdateStats();
        timeUs_t deltaT = currentTimeUs - lastTimeUs;
        osdFlyTime += deltaT;
        stats.armed_time += deltaT;
    } else if (osdStatsEnabled) {  // handle showing/hiding stats based on OSD disable switch position
        if (displayIsGrabbed(osdDisplayPort)) {
            osdStatsEnabled = false;
            resumeRefreshAt = 0;
            stats.armed_time = 0;
        } else {
            if (IS_RC_MODE_ACTIVE(BOXOSD) && osdStatsVisible) {
                osdStatsVisible = false;
                displayClearScreen(osdDisplayPort);
            } else if (!IS_RC_MODE_ACTIVE(BOXOSD)) {
                if (!osdStatsVisible) {
                    osdStatsVisible = true;
                    osdStatsRefreshTimeUs = 0;
                }
                if (currentTimeUs >= osdStatsRefreshTimeUs) {
                    osdStatsRefreshTimeUs = currentTimeUs + REFRESH_1S;
                    osdShowStats(endBatteryVoltage);
                }
            }
        }
    }
    lastTimeUs = currentTimeUs;

    if (resumeRefreshAt) {
        if (cmp32(currentTimeUs, resumeRefreshAt) < 0) {
            // in timeout period, check sticks for activity to resume display.
            if (IS_HI(THROTTLE) || IS_HI(PITCH)) {
                resumeRefreshAt = currentTimeUs;
            }
            displayHeartbeat(osdDisplayPort);
            return;
        } else {
            displayClearScreen(osdDisplayPort);
            resumeRefreshAt = 0;
            osdStatsEnabled = false;
            stats.armed_time = 0;
        }
    }

#ifdef USE_ESC_SENSOR
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        osdEscDataCombined = getEscSensorData(ESC_SENSOR_COMBINED);
    }
#endif

#if defined(USE_ACC)
    if (sensors(SENSOR_ACC)
       && (VISIBLE(osdConfig()->item_pos[OSD_G_FORCE]) || osdStatGetState(OSD_STAT_MAX_G_FORCE))) {
            // only calculate the G force if the element is visible or the stat is enabled
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            const float a = accAverage[axis];
            osdGForce += a * a;
        }
        osdGForce = sqrtf(osdGForce) * acc.dev.acc_1G_rec;
    }
#endif

#ifdef USE_CMS
    if (!displayIsGrabbed(osdDisplayPort))
#endif
    {
        osdUpdateAlarms();
        osdDrawElements(currentTimeUs);
        displayHeartbeat(osdDisplayPort);
    }
}

/*
 * Called periodically by the scheduler
 */
void osdUpdate(timeUs_t currentTimeUs)
{
    static uint32_t counter = 0;

    if (isBeeperOn()) {
        showVisualBeeper = true;
    }

#ifdef MAX7456_DMA_CHANNEL_TX
    // don't touch buffers if DMA transaction is in progress
    if (displayIsTransferInProgress(osdDisplayPort)) {
        return;
    }
#endif // MAX7456_DMA_CHANNEL_TX

#ifdef USE_SLOW_MSP_DISPLAYPORT_RATE_WHEN_UNARMED
    static uint32_t idlecounter = 0;
    if (!ARMING_FLAG(ARMED)) {
        if (idlecounter++ % 4 != 0) {
            return;
        }
    }
#endif

    // redraw values in buffer
#ifdef USE_MAX7456
#define DRAW_FREQ_DENOM 5
#else
#define DRAW_FREQ_DENOM 10 // MWOSD @ 115200 baud (
#endif
#define STATS_FREQ_DENOM    50

    if (counter % DRAW_FREQ_DENOM == 0) {
        osdRefresh(currentTimeUs);
        showVisualBeeper = false;
    } else {
        // rest of time redraw screen 10 chars per idle so it doesn't lock the main idle
        displayDrawScreen(osdDisplayPort);
    }
    ++counter;
}

void osdSuppressStats(bool flag)
{
    suppressStatsDisplay = flag;
}

#ifdef USE_OSD_PROFILES
bool osdElementVisible(uint16_t value)
{
    return (bool)((((value & OSD_PROFILE_MASK) >> OSD_PROFILE_BITS_POS) & osdProfile) != 0);
}
#endif

bool osdGetVisualBeeperState(void)
{
    return showVisualBeeper;
}

statistic_t *osdGetStats(void)
{
    return &stats;
}
#endif // USE_OSD
