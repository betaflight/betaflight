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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "blackbox/blackbox.h"

    #include "pg/pg_ids.h"

    #include "common/time.h"

    #include "drivers/max7456_symbols.h"
    #include "drivers/serial.h"

    #include "fc/config.h"
    #include "fc/fc_core.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/pid.h"
    #include "flight/imu.h"

    #include "io/gps.h"
    #include "io/osd.h"

    #include "sensors/battery.h"

    #include "rx/rx.h"

    void osdRefresh(timeUs_t currentTimeUs);
    void osdFormatTime(char * buff, osd_timer_precision_e precision, timeUs_t time);
    void osdFormatTimer(char *buff, bool showSymbol, int timerIndex);
    int osdConvertTemperatureToSelectedUnit(int tempInDeciDegrees);

    uint16_t rssi;
    attitudeEulerAngles_t attitude;
    pidProfile_t *currentPidProfile;
    int16_t debug[DEBUG16_VALUE_COUNT];
    int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    uint8_t GPS_numSat;
    uint16_t GPS_distanceToHome;
    int16_t GPS_directionToHome;
    int32_t GPS_coord[2];
    gpsSolutionData_t gpsSol;

    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);

    timeUs_t simulationTime = 0;
    batteryState_e simulationBatteryState;
    uint8_t simulationBatteryCellCount;
    uint16_t simulationBatteryVoltage;
    uint32_t simulationBatteryAmperage;
    uint32_t simulationMahDrawn;
    int32_t simulationAltitude;
    int32_t simulationVerticalSpeed;
    uint16_t simulationCoreTemperature;
}

/* #define DEBUG_OSD */

#include "unittest_macros.h"
#include "unittest_displayport.h"
#include "gtest/gtest.h"

void setDefualtSimulationState()
{
    rssi = 1024;

    simulationBatteryState = BATTERY_OK;
    simulationBatteryCellCount = 4;
    simulationBatteryVoltage = 168;
    simulationBatteryAmperage = 0;
    simulationMahDrawn = 0;
    simulationAltitude = 0;
    simulationVerticalSpeed = 0;
    simulationCoreTemperature = 0;
}

/*
 * Performs a test of the OSD actions on arming.
 * (reused throughout the test suite)
 */
void doTestArm(bool testEmpty = true)
{
    // given
    // craft has been armed
    ENABLE_ARMING_FLAG(ARMED);

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // arming alert displayed
    displayPortTestBufferSubstring(12, 7, "ARMED");

    // given
    // armed alert times out (0.5 seconds)
    simulationTime += 0.5e6;

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // arming alert disappears
#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif
    if (testEmpty) {
        displayPortTestBufferIsEmpty();
    }
}

/*
 * Auxiliary function. Test is there're stats that must be shown
 */
bool isSomeStatEnabled(void) {
    for (int i = 0; i < OSD_STAT_COUNT; i++) {
        if (osdConfigMutable()->enabled_stats[i]) {
            return true;
        }
    }
    return false;
}

/*
 * Performs a test of the OSD actions on disarming.
 * (reused throughout the test suite)
 */
void doTestDisarm()
{
    // given
    // craft is disarmed after having been armed
    DISABLE_ARMING_FLAG(ARMED);

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // post flight statistics displayed
    if (isSomeStatEnabled()) {
        displayPortTestBufferSubstring(2, 2, "  --- STATS ---");
    }
}

/*
 * Tests initialisation of the OSD and the power on splash screen.
 */
TEST(OsdTest, TestInit)
{
    // given
    // display port is initialised
    displayPortTestInit();

    // and
    // default state values are set
    setDefualtSimulationState();

    // and
    // this battery configuration (used for battery voltage elements)
    batteryConfigMutable()->vbatmincellvoltage = 33;
    batteryConfigMutable()->vbatmaxcellvoltage = 43;

    // when
    // OSD is initialised
    osdInit(&testDisplayPort);

    // then
    // display buffer should contain splash screen
    displayPortTestBufferSubstring(7, 8, "MENU:THR MID");
    displayPortTestBufferSubstring(11, 9, "+ YAW LEFT");
    displayPortTestBufferSubstring(11, 10, "+ PITCH UP");

    // when
    // splash screen timeout has elapsed
    simulationTime += 4e6;
    osdUpdate(simulationTime);

    // then
    // display buffer should be empty
#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif
    displayPortTestBufferIsEmpty();
}

/*
 * Tests visibility of the ARMED notification after arming.
 */
TEST(OsdTest, TestArm)
{
    doTestArm();
}

/*
 * Tests display and timeout of the post flight statistics screen after disarming.
 */
TEST(OsdTest, TestDisarm)
{
    doTestDisarm();

    // given
    // post flight stats times out (60 seconds)
    simulationTime += 60e6;

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);

    // then
    // post flight stats screen disappears
#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif
    displayPortTestBufferIsEmpty();
}

/*
 * Tests disarming and immediately rearming clears post flight stats and shows ARMED notification.
 */
TEST(OsdTest, TestDisarmWithImmediateRearm)
{
    doTestArm();
    doTestDisarm();
    doTestArm();
}

/*
 * Tests dismissing the statistics screen with pitch stick after disarming.
 */
TEST(OsdTest, TestDisarmWithDismissStats)
{
    // Craft is alread armed after previous test

    doTestDisarm();

    // given
    // sticks have been moved
    rcData[PITCH] = 1800;

    // when
    // sufficient OSD updates have been called
    osdRefresh(simulationTime);
    osdRefresh(simulationTime);

    // then
    // post flight stats screen disappears
#ifdef DEBUG_OSD
    displayPortTestPrint();
#endif
    displayPortTestBufferIsEmpty();

    rcData[PITCH] = 1500;
}

/*
 * Tests the calculation of statistics with imperial unit output.
 */
TEST(OsdTest, TestStatsImperial)
{
    // given
    // this set of enabled post flight statistics
    osdConfigMutable()->enabled_stats[OSD_STAT_MAX_SPEED]       = true;
    osdConfigMutable()->enabled_stats[OSD_STAT_MIN_BATTERY]     = true;
    osdConfigMutable()->enabled_stats[OSD_STAT_MIN_RSSI]        = true;
    osdConfigMutable()->enabled_stats[OSD_STAT_MAX_CURRENT]     = false;
    osdConfigMutable()->enabled_stats[OSD_STAT_USED_MAH]        = false;
    osdConfigMutable()->enabled_stats[OSD_STAT_MAX_ALTITUDE]    = true;
    osdConfigMutable()->enabled_stats[OSD_STAT_BLACKBOX]        = false;
    osdConfigMutable()->enabled_stats[OSD_STAT_END_BATTERY]     = true;
    osdConfigMutable()->enabled_stats[OSD_STAT_TIMER_1]         = true;
    osdConfigMutable()->enabled_stats[OSD_STAT_TIMER_2]         = true;
    osdConfigMutable()->enabled_stats[OSD_STAT_RTC_DATE_TIME]   = true;
    osdConfigMutable()->enabled_stats[OSD_STAT_MAX_DISTANCE]    = true;
    osdConfigMutable()->enabled_stats[OSD_STAT_BLACKBOX_NUMBER] = false;

    // and
    // using imperial unit system
    osdConfigMutable()->units = OSD_UNIT_IMPERIAL;

    // and
    // this timer 1 configuration
    osdConfigMutable()->timers[OSD_TIMER_1] = OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_HUNDREDTHS, 0);

    // and
    // this timer 2 configuration
    osdConfigMutable()->timers[OSD_TIMER_2] = OSD_TIMER(OSD_TIMER_SRC_LAST_ARMED, OSD_TIMER_PREC_SECOND, 0);

    // and
    // a GPS fix is present
    stateFlags |= GPS_FIX | GPS_FIX_HOME;

    // and
    // this RTC time
    dateTime_t dateTime;
    dateTime.year = 2017;
    dateTime.month = 11;
    dateTime.day = 19;
    dateTime.hours = 10;
    dateTime.minutes = 12;
    dateTime.seconds = 0;
    dateTime.millis = 0;
    rtcSetDateTime(&dateTime);

    // when
    // the craft is armed
    doTestArm();

    // and
    // these conditions occur during flight
    rssi = 1024;
    gpsSol.groundSpeed = 500;
    GPS_distanceToHome = 20;
    simulationBatteryVoltage = 158;
    simulationAltitude = 100;
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    rssi = 512;
    gpsSol.groundSpeed = 800;
    GPS_distanceToHome = 50;
    simulationBatteryVoltage = 147;
    simulationAltitude = 150;
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    rssi = 256;
    gpsSol.groundSpeed = 200;
    GPS_distanceToHome = 100;
    simulationBatteryVoltage = 152;
    simulationAltitude = 200;
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    // and
    // the craft is disarmed
    doTestDisarm();

    // then
    // statistics screen should display the following
    int row = 3;
    displayPortTestBufferSubstring(2, row++, "2017-11-19 10:12:");
    displayPortTestBufferSubstring(2, row++, "TOTAL ARM         : 00:05.00");
    displayPortTestBufferSubstring(2, row++, "LAST ARM          : 00:03");
    displayPortTestBufferSubstring(2, row++, "MAX SPEED         : 28");
    displayPortTestBufferSubstring(2, row++, "MAX DISTANCE      : 328%c", SYM_FT);
    displayPortTestBufferSubstring(2, row++, "MIN BATTERY       : 14.7%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "END BATTERY       : 15.2%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "MIN RSSI          : 25%%");
    displayPortTestBufferSubstring(2, row++, "MAX ALTITUDE      : 6.5%c", SYM_FT);
}

/*
 * Tests the calculation of statistics with metric unit output.
 * (essentially an abridged version of the previous test
 */
TEST(OsdTest, TestStatsMetric)
{
    // given
    // using metric unit system
    osdConfigMutable()->units = OSD_UNIT_METRIC;

    // and
    // default state values are set
    setDefualtSimulationState();

    // when
    // the craft is armed
    doTestArm();

    // and
    // these conditions occur during flight (simplified to less assignments than previous test)
    rssi = 256;
    gpsSol.groundSpeed = 800;
    GPS_distanceToHome = 100;
    simulationBatteryVoltage = 147;
    simulationAltitude = 200;
    simulationTime += 1e6;
    osdRefresh(simulationTime);
    osdRefresh(simulationTime);

    simulationBatteryVoltage = 152;
    simulationTime += 1e6;
    osdRefresh(simulationTime);

    // and
    // the craft is disarmed
    doTestDisarm();

    // then
    // statistics screen should display the following
    int row = 3;
    displayPortTestBufferSubstring(2, row++, "2017-11-19 10:12:");
    displayPortTestBufferSubstring(2, row++, "TOTAL ARM         : 00:07.50");
    displayPortTestBufferSubstring(2, row++, "LAST ARM          : 00:02");
    displayPortTestBufferSubstring(2, row++, "MAX SPEED         : 28");
    displayPortTestBufferSubstring(2, row++, "MAX DISTANCE      : 100%c", SYM_M);
    displayPortTestBufferSubstring(2, row++, "MIN BATTERY       : 14.7%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "END BATTERY       : 15.2%c", SYM_VOLT);
    displayPortTestBufferSubstring(2, row++, "MIN RSSI          : 25%%");
    displayPortTestBufferSubstring(2, row++, "MAX ALTITUDE      : 2.0%c", SYM_M);
}

/*
 * Tests activation of alarms and element flashing.
 */
TEST(OsdTest, TestAlarms)
{
    // given
    // default state is set
    setDefualtSimulationState();

    // and
    // the following OSD elements are visible
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]              = OSD_POS(8, 1)  | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE]       = OSD_POS(12, 1) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_1]            = OSD_POS(20, 1) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]            = OSD_POS(1, 1)  | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_REMAINING_TIME_ESTIMATE] = OSD_POS(1, 2) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ALTITUDE]                = OSD_POS(23, 7) | VISIBLE_FLAG;

    // and
    // this set of alarm values
    osdConfigMutable()->rssi_alarm = 20;
    osdConfigMutable()->cap_alarm  = 2200;
    osdConfigMutable()->alt_alarm  = 100; // meters

    // and
    // this timer 1 configuration
    osdConfigMutable()->timers[OSD_TIMER_1] = OSD_TIMER(OSD_TIMER_SRC_ON, OSD_TIMER_PREC_HUNDREDTHS, 2);
    EXPECT_EQ(OSD_TIMER_SRC_ON, OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_1]));
    EXPECT_EQ(OSD_TIMER_PREC_HUNDREDTHS, OSD_TIMER_PRECISION(osdConfig()->timers[OSD_TIMER_1]));
    EXPECT_EQ(2, OSD_TIMER_ALARM(osdConfig()->timers[OSD_TIMER_1]));

    // and
    // this timer 2 configuration
    osdConfigMutable()->timers[OSD_TIMER_2] = OSD_TIMER(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_PREC_SECOND, 1);
    EXPECT_EQ(OSD_TIMER_SRC_TOTAL_ARMED, OSD_TIMER_SRC(osdConfig()->timers[OSD_TIMER_2]));
    EXPECT_EQ(OSD_TIMER_PREC_SECOND, OSD_TIMER_PRECISION(osdConfig()->timers[OSD_TIMER_2]));
    EXPECT_EQ(1, OSD_TIMER_ALARM(osdConfig()->timers[OSD_TIMER_2]));

    // and
    // using the metric unit system
    osdConfigMutable()->units = OSD_UNIT_METRIC;

    // when
    // the craft is armed
    doTestArm(false);

    // then
    // no elements should flash as all values are out of alarm range
    for (int i = 0; i < 30; i++) {
        // Check for visibility every 100ms, elements should always be visible
        simulationTime += 0.1e6;
        osdRefresh(simulationTime);

#ifdef DEBUG_OSD
        printf("%d\n", i);
#endif
        displayPortTestBufferSubstring(8,  1, "%c99", SYM_RSSI);
        displayPortTestBufferSubstring(12, 1, "%c16.8%c", SYM_BATT_FULL, SYM_VOLT);
        displayPortTestBufferSubstring(1,  1, "%c00:", SYM_FLY_M); // only test the minute part of the timer
        displayPortTestBufferSubstring(20, 1, "%c01:", SYM_ON_M); // only test the minute part of the timer
        displayPortTestBufferSubstring(23, 7, "   0.0%c", SYM_M);
    }

    // when
    // all values are out of range
    rssi = 128;
    simulationBatteryState = BATTERY_CRITICAL;
    simulationBatteryVoltage = 135;
    simulationAltitude = 12000;
    simulationTime += 60e6;
    osdRefresh(simulationTime);
    simulationMahDrawn = 999999;

    // then
    // elements showing values in alarm range should flash
    for (int i = 0; i < 15; i++) {
        // Blinking should happen at 5Hz
        simulationTime += 0.2e6;
        osdRefresh(simulationTime);

#ifdef DEBUG_OSD
        printf("%d\n", i);
        displayPortTestPrint();
#endif
        if (i % 2 == 0) {
            displayPortTestBufferSubstring(8,  1, "%c12", SYM_RSSI);
            displayPortTestBufferSubstring(12, 1, "%c13.5%c", SYM_MAIN_BATT, SYM_VOLT);
            displayPortTestBufferSubstring(1,  1, "%c01:", SYM_FLY_M); // only test the minute part of the timer
            displayPortTestBufferSubstring(20, 1, "%c02:", SYM_ON_M); // only test the minute part of the timer
            displayPortTestBufferSubstring(23, 7, " 120.0%c", SYM_M);
        } else {
            displayPortTestBufferIsEmpty();
        }
    }
}

/*
 * Tests the RSSI OSD element.
 */
TEST(OsdTest, TestElementRssi)
{
    // given
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE] = OSD_POS(8, 1) | VISIBLE_FLAG;
    osdConfigMutable()->rssi_alarm = 0;

    // when
    rssi = 1024;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(8, 1, "%c99", SYM_RSSI);

    // when
    rssi = 0;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(8, 1, "%c 0", SYM_RSSI);

    // when
    rssi = 512;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(8, 1, "%c50", SYM_RSSI);
}

/*
 * Tests the instantaneous battery current OSD element.
 */
TEST(OsdTest, TestElementAmperage)
{
    // given
    osdConfigMutable()->item_pos[OSD_CURRENT_DRAW] = OSD_POS(1, 12) | VISIBLE_FLAG;

    // when
    simulationBatteryAmperage = 0;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 12, "  0.00%c", SYM_AMP);

    // when
    simulationBatteryAmperage = 2156;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 12, " 21.56%c", SYM_AMP);

    // when
    simulationBatteryAmperage = 12345;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 12, "123.45%c", SYM_AMP);
}

/*
 * Tests the battery capacity drawn OSD element.
 */
TEST(OsdTest, TestElementMahDrawn)
{
    // given
    osdConfigMutable()->item_pos[OSD_MAH_DRAWN] = OSD_POS(1, 11) | VISIBLE_FLAG;

    // when
    simulationMahDrawn = 0;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, "   0%c", SYM_MAH);

    // when
    simulationMahDrawn = 4;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, "   4%c", SYM_MAH);

    // when
    simulationMahDrawn = 15;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, "  15%c", SYM_MAH);

    // when
    simulationMahDrawn = 246;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, " 246%c", SYM_MAH);

    // when
    simulationMahDrawn = 1042;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 11, "1042%c", SYM_MAH);
}

/*
 * Tests the instantaneous electrical power OSD element.
 */
TEST(OsdTest, TestElementPower)
{
    // given
    osdConfigMutable()->item_pos[OSD_POWER] = OSD_POS(1, 10)  | VISIBLE_FLAG;

    // and
    simulationBatteryVoltage = 100; // 10V

    // and
    simulationBatteryAmperage = 0; // 0A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, "   0W");

    // given
    simulationBatteryAmperage = 10; // 0.1A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, "   1W");

    // given
    simulationBatteryAmperage = 120; // 1.2A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, "  12W");

    // given
    simulationBatteryAmperage = 1230; // 12.3A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, " 123W");

    // given
    simulationBatteryAmperage = 12340; // 123.4A

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 10, "1234W");
}

/*
 * Tests the altitude OSD element.
 */
TEST(OsdTest, TestElementAltitude)
{
    // given
    osdConfigMutable()->item_pos[OSD_ALTITUDE] = OSD_POS(23, 7) | VISIBLE_FLAG;

    // and
    osdConfigMutable()->units = OSD_UNIT_METRIC;

    // when
    simulationAltitude = 0;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "   0.0%c", SYM_M);

    // when
    simulationAltitude = 247;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "   2.4%c", SYM_M);

    // when
    simulationAltitude = 4247;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "  42.4%c", SYM_M);

    // when
    simulationAltitude = -247;
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(23, 7, "  -2.4%c", SYM_M);
}

/*
 * Tests the core temperature OSD element.
 */
TEST(OsdTest, TestElementCoreTemperature)
{
    // given
    osdConfigMutable()->item_pos[OSD_CORE_TEMPERATURE] = OSD_POS(1, 8) | VISIBLE_FLAG;

    // and
    osdConfigMutable()->units = OSD_UNIT_METRIC;

    // and
    simulationCoreTemperature = 0;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 8, "  0C");

    // given
    simulationCoreTemperature = 33;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 8, " 33C");

    // given
    osdConfigMutable()->units = OSD_UNIT_IMPERIAL;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(1, 8, " 91F");
}

/*
 * Tests the battery notifications shown on the warnings OSD element.
 */
TEST(OsdTest, TestElementWarningsBattery)
{
    // given
    osdConfigMutable()->item_pos[OSD_WARNINGS] = OSD_POS(9, 10) | VISIBLE_FLAG;
    osdConfigMutable()->enabledWarnings = OSD_WARNING_BATTERY_WARNING | OSD_WARNING_BATTERY_CRITICAL | OSD_WARNING_BATTERY_NOT_FULL;

    // and
    batteryConfigMutable()->vbatfullcellvoltage = 41;

    // and
    // 4S battery
    simulationBatteryCellCount = 4;

    // and
    // full battery
    simulationBatteryVoltage = 168;
    simulationBatteryState = BATTERY_OK;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, "           ");

    // given
    // low battery
    simulationBatteryVoltage = 140;
    simulationBatteryState = BATTERY_WARNING;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, "LOW BATTERY ");

    // given
    // crtical battery
    simulationBatteryVoltage = 132;
    simulationBatteryState = BATTERY_CRITICAL;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, " LAND NOW   ");

    // given
    // used battery
    simulationBatteryVoltage = ((batteryConfig()->vbatmaxcellvoltage - 2) * simulationBatteryCellCount) - 1;
    simulationBatteryState = BATTERY_OK;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, "BATT < FULL");

    // given
    // full battery
    simulationBatteryVoltage = ((batteryConfig()->vbatmaxcellvoltage - 2) * simulationBatteryCellCount);
    simulationBatteryState = BATTERY_OK;

    // when
    displayClearScreen(&testDisplayPort);
    osdRefresh(simulationTime);

    // then
    displayPortTestBufferSubstring(9, 10, "             ");

    // TODO
}

/*
 * Tests the time string formatting function with a series of precision settings and time values.
 */
TEST(OsdTest, TestFormatTimeString)
{
    char buff[OSD_ELEMENT_BUFFER_LENGTH];

    /* Seconds precision, 0 us */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 0);
    EXPECT_EQ(0, strcmp("00:00", buff));

    /* Seconds precision, 0.9 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 0.9e6);
    EXPECT_EQ(0, strcmp("00:00", buff));

    /* Seconds precision, 10 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 10e6);
    EXPECT_EQ(0, strcmp("00:10", buff));

    /* Seconds precision, 1 minute */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 60e6);
    EXPECT_EQ(0, strcmp("01:00", buff));

    /* Seconds precision, 1 minute 59 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_SECOND, 119e6);
    EXPECT_EQ(0, strcmp("01:59", buff));

    /* Hundredths precision, 0 us */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 0);
    EXPECT_EQ(0, strcmp("00:00.00", buff));

    /* Hundredths precision, 10 milliseconds (one 100th of a second) */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 10e3);
    EXPECT_EQ(0, strcmp("00:00.01", buff));

    /* Hundredths precision, 0.9 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 0.9e6);
    EXPECT_EQ(0, strcmp("00:00.90", buff));

    /* Hundredths precision, 10 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 10e6);
    EXPECT_EQ(0, strcmp("00:10.00", buff));

    /* Hundredths precision, 1 minute */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 60e6);
    EXPECT_EQ(0, strcmp("01:00.00", buff));

    /* Hundredths precision, 1 minute 59 seconds */
    osdFormatTime(buff, OSD_TIMER_PREC_HUNDREDTHS, 119e6);
    EXPECT_EQ(0, strcmp("01:59.00", buff));
}

TEST(OsdTest, TestConvertTemperatureUnits)
{
    /* In Celsius */
    osdConfigMutable()->units = OSD_UNIT_METRIC;
    EXPECT_EQ(osdConvertTemperatureToSelectedUnit(330), 330);

    /* In Fahrenheit */
    osdConfigMutable()->units = OSD_UNIT_IMPERIAL;
    EXPECT_EQ(osdConvertTemperatureToSelectedUnit(330), 914);
}

// STUBS
extern "C" {
    void beeperConfirmationBeeps(uint8_t) {}

    bool isModeActivationConditionPresent(boxId_e) {
        return false;
    }

    bool IS_RC_MODE_ACTIVE(boxId_e) {
        return false;
    }

    uint32_t micros() {
        return simulationTime;
    }

    uint32_t millis() {
        return micros() / 1000;
    }

    bool isBeeperOn() {
        return false;
    }

    bool isAirmodeActive() {
        return false;
    }

    uint8_t getCurrentPidProfileIndex() {
        return 0;
    }

    uint8_t getCurrentControlRateProfileIndex() {
        return 0;
    }

    batteryState_e getBatteryState() {
        return simulationBatteryState;
    }

    uint8_t getBatteryCellCount() {
        return simulationBatteryCellCount;
    }

    uint16_t getBatteryVoltage() {
        return simulationBatteryVoltage;
    }

    uint16_t getBatteryAverageCellVoltage() {
        return simulationBatteryVoltage / simulationBatteryCellCount;
    }

    int32_t getAmperage() {
        return simulationBatteryAmperage;
    }

    int32_t getMAhDrawn() {
        return simulationMahDrawn;
    }

    int32_t getEstimatedAltitude() {
        return simulationAltitude;
    }

    int32_t getEstimatedVario() {
        return simulationVerticalSpeed;
    }

    unsigned int blackboxGetLogNumber() {
        return 0;
    }

    bool isSerialTransmitBufferEmpty(const serialPort_t *) {
        return false;
    }

    void serialWrite(serialPort_t *, uint8_t) {}

    bool cmsDisplayPortRegister(displayPort_t *) {
        return false;
    }

    uint16_t getRssi(void) { return rssi; }

    uint16_t getCoreTemperatureCelsius(void) { return simulationCoreTemperature; }

    bool isFlipOverAfterCrashMode(void) {
        return false;
    }
}
