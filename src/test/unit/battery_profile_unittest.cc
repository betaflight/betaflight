/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <limits.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "common/filter.h"
    #include "common/maths.h"
    #include "common/utils.h"

    #include "config/config.h"
    #include "config/feature.h"

    #include "drivers/adc.h"

    #include "fc/runtime_config.h"
    #include "fc/rc_controls.h"

    #include "flight/mixer.h"

    #include "io/beeper.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "sensors/battery.h"

    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 4);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Verifies that all 3 battery profiles are initialized with the expected
// default voltage thresholds and capacity values.
TEST(BatteryProfileTest, ProfileDefaultsAreCorrect)
{
    pgResetAll();

    for (int i = 0; i < BATTERY_PROFILE_COUNT; i++) {
        const batteryProfile_t *profile = batteryProfiles(i);

        EXPECT_EQ(VBAT_CELL_VOLTAGE_DEFAULT_MAX, profile->vbatmaxcellvoltage);
        EXPECT_EQ(VBAT_CELL_VOLTAGE_DEFAULT_MIN, profile->vbatmincellvoltage);
        EXPECT_EQ(350, profile->vbatwarningcellvoltage);
        EXPECT_EQ(410, profile->vbatfullcellvoltage);
        EXPECT_EQ(0, profile->batteryCapacity);
        EXPECT_EQ(0, profile->forceBatteryCellCount);
        EXPECT_EQ(10, profile->consumptionWarningPercentage);
        EXPECT_EQ('\0', profile->profileName[0]);
    }
}

// Verifies that switching battery profiles updates the currentBatteryProfile pointer
// and that changes to one profile do not affect another.
TEST(BatteryProfileTest, ProfileSwitching)
{
    pgResetAll();

    // Modify profile 0 to have LiPo defaults
    batteryProfilesMutable(0)->vbatmincellvoltage = 330;

    // Modify profile 1 to have Li-Ion values
    batteryProfilesMutable(1)->vbatmincellvoltage = 280;
    batteryProfilesMutable(1)->vbatmaxcellvoltage = 420;
    batteryProfilesMutable(1)->batteryCapacity = 3000;

    // Switch to profile 0
    loadBatteryProfile();
    EXPECT_EQ(330, currentBatteryProfile->vbatmincellvoltage);

    // Switch to profile 1
    systemConfigMutable()->activeBatteryProfile = 1;
    loadBatteryProfile();
    EXPECT_EQ(280, currentBatteryProfile->vbatmincellvoltage);
    EXPECT_EQ(420, currentBatteryProfile->vbatmaxcellvoltage);
    EXPECT_EQ(3000, currentBatteryProfile->batteryCapacity);

    // Switch to profile 2 (should be defaults)
    systemConfigMutable()->activeBatteryProfile = 2;
    loadBatteryProfile();
    EXPECT_EQ(VBAT_CELL_VOLTAGE_DEFAULT_MIN, currentBatteryProfile->vbatmincellvoltage);
}

// Verifies that changeBatteryProfile clamps out-of-range indices to a valid value.
TEST(BatteryProfileTest, ProfileBoundaryClamping)
{
    pgResetAll();

    // Set a known active profile first
    systemConfigMutable()->activeBatteryProfile = 0;
    loadBatteryProfile();
    EXPECT_EQ(0, systemConfig()->activeBatteryProfile);

    // Attempting to switch to an invalid profile index should not change the stored index
    changeBatteryProfile(255);
    // Index stays at 0 because 255 >= BATTERY_PROFILE_COUNT, the if-guard skips the write
    EXPECT_EQ(0, systemConfig()->activeBatteryProfile);
}

// Verifies that battery profile names can be set and retrieved correctly.
TEST(BatteryProfileTest, ProfileNames)
{
    pgResetAll();

    // Set a profile name
    strncpy(batteryProfilesMutable(0)->profileName, "LiPo", MAX_BATTERY_PROFILE_NAME_LENGTH);
    batteryProfilesMutable(0)->profileName[MAX_BATTERY_PROFILE_NAME_LENGTH] = '\0';

    strncpy(batteryProfilesMutable(1)->profileName, "Li-Ion", MAX_BATTERY_PROFILE_NAME_LENGTH);
    batteryProfilesMutable(1)->profileName[MAX_BATTERY_PROFILE_NAME_LENGTH] = '\0';

    EXPECT_STREQ("LiPo", batteryProfiles(0)->profileName);
    EXPECT_STREQ("Li-Ion", batteryProfiles(1)->profileName);
    EXPECT_STREQ("", batteryProfiles(2)->profileName);
}

// Verifies that each profile maintains independent voltage and capacity values
// and that switching profiles reads the correct per-profile data.
TEST(BatteryProfileTest, IndependentProfileValues)
{
    pgResetAll();

    // Set distinct values for each profile
    for (int i = 0; i < BATTERY_PROFILE_COUNT; i++) {
        batteryProfilesMutable(i)->vbatmincellvoltage = 300 + i * 10;
        batteryProfilesMutable(i)->vbatmaxcellvoltage = 420 + i * 5;
        batteryProfilesMutable(i)->batteryCapacity = 1000 * (i + 1);
    }

    // Verify each profile has distinct values
    for (int i = 0; i < BATTERY_PROFILE_COUNT; i++) {
        systemConfigMutable()->activeBatteryProfile = i;
        loadBatteryProfile();

        EXPECT_EQ((uint16_t)(300 + i * 10), currentBatteryProfile->vbatmincellvoltage);
        EXPECT_EQ((uint16_t)(420 + i * 5), currentBatteryProfile->vbatmaxcellvoltage);
        EXPECT_EQ((uint16_t)(1000 * (i + 1)), currentBatteryProfile->batteryCapacity);
    }
}

// Stubs for dependencies that battery.c requires but that we don't test here
extern "C" {
    // Runtime config stubs
    uint8_t armingFlags = 0;
    uint8_t stateFlags = 0;
    uint16_t flightModeFlags = 0;

    bool featureIsEnabled(uint32_t mask) {
        UNUSED(mask);
        return false;
    }

    throttleStatus_e calculateThrottleStatus(void) {
        return THROTTLE_HIGH;
    }

    bool isRxReceivingSignal(void) {
        return true;
    }

    uint32_t millis(void) {
        return 0;
    }

    uint32_t micros(void) {
        return 0;
    }

    void beeperConfirmationBeeps(uint8_t beepCount) {
        UNUSED(beepCount);
    }

    void beeper(beeperMode_e mode) {
        UNUSED(mode);
    }

    void saveConfigAndNotify(void) {
    }

    // Mixer stubs
    bool isMotorProtocolEnabled(void) {
        return true;
    }

    // Debug stubs
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode = 0;

    // Scheduler stubs
    bool schedulerGetIgnoreTaskExecRate(void) {
        return false;
    }

    bool schedulerGetIgnoreTaskExecTime(void) {
        return false;
    }

    void schedulerIgnoreTaskExecRate(void) {
    }

    void schedulerIgnoreTaskExecTime(void) {
    }

    void schedulerSetNextStateTime(timeDelta_t) {
    }

    // PG stubs for config.c dependencies
    void changePidProfile(uint8_t pidProfileIndex) {
        UNUSED(pidProfileIndex);
    }

    void changePidProfileFromCellCount(uint8_t) {
    }

    // Voltage/current meter stubs
    void voltageMeterADCInit(void) {}
    void voltageMeterADCRefresh(void) {}
    void voltageMeterADCRead(voltageSensorADC_e src, voltageMeter_t *meter) {
        UNUSED(src);
        UNUSED(meter);
    }
    void voltageMeterGenericInit(void) {}
    void voltageMeterReset(voltageMeter_t *meter) {
        UNUSED(meter);
    }
    bool voltageIsStable(voltageMeter_t *vm) { UNUSED(vm); return true; }
    void voltageStableUpdate(voltageMeter_t *vm) { UNUSED(vm); }

    void currentMeterADCInit(void) {}
    void currentMeterADCRefresh(int32_t lastUpdateAt) { UNUSED(lastUpdateAt); }
    void currentMeterADCRead(currentMeter_t *meter) {
        UNUSED(meter);
    }
    void currentMeterReset(currentMeter_t *meter) {
        UNUSED(meter);
    }

}
