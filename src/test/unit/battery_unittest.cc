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
#include <limits.h>

extern "C" {
    #include "sensors/battery.h"
    #include "io/beeper.h"
    #include "build/debug.h"

    int8_t voltage = 0;
    int8_t mah = 0;

    void initBatteryConfig(void) {
        batteryConfigMutable()->currentMeterSource = CURRENT_METER_ADC;
        batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;

        batteryConfigMutable()->vbatmaxcellvoltage = 43;
        batteryConfigMutable()->vbatmincellvoltage = 33;
        batteryConfigMutable()->vbatnotpresentcellvoltage = 0;

        voltage = 0;
        mah = 0;

        batteryInit();
        batteryUpdatePresence();
    }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(BatteryTest, UninitializedBatteryCompensatedThrottleLimitTest)
{
    float throttleLimitFactor = 0.25f;
    float factor = calculateThrottleLimitBatteryCompensatedFactor(throttleLimitFactor);
    EXPECT_EQ(throttleLimitFactor, factor);
}

TEST(BatteryTest, MahCompensatedThrottleLimitTest)
{
    initBatteryConfig();

    float throttleLimitFactor = 0.8f;
    int8_t capacityData[7] = {-20, 0, 25, 50, 75, 100, 120};
    float expectedFactors[7] = {0.8f, 0.8f, 0.85f, 0.9f, 0.95f, 1.0f, 1.0f};
    batteryConfigMutable()->batteryCapacity = 100;

    for (int i = 0; i < 7; i++) {
        mah = capacityData[i];
        batteryUpdateCurrentMeter(0);
        float factor = calculateThrottleLimitBatteryCompensatedFactor(throttleLimitFactor);
        EXPECT_EQ(expectedFactors[i], factor);
    }
}

TEST(BatteryTest, VoltageCompensatedThrottleLimitTest)
{
    initBatteryConfig();

    float throttleLimitFactor = 0.5f;
    int8_t voltageData[7] = {45, 43, 42, 40, 36, 33, 25};
    float expectedFactors[7] = {0.5f, 0.5f, 0.55f, 0.65f, 0.85f, 1.0f, 1.0f};
    batteryConfigMutable()->batteryCapacity = 0;

    for (int i = 0; i < 7; i++) {
        voltage = voltageData[i];
        batteryUpdateVoltage(0);
        float factor = calculateThrottleLimitBatteryCompensatedFactor(throttleLimitFactor);
        EXPECT_EQ(expectedFactors[i], factor);
    }
}

// STUBS

extern "C" {

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    void beeper(beeperMode_e ) {}

    void voltageMeterADCInit(void) {}
    void voltageMeterADCRefresh(void) {}
    void voltageMeterReset(voltageMeter_t * ) {}

    void voltageMeterADCRead(voltageSensorADC_e , voltageMeter_t *meter)
    {
        meter->filtered = voltage;
        meter->unfiltered = voltage;
    }

    void currentMeterADCInit(void) {}
    void currentMeterADCRefresh(int32_t ) {}
    void currentMeterReset(currentMeter_t * ) {}

    void currentMeterADCRead(currentMeter_t *meter)
    {
        meter->mAhDrawn = mah;
    }
}