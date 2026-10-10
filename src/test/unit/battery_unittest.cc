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

#include <stdint.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "common/filter.h"
    #include "common/maths.h"
    #include "common/utils.h"

    #include "config/config.h"
    #include "config/feature.h"

    #include "drivers/adc.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/mixer.h"

    #include "io/beeper.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "sensors/adcinternal.h"
    #include "sensors/battery.h"
    #include "sensors/voltage.h"

    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 4);

    // Exposed by STATIC_UNIT_TESTED in voltage.c for unit-test builds.
    uint16_t voltageAdcToVoltage(uint16_t sample, const voltageSensorADCConfig_t *config);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

uint16_t simulatedAdcReading;
timeUs_t simulatedTimeUs;

void runBatteryFor(uint16_t adcReading, uint32_t durationMs)
{
    const timeUs_t endTimeUs = simulatedTimeUs + durationMs * 1000;
    while (simulatedTimeUs < endTimeUs) {
        simulatedTimeUs += 20000; // Sample at the voltage task's 50 Hz rate.
        simulatedAdcReading = adcReading;
        batteryUpdateVoltage(simulatedTimeUs);
        batteryUpdatePresence();
        batteryUpdateStates(simulatedTimeUs);
    }
}

} // namespace

TEST(BatteryTest, AdcConversionAndVoltageState)
{
    voltageSensorADCConfig_t adcConfig = {
        .vbatscale = 110,
        .vbatresdivval = 10,
        .vbatresdivmultiplier = 1,
    };

    EXPECT_EQ(0, voltageAdcToVoltage(0, &adcConfig));
    EXPECT_EQ(1259, voltageAdcToVoltage(1420, &adcConfig));
    EXPECT_EQ(3630, voltageAdcToVoltage(4095, &adcConfig));

    pgResetAll();
    simulatedTimeUs = 0;
    simulatedAdcReading = 0;

    // A short filter period keeps this state-machine test quick while still
    // exercising the production ADC conversion and voltage-filter path.
    batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
    batteryConfigMutable()->vbatDisplayLpfPeriod = 1;
    batteryInit();

    runBatteryFor(1420, 2000);
    ASSERT_EQ(3, getBatteryCellCount());
    EXPECT_EQ(BATTERY_OK, getBatteryState());

    runBatteryFor(1175, 1000);
    EXPECT_EQ(BATTERY_WARNING, getBatteryState());

    runBatteryFor(1108, 1000);
    EXPECT_EQ(BATTERY_CRITICAL, getBatteryState());

    runBatteryFor(1140, 1000);
    EXPECT_EQ(BATTERY_WARNING, getBatteryState());

    runBatteryFor(1200, 1000);
    EXPECT_EQ(BATTERY_OK, getBatteryState());
}

extern "C" {

uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode = 0;

bool featureIsEnabled(uint32_t mask)
{
    UNUSED(mask);
    return false;
}

throttleStatus_e calculateThrottleStatus(void)
{
    return THROTTLE_HIGH;
}

bool isRxReceivingSignal(void)
{
    return true;
}

uint32_t millis(void)
{
    return simulatedTimeUs / 1000;
}

uint32_t micros(void)
{
    return simulatedTimeUs;
}

uint16_t getVrefMv(void)
{
    return 3300;
}

uint16_t adcGetValue(adcSource_e source)
{
    UNUSED(source);
    return simulatedAdcReading;
}

void beeperConfirmationBeeps(uint8_t beepCount)
{
    UNUSED(beepCount);
}

void beeper(beeperMode_e mode)
{
    UNUSED(mode);
}

void saveConfigAndNotify(void)
{
}

bool isMotorProtocolEnabled(void)
{
    return true;
}

bool schedulerGetIgnoreTaskExecRate(void)
{
    return false;
}

bool schedulerGetIgnoreTaskExecTime(void)
{
    return false;
}

void schedulerIgnoreTaskExecRate(void)
{
}

void schedulerIgnoreTaskExecTime(void)
{
}

void schedulerSetNextStateTime(timeDelta_t nextStateTime)
{
    UNUSED(nextStateTime);
}

void changePidProfile(uint8_t pidProfileIndex)
{
    UNUSED(pidProfileIndex);
}

void changePidProfileFromCellCount(uint8_t cellCount)
{
    UNUSED(cellCount);
}

void currentMeterADCInit(void)
{
}

void currentMeterADCRefresh(int32_t lastUpdateAt)
{
    UNUSED(lastUpdateAt);
}

void currentMeterADCRead(currentMeter_t *meter)
{
    UNUSED(meter);
}

void currentMeterReset(currentMeter_t *meter)
{
    memset(meter, 0, sizeof(*meter));
}

}
