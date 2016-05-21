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

#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#include <platform.h>
#include "build/build_config.h"

#include "common/maths.h"
#include "common/filter.h"

#include "drivers/adc.h"
#include "drivers/system.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "fc/runtime_config.h"
#include "fc/config.h"
#include "fc/rc_controls.h"

#include "io/beeper.h"

#include "sensors/battery.h"


// FIXME there is too much going on in here - the code is not re-usable and has lots of shared configuration, suggest splitting into these topics.
// 1) voltage monitoring/adc conversion
// 2) battery cell count detection and status (present or not)
// 3) battery warnings (aka alarms)
// 4) adc current sensor
// 5) virtual current sensor (has a dependency on an RC control input)

#define VBATT_PRESENT_THRESHOLD_MV    10
#define VBATT_LPF_FREQ  1.0f

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;
uint16_t batteryCriticalVoltage;

uint16_t vbat = 0;                   // battery voltage in 0.1V steps (filtered)
uint16_t vbatLatestADC = 0;         // most recent unsmoothed raw reading from vbat ADC
uint16_t amperageLatestADC = 0;     // most recent raw reading from current ADC

int32_t amperage = 0;               // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhDrawn = 0;               // milliampere hours drawn from the battery since start

static batteryState_e batteryState;
static biquad_t vbatFilterState;

PG_REGISTER_WITH_RESET_TEMPLATE(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);

PG_RESET_TEMPLATE(batteryConfig_t, batteryConfig,
    .vbatscale = VBAT_SCALE_DEFAULT,
    .vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
    .vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
    .vbatmaxcellvoltage = 43,
    .vbatmincellvoltage = 33,
    .vbatwarningcellvoltage = 35,
    .currentMeterScale = 400, // for Allegro ACS758LCB-100U (40mV/A)
    .currentMeterType = CURRENT_SENSOR_ADC,
);

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return ((((uint32_t)src * batteryConfig()->vbatscale * 33 + (0xFFF * 5)) / (0xFFF * batteryConfig()->vbatresdivval)) / batteryConfig()->vbatresdivmultiplier);
}

static void updateBatteryVoltage(void)
{
#ifdef ADC_BATTERY
    uint16_t vbatSample;
    // store the battery voltage with some other recent battery voltage readings
    vbatSample = vbatLatestADC = adcGetChannel(ADC_BATTERY);
    vbatSample = applyBiQuadFilter(vbatSample, &vbatFilterState);
    vbat = batteryAdcToVoltage(vbatSample);
#endif
}

#define VBATTERY_STABLE_DELAY 40
/* Batt Hysteresis of +/-100mV */
#define VBATT_HYSTERESIS 1

void updateBattery(void)
{
    updateBatteryVoltage();
    
    /* battery has just been connected*/
    if (batteryState == BATTERY_NOT_PRESENT && vbat > VBATT_PRESENT_THRESHOLD_MV)
    {
        /* Actual battery state is calculated below, this is really BATTERY_PRESENT */
        batteryState = BATTERY_OK;
        /* wait for VBatt to stabilise then we can calc number of cells
        (using the filtered value takes a long time to ramp up) 
        We only do this on the ground so don't care if we do block, not
        worse than original code anyway*/
        delay(VBATTERY_STABLE_DELAY);
        updateBatteryVoltage();

        unsigned cells = (batteryAdcToVoltage(vbatLatestADC) / batteryConfig()->vbatmaxcellvoltage) + 1;
        if (cells > 8) {
            // something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)
            cells = 8;
        }
        batteryCellCount = cells;
        batteryWarningVoltage = batteryCellCount * batteryConfig()->vbatwarningcellvoltage;
        batteryCriticalVoltage = batteryCellCount * batteryConfig()->vbatmincellvoltage;
    }
    /* battery has been disconnected - can take a while for filter cap to disharge so we use a threshold of VBATT_PRESENT_THRESHOLD_MV */
    else if (batteryState != BATTERY_NOT_PRESENT && vbat <= VBATT_PRESENT_THRESHOLD_MV)
    {
        batteryState = BATTERY_NOT_PRESENT;
        batteryCellCount = 0;
        batteryWarningVoltage = 0;
        batteryCriticalVoltage = 0;
    }    

    switch(batteryState)
    {
        case BATTERY_OK:
            if (vbat <= (batteryWarningVoltage - VBATT_HYSTERESIS)) {
                batteryState = BATTERY_WARNING;
                beeper(BEEPER_BAT_LOW);
            }
            break;
        case BATTERY_WARNING:
            if (vbat <= (batteryCriticalVoltage - VBATT_HYSTERESIS)) {
                batteryState = BATTERY_CRITICAL;
                beeper(BEEPER_BAT_CRIT_LOW);
            } else if (vbat > (batteryWarningVoltage + VBATT_HYSTERESIS)){
                batteryState = BATTERY_OK;
            } else {
                beeper(BEEPER_BAT_LOW);
            }
            break;
        case BATTERY_CRITICAL:
            if (vbat > (batteryCriticalVoltage + VBATT_HYSTERESIS)){
                batteryState = BATTERY_WARNING;
                beeper(BEEPER_BAT_LOW);
            } else {
                beeper(BEEPER_BAT_CRIT_LOW);
            }
            break;
        case BATTERY_NOT_PRESENT:
            break;
    }
}

batteryState_e getBatteryState(void)
{
    return batteryState;
}

const char * const batteryStateStrings[] = {"OK", "WARNING", "CRITICAL", "NOT PRESENT"};

const char * getBatteryStateString(void)
{
    return batteryStateStrings[batteryState];
}

void batteryInit(void)
{
    batteryState = BATTERY_NOT_PRESENT;
    batteryCellCount = 1;
    batteryWarningVoltage = 0;
    batteryCriticalVoltage = 0;

    BiQuadNewLpf(VBATT_LPF_FREQ, &vbatFilterState, 50000);

}

#define ADCVREF 3300   // in mV
int32_t currentSensorToCentiamps(uint16_t src)
{
    int32_t millivolts;

    millivolts = ((uint32_t)src * ADCVREF) / 4096;
    millivolts -= batteryConfig()->currentMeterOffset;

    return (millivolts * 1000) / (int32_t)batteryConfig()->currentMeterScale; // current in 0.01A steps
}

void updateCurrentMeter(int32_t lastUpdateAt)
{
#ifndef ADC_CURRENT
    UNUSED(lastUpdateAt);
#else
    static int32_t amperageRaw = 0;
    static int64_t mAhdrawnRaw = 0;

    amperageRaw -= amperageRaw / 8;
    amperageRaw += (amperageLatestADC = adcGetChannel(ADC_CURRENT));
    amperage = currentSensorToCentiamps(amperageRaw / 8);

    mAhdrawnRaw += (MAX(0, amperage) * lastUpdateAt) / 1000;
    mAhDrawn = mAhdrawnRaw / (3600 * 100);
#endif
}


void updateVirtualCurrentMeter(int32_t lastUpdateAt, throttleStatus_e throttleStatus)
{
    static int64_t mAhdrawnRaw = 0;
    int32_t throttleOffset = (int32_t)rcCommand[THROTTLE] - 1000;
    int32_t throttleFactor = 0;


    amperage = (int32_t)batteryConfig()->currentMeterOffset;
    if (ARMING_FLAG(ARMED)) {
        if (throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP))
            throttleOffset = 0;
        throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50);
        amperage += throttleFactor * (int32_t)batteryConfig()->currentMeterScale  / 1000;
    }

    mAhdrawnRaw += (MAX(0, amperage) * lastUpdateAt) / 1000;
    mAhDrawn = mAhdrawnRaw / (3600 * 100);
}

uint8_t calculateBatteryPercentage(void)
{
    return constrain((((uint32_t)vbat - (batteryConfig()->vbatmincellvoltage * batteryCellCount)) * 100) / ((batteryConfig()->vbatmaxcellvoltage - batteryConfig()->vbatmincellvoltage) * batteryCellCount), 0, 100);
}

uint8_t calculateBatteryCapacityRemainingPercentage(void)
{
    uint16_t batteryCapacity = batteryConfig()->batteryCapacity;

    return constrain((batteryCapacity - constrain(mAhDrawn, 0, 0xFFFF)) * 100.0f / batteryCapacity , 0, 100);
}
