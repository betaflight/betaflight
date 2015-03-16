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

#include "common/maths.h"

#include "drivers/adc.h"
#include "drivers/system.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/battery.h"

#include "rx/rx.h"

#include "io/rc_controls.h"

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;
uint16_t batteryCriticalVoltage;

uint8_t vbat = 0;                   // battery voltage in 0.1V steps
uint16_t vbatLatestADC = 0;         // most recent unsmoothed raw reading from vbat ADC
uint16_t amperageLatestADC = 0;     // most recent raw reading from current ADC

int32_t amperage = 0;               // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhDrawn = 0;               // milliampere hours drawn from the battery since start

batteryConfig_t *batteryConfig;

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return ((uint32_t)src * batteryConfig->vbatscale * 33 + (0xFFF * 5)) / (0xFFF * 10);
}

#define BATTERY_SAMPLE_COUNT 8

void updateBatteryVoltage(void)
{
    static uint16_t vbatSamples[BATTERY_SAMPLE_COUNT];
    static uint8_t currentSampleIndex = 0;
    uint8_t index;
    uint16_t vbatSampleTotal = 0;

    // store the battery voltage with some other recent battery voltage readings
    vbatSamples[(currentSampleIndex++) % BATTERY_SAMPLE_COUNT] = vbatLatestADC = adcGetChannel(ADC_BATTERY);

    // calculate vbat based on the average of recent readings
    for (index = 0; index < BATTERY_SAMPLE_COUNT; index++) {
        vbatSampleTotal += vbatSamples[index];
    }
    vbat = batteryAdcToVoltage(vbatSampleTotal / BATTERY_SAMPLE_COUNT);
}

batteryState_e calculateBatteryState(void)
{
    if (vbat <= batteryCriticalVoltage) {
        return BATTERY_CRITICAL;
    }
    if (vbat <= batteryWarningVoltage) {
        return BATTERY_WARNING;
    }
    return BATTERY_OK;
}

void batteryInit(batteryConfig_t *initialBatteryConfig)
{
    batteryConfig = initialBatteryConfig;

    uint32_t i;

    for (i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
        updateBatteryVoltage();
        delay((32 / BATTERY_SAMPLE_COUNT) * 10);
    }

    unsigned cells = (vbat / batteryConfig->vbatmaxcellvoltage) + 1;
    if(cells > 8)            // something is wrong, we expect 8 cells maximum (and autodetection will be problematic at 6+ cells)
        cells = 8;
    batteryCellCount = cells;
    batteryWarningVoltage = batteryCellCount * batteryConfig->vbatwarningcellvoltage;
    batteryCriticalVoltage = batteryCellCount * batteryConfig->vbatmincellvoltage;
}

#define ADCVREF 3300   // in mV
int32_t currentSensorToCentiamps(uint16_t src)
{
    int32_t millivolts;

    millivolts = ((uint32_t)src * ADCVREF) / 4096;
    millivolts -= batteryConfig->currentMeterOffset;

    return (millivolts * 1000) / (int32_t)batteryConfig->currentMeterScale; // current in 0.01A steps
}

void updateCurrentMeter(int32_t lastUpdateAt, rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    static int32_t amperageRaw = 0;
    static int64_t mAhdrawnRaw = 0;
    int32_t throttleOffset = (int32_t)rcCommand[THROTTLE] - 1000;
    int32_t throttleFactor = 0;

    switch(batteryConfig->currentMeterType) {
        case CURRENT_SENSOR_ADC:
            amperageRaw -= amperageRaw / 8;
            amperageRaw += (amperageLatestADC = adcGetChannel(ADC_CURRENT));
            amperage = currentSensorToCentiamps(amperageRaw / 8);
            break;
        case CURRENT_SENSOR_VIRTUAL:
            amperage = (int32_t)batteryConfig->currentMeterOffset;
            if(ARMING_FLAG(ARMED)) {
                throttleStatus_e throttleStatus = calculateThrottleStatus(rxConfig, deadband3d_throttle);
                if (throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP))
                    throttleOffset = 0;
                throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50);
                amperage += throttleFactor * (int32_t)batteryConfig->currentMeterScale  / 1000;
            }
            break;
        case CURRENT_SENSOR_NONE:
            amperage = 0;
            break;
    }

    mAhdrawnRaw += (amperage * lastUpdateAt) / 1000;
    mAhDrawn = mAhdrawnRaw / (3600 * 100);
}

uint8_t calculateBatteryPercentage(void)
{
    return (((uint32_t)vbat - (batteryConfig->vbatmincellvoltage * batteryCellCount)) * 100) / ((batteryConfig->vbatmaxcellvoltage - batteryConfig->vbatmincellvoltage) * batteryCellCount);
}

uint8_t calculateBatteryCapacityRemainingPercentage(void)
{
    uint16_t batteryCapacity = batteryConfig->batteryCapacity;

    return constrain((batteryCapacity - constrain(mAhDrawn, 0, 0xFFFF)) * 100.0 / batteryCapacity , 0, 100);
}
