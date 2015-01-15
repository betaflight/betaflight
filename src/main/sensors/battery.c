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

#include "sensors/battery.h"

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;
uint16_t batteryCriticalVoltage;

uint8_t vbat = 0;                   // battery voltage in 0.1V steps
uint16_t vbatLatest = 0;            // most recent unsmoothed raw reading from vbat adc

int32_t amperage = 0;               // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhDrawn = 0;               // milliampere hours drawn from the battery since start

batteryConfig_t *batteryConfig;

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 0xFFF) * batteryConfig->vbatscale;
}

#define BATTERY_SAMPLE_COUNT 8

void updateBatteryVoltage(void)
{
    static uint16_t vbatSamples[BATTERY_SAMPLE_COUNT];
    static uint8_t currentSampleIndex = 0;
    uint8_t index;
    uint16_t vbatSampleTotal = 0;

    // store the battery voltage with some other recent battery voltage readings
    vbatSamples[(currentSampleIndex++) % BATTERY_SAMPLE_COUNT] = vbatLatest = adcGetChannel(ADC_BATTERY);

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

    // autodetect cell count, going from 1S..8S
    for (i = 1; i < 8; i++) {
        if (vbat < i * batteryConfig->vbatmaxcellvoltage)
            break;
    }

    batteryCellCount = i;
    batteryWarningVoltage = batteryCellCount * batteryConfig->vbatwarningcellvoltage;
    batteryCriticalVoltage = batteryCellCount * batteryConfig->vbatmincellvoltage;
}

#define ADCVREF 33L
int32_t currentSensorToCentiamps(uint16_t src)
{
    int32_t millivolts;

    millivolts = ((uint32_t)src * ADCVREF * 100) / 4095;
    millivolts -= batteryConfig->currentMeterOffset;

    return (millivolts * 1000) / (int32_t)batteryConfig->currentMeterScale; // current in 0.01A steps
}

void updateCurrentMeter(int32_t lastUpdateAt)
{
    static int32_t amperageRaw = 0;
    static int64_t mAhdrawnRaw = 0;

	amperageRaw -= amperageRaw / 8;
	amperageRaw += adcGetChannel(ADC_CURRENT);
	amperage = currentSensorToCentiamps(amperageRaw / 8);

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
