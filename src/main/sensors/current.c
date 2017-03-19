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
#include "common/utils.h"
#include "common/filter.h"

#include "drivers/adc.h"
#include "drivers/system.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_reset.h"

#include "sensors/current.h"
#include "sensors/esc_sensor.h"

const uint8_t currentMeterIds[] = {
    CURRENT_METER_ID_BATTERY_1,
    CURRENT_METER_ID_VIRTUAL_1,
#ifdef USE_ESC_SENSOR
    CURRENT_METER_ID_ESC_COMBINED_1,
    CURRENT_METER_ID_ESC_MOTOR_1,
    CURRENT_METER_ID_ESC_MOTOR_2,
    CURRENT_METER_ID_ESC_MOTOR_3,
    CURRENT_METER_ID_ESC_MOTOR_4,
    CURRENT_METER_ID_ESC_MOTOR_5,
    CURRENT_METER_ID_ESC_MOTOR_6,
    CURRENT_METER_ID_ESC_MOTOR_7,
    CURRENT_METER_ID_ESC_MOTOR_8,
    CURRENT_METER_ID_ESC_MOTOR_9,
    CURRENT_METER_ID_ESC_MOTOR_10,
    CURRENT_METER_ID_ESC_MOTOR_11,
    CURRENT_METER_ID_ESC_MOTOR_12,
#endif
};

const uint8_t supportedCurrentMeterCount = ARRAYLEN(currentMeterIds);

//
// ADC/Virtual/ESC shared
//

void currentMeterReset(currentMeter_t *meter)
{
    meter->amperage = 0;
    meter->amperageLatest = 0;
    meter->mAhDrawn = 0;
}

//
// ADC/Virtual shared
//

#define ADCVREF 3300   // in mV

#define IBAT_LPF_FREQ  0.4f
static biquadFilter_t adciBatFilter;

#ifndef CURRENT_METER_SCALE_DEFAULT
#define CURRENT_METER_SCALE_DEFAULT 400 // for Allegro ACS758LCB-100U (40mV/A)
#endif

#ifndef CURRENT_METER_OFFSET_DEFAULT
#define CURRENT_METER_OFFSET_DEFAULT 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig, PG_CURRENT_SENSOR_ADC_CONFIG, 0);

PG_RESET_TEMPLATE(currentSensorADCConfig_t, currentSensorADCConfig,
    .scale = CURRENT_METER_SCALE_DEFAULT,
    .offset = CURRENT_METER_OFFSET_DEFAULT,
);

PG_REGISTER(currentSensorVirtualConfig_t, currentSensorVirtualConfig, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, 0);

static int32_t currentMeterADCToCentiamps(const uint16_t src)
{

    const currentSensorADCConfig_t *config = currentSensorADCConfig();

    int32_t millivolts = ((uint32_t)src * ADCVREF) / 4096;
    millivolts -= config->offset;

    return (millivolts * 1000) / (int32_t)config->scale; // current in 0.01A steps
}

static void updateCurrentmAhDrawnState(currentMeterMAhDrawnState_t *state, int32_t amperageLatest, int32_t lastUpdateAt)
{
    state->mAhDrawnF = state->mAhDrawnF + (amperageLatest * lastUpdateAt / (100.0f * 1000 * 3600));
    state->mAhDrawn = state->mAhDrawnF;
}

//
// ADC
//

currentMeterADCState_t currentMeterADCState;

void currentMeterADCInit(void)
{
    memset(&currentMeterADCState, 0, sizeof(currentMeterADCState_t));
    biquadFilterInitLPF(&adciBatFilter, IBAT_LPF_FREQ, 50000); //50HZ Update
}

void currentMeterADCRefresh(int32_t lastUpdateAt)
{
    const uint16_t iBatSample = adcGetChannel(ADC_CURRENT);
    currentMeterADCState.amperageLatest = currentMeterADCToCentiamps(iBatSample);
    currentMeterADCState.amperage = currentMeterADCToCentiamps(biquadFilterApply(&adciBatFilter, iBatSample));

    updateCurrentmAhDrawnState(&currentMeterADCState.mahDrawnState, currentMeterADCState.amperageLatest, lastUpdateAt);
}

void currentMeterADCRead(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterADCState.amperageLatest;
    meter->amperage = currentMeterADCState.amperage;
    meter->mAhDrawn = currentMeterADCState.mahDrawnState.mAhDrawn;
}

//
// VIRTUAL
//

currentSensorVirtualState_t currentMeterVirtualState;

void currentMeterVirtualInit(void)
{
    memset(&currentMeterVirtualState, 0, sizeof(currentSensorVirtualState_t));
}

void currentMeterVirtualRefresh(int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset)
{
    currentMeterVirtualState.amperage = (int32_t)currentSensorVirtualConfig()->offset;
    if (armed) {
        if (throttleLowAndMotorStop) {
            throttleOffset = 0;
        }

        int throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50); // FIXME magic number 50,  50hz?
        currentMeterVirtualState.amperage += throttleFactor * (int32_t)currentSensorVirtualConfig()->scale / 1000;
    }
    updateCurrentmAhDrawnState(&currentMeterVirtualState.mahDrawnState, currentMeterVirtualState.amperage, lastUpdateAt);
}

void currentMeterVirtualRead(currentMeter_t *meter)
{
    meter->amperageLatest = currentMeterVirtualState.amperage;
    meter->amperage = currentMeterVirtualState.amperage;
    meter->mAhDrawn = currentMeterVirtualState.mahDrawnState.mAhDrawn;
}

//
// ESC
//

#ifdef USE_ESC_SENSOR
currentMeterESCState_t currentMeterESCState;
#endif

void currentMeterESCInit(void)
{
#ifdef USE_ESC_SENSOR
    memset(&currentMeterESCState, 0, sizeof(currentMeterESCState_t));
#endif
}

void currentMeterESCRefresh(int32_t lastUpdateAt)
{
    UNUSED(lastUpdateAt);
#ifdef USE_ESC_SENSOR
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData->dataAge <= ESC_BATTERY_AGE_MAX) {
        currentMeterESCState.amperage = escData->current;
        currentMeterESCState.mAhDrawn = escData->consumption;
    } else {
        currentMeterESCState.amperage = 0;
        currentMeterESCState.mAhDrawn = 0;
    }
#endif
}

void currentMeterESCReadCombined(currentMeter_t *meter)
{
#ifdef USE_ESC_SENSOR
    meter->amperageLatest = currentMeterESCState.amperage;
    meter->amperage = currentMeterESCState.amperage;
    meter->mAhDrawn = currentMeterESCState.mAhDrawn;
#else
    currentMeterReset(meter);
#endif
}

void currentMeterESCReadMotor(uint8_t motorNumber, currentMeter_t *meter)
{
#ifndef USE_ESC_SENSOR
    UNUSED(motorNumber);
    currentMeterReset(meter);
#else
    escSensorData_t *escData = getEscSensorData(motorNumber);
    if (escData->dataAge <= ESC_BATTERY_AGE_MAX) {
        meter->amperage = escData->current;
        meter->amperageLatest = escData->current;
        meter->mAhDrawn = escData->consumption;
        return;
    }
#endif
}

//
// API for current meters using IDs
//
// This API is used by MSP, for configuration/status.
//

void currentMeterRead(currentMeterId_e id, currentMeter_t *meter)
{
    if (id == CURRENT_METER_ID_BATTERY_1) {
        currentMeterADCRead(meter);
    } else if (id == CURRENT_METER_ID_VIRTUAL_1) {
        currentMeterVirtualRead(meter);
    }
#ifdef USE_ESC_SENSOR
    if (id == CURRENT_METER_ID_ESC_COMBINED_1) {
        currentMeterESCReadCombined(meter);
    } else
    if (id >= CURRENT_METER_ID_ESC_MOTOR_1 && id <= CURRENT_METER_ID_ESC_MOTOR_20 ) {
        int motor = id - CURRENT_METER_ID_ESC_MOTOR_1;
        currentMeterESCReadMotor(motor, meter);
    } else
#endif
    {
        currentMeterReset(meter);
    }
}
