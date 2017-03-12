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

#define ADCVREF 3300   // in mV

#define IBAT_LPF_FREQ  0.4f
static biquadFilter_t adciBatFilter;

#ifndef CURRENT_METER_SCALE_DEFAULT
#define CURRENT_METER_SCALE_DEFAULT 400 // for Allegro ACS758LCB-100U (40mV/A)
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(currentMeterADCOrVirtualConfig_t, MAX_ADC_OR_VIRTUAL_CURRENT_METERS, currentMeterADCOrVirtualConfig, PG_CURRENT_SENSOR_ADC_OR_VIRTUAL_CONFIG, 0);

void pgResetFn_currentMeterADCOrVirtualConfig(currentMeterADCOrVirtualConfig_t *instance)
{
    for (int i = 0; i < MAX_ADC_OR_VIRTUAL_CURRENT_METERS; i++) {
        if (i == CURRENT_METER_ADC) {
            RESET_CONFIG(currentMeterADCOrVirtualConfig_t, &instance[i],
                .scale = CURRENT_METER_SCALE_DEFAULT,
            );
        }
    }
}

static int32_t currentMeterADCToCentiamps(const uint16_t src)
{
    int32_t millivolts;

    const currentMeterADCOrVirtualConfig_t *config = currentMeterADCOrVirtualConfig(CURRENT_SENSOR_ADC);

    millivolts = ((uint32_t)src * ADCVREF) / 4096;
    millivolts -= config->offset;

    return (millivolts * 1000) / (int32_t)config->scale; // current in 0.01A steps
}

void updateCurrentDrawn(currentMeter_t *state, int32_t lastUpdateAt)
{
    state->mAhDrawnF = state->mAhDrawnF + (state->amperageLatest * lastUpdateAt / (100.0f * 1000 * 3600));
    state->mAhDrawn = state->mAhDrawnF;
}

void currentUpdateADCMeter(currentMeter_t *state, int32_t lastUpdateAt)
{
    uint16_t iBatSample = adcGetChannel(ADC_CURRENT);
    state->amperageLatest = currentMeterADCToCentiamps(iBatSample);
    state->amperage = currentMeterADCToCentiamps(biquadFilterApply(&adciBatFilter, iBatSample));

    updateCurrentDrawn(state, lastUpdateAt);
}

void currentUpdateVirtualMeter(currentMeter_t *state, int32_t lastUpdateAt, bool armed, bool throttleLowAndMotorStop, int32_t throttleOffset)
{
    state->amperage = (int32_t)currentMeterADCOrVirtualConfig(CURRENT_SENSOR_VIRTUAL)->offset;
    if (armed) {
        if (throttleLowAndMotorStop) {
            throttleOffset = 0;
        }

        int throttleFactor = throttleOffset + (throttleOffset * throttleOffset / 50); // FIXME magic number 50,  50hz?
        state->amperageLatest = state->amperage += throttleFactor * (int32_t)currentMeterADCOrVirtualConfig(CURRENT_SENSOR_VIRTUAL)->scale / 1000;
    }
    updateCurrentDrawn(state, lastUpdateAt);
}

void currentUpdateESCMeter(currentMeter_t *state, int32_t lastUpdateAt)
{
    UNUSED(lastUpdateAt);
#ifndef USE_ESC_SENSOR
    UNUSED(state);
#else
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    if (escData->dataAge <= ESC_BATTERY_AGE_MAX) {
        state->amperageLatest = escData->current;
        state->mAhDrawn = escData->consumption;
    } else {
        state->amperageLatest = 0;
        state->mAhDrawn = 0;
    }
    state->amperage = state->amperageLatest;
#endif
}

void resetCurrentMeterState(currentMeter_t *state)
{
    state->amperage = 0;
    state->amperageLatest = 0;
}

void currentMeterADCInit(void)
{
    biquadFilterInitLPF(&adciBatFilter, IBAT_LPF_FREQ, 50000); //50HZ Update
}
