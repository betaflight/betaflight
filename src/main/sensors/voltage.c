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
#include "common/utils.h"

#include "drivers/adc.h"
#include "drivers/system.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/config_reset.h"

#include "sensors/voltage.h"
#include "sensors/esc_sensor.h"

#ifndef VBAT_SCALE_DEFAULT
#define VBAT_SCALE_DEFAULT 110
#endif

#ifdef USE_ESC_SENSOR
static biquadFilter_t escvBatFilter;
#endif

voltageMeterADCState_t voltageMeterADCStates[MAX_VOLTAGE_SENSOR_ADC];

voltageMeterADCState_t *getVoltageMeterADC(uint8_t index)
{
    return &voltageMeterADCStates[index];
}

PG_REGISTER_ARRAY_WITH_RESET_FN(voltageSensorADCConfig_t, MAX_VOLTAGE_SENSOR_ADC, voltageSensorADCConfig, PG_VOLTAGE_SENSOR_ADC_CONFIG, 0);

void pgResetFn_voltageSensorADCConfig(voltageSensorADCConfig_t *instance)
{
    for (int i = 0; i < MAX_VOLTAGE_SENSOR_ADC; i++) {
        RESET_CONFIG(voltageSensorADCConfig_t, &instance[i],
            .vbatscale = VBAT_SCALE_DEFAULT,
            .vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
            .vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
        );
    }
}


static const uint8_t voltageMeterAdcChannelMap[] = {
    ADC_BATTERY,
#ifdef ADC_POWER_12V
    ADC_POWER_12V,
#endif
#ifdef ADC_POWER_5V
    ADC_POWER_5V,
#endif
};

STATIC_UNIT_TESTED uint16_t voltageAdcToVoltage(const uint16_t src, const voltageSensorADCConfig_t *config)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 10:1 voltage divider (10k:1k) * 10 for 0.1V
    return ((((uint32_t)src * config->vbatscale * 33 + (0xFFF * 5)) / (0xFFF * config->vbatresdivval)) / config->vbatresdivmultiplier);
}

void voltageMeterADCRefresh(void)
{


    for (uint8_t i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        // store the battery voltage with some other recent battery voltage readings

        voltageMeterADCState_t *state = &voltageMeterADCStates[i];
        const voltageSensorADCConfig_t *config = voltageSensorADCConfig(i);

        uint8_t channel = voltageMeterAdcChannelMap[i];
        uint16_t rawSample = adcGetChannel(channel);

        uint16_t filteredSample = biquadFilterApply(&state->vbatFilterState, rawSample);

        // always calculate the latest voltage, see getLatestVoltage() which does the calculation on demand.
        state->voltageFiltered = voltageAdcToVoltage(filteredSample, config);
        state->voltageUnfiltered = voltageAdcToVoltage(rawSample, config);
    }
}

void voltageMeterADCUpdate(voltageMeter_t *voltageMeter, voltageSensorADC_e adcChannel)
{
    voltageMeterADCState_t *state = &voltageMeterADCStates[adcChannel];

    voltageMeter->filtered = state->voltageFiltered;
    voltageMeter->unfiltered = state->voltageUnfiltered;
}

void voltageMeterADCInit(void)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_SENSOR_ADC && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        // store the battery voltage with some other recent battery voltage readings

        voltageMeterADCState_t *state = &voltageMeterADCStates[i];
        memset(state, 0, sizeof(voltageMeterADCState_t));

        biquadFilterInitLPF(&state->vbatFilterState, VBATT_LPF_FREQ, 50000);
    }
}

void voltageMeterReset(voltageMeter_t *meter)
{
    meter->filtered = 0;
    meter->unfiltered = 0;
}

#define VBAT_LPF_FREQ  0.4f

void voltageMeterESCInit(void)
{
#ifdef USE_ESC_SENSOR
    biquadFilterInitLPF(&escvBatFilter, VBAT_LPF_FREQ, 50000); //50HZ Update
#endif
}
void voltageMeterESCUpdate(voltageMeter_t *voltageMeter)
{
#ifndef USE_ESC_SENSOR
    UNUSED(voltageMeter);
#else
    escSensorData_t *escData = getEscSensorData(ESC_SENSOR_COMBINED);
    voltageMeter->unfiltered = escData->dataAge <= ESC_BATTERY_AGE_MAX ? escData->voltage / 10 : 0;
    voltageMeter->filtered = biquadFilterApply(&escvBatFilter, voltageMeter->unfiltered);
#endif
}

