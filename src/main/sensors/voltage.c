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

voltageMeterState_t voltageMeterStates[MAX_VOLTAGE_METERS];

static const uint8_t voltageMeterAdcChannelMap[] = {
#ifdef ADC_BATTERY
    ADC_BATTERY,
#endif
#ifdef ADC_POWER_12V
    ADC_POWER_12V,
#endif
#ifdef ADC_POWER_5V
    ADC_POWER_5V,
#endif
};

PG_REGISTER_ARR_WITH_RESET_FN(voltageMeterConfig_t, MAX_VOLTAGE_METERS, voltageMeterConfig, PG_VOLTAGE_METER_CONFIG, 0);

void pgResetFn_voltageMeterConfig(voltageMeterConfig_t *instance)
{
    for (int i = 0; i < MAX_VOLTAGE_METERS; i++) {
        RESET_CONFIG(voltageMeterConfig_t, &instance[i],
            .vbatscale = VBAT_SCALE_DEFAULT,
            .vbatresdivval = VBAT_RESDIVVAL_DEFAULT,
            .vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT,
        );
    }
}

STATIC_UNIT_TESTED uint16_t voltageAdcToVoltage(const uint16_t src, voltageMeterConfig_t *config)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 10:1 voltage divider (10k:1k) * 10 for 0.1V
    return ((((uint32_t)src * config->vbatscale * 33 + (0xFFF * 5)) / (0xFFF * config->vbatresdivval)) / config->vbatresdivmultiplier);
}

voltageMeterConfig_t *getVoltageMeterConfig(const uint8_t channel)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_METERS && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        if (voltageMeterAdcChannelMap[i] == channel) {
            return voltageMeterConfig(i);
        }
    }

    failureMode(FAILURE_DEVELOPER);

    return NULL;
}

// filtered - uses pre-calculated value
uint16_t getVoltageForADCChannel(uint8_t channel)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_METERS && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        if (voltageMeterAdcChannelMap[i] == channel) {
            voltageMeterState_t *state = &voltageMeterStates[i];
            return state->vbat;
        }
    }

    failureMode(FAILURE_DEVELOPER);

    return 0;
}

voltageMeterState_t *getVoltageMeter(uint8_t index)
{
    return &voltageMeterStates[index];
}

// unfiltered - always recalcualates voltage based on last adc sensor reading
uint16_t getLatestVoltageForADCChannel(uint8_t channel)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_METERS && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        if (voltageMeterAdcChannelMap[i] == channel) {
            voltageMeterState_t *state = &voltageMeterStates[i];
            voltageMeterConfig_t *config = voltageMeterConfig(i);

            return voltageAdcToVoltage(state->vbatLatestADC, config);
        }
    }

    failureMode(FAILURE_DEVELOPER);

    return 0;
}

void voltageMeterUpdate(void)
{
    uint16_t vbatSample;

    for (uint8_t i = 0; i < MAX_VOLTAGE_METERS && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        // store the battery voltage with some other recent battery voltage readings

        voltageMeterState_t *state = &voltageMeterStates[i];
        voltageMeterConfig_t *config = voltageMeterConfig(i);

        uint8_t channel = voltageMeterAdcChannelMap[i];
        vbatSample = state->vbatLatestADC = adcGetChannel(channel);

        vbatSample = biquadFilterApply(&state->vbatFilterState, vbatSample);

        // always calculate the latest voltage, see getLatestVoltage() which does the calculation on demand.
        state->vbat = voltageAdcToVoltage(vbatSample, config);
    }
}

void voltageMeterInit(void)
{
    for (uint8_t i = 0; i < MAX_VOLTAGE_METERS && i < ARRAYLEN(voltageMeterAdcChannelMap); i++) {
        // store the battery voltage with some other recent battery voltage readings

        voltageMeterState_t *state = &voltageMeterStates[i];

        biquadFilterInitLPF(&state->vbatFilterState, VBATT_LPF_FREQ, 50000);
    }
}
