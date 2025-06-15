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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "adcinternal.h"

#if defined(USE_ADC_INTERNAL)

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/adc.h"

typedef struct movingAverageStateUint16_s {
    uint32_t sum;
    uint16_t *values;
    uint8_t size;
    uint8_t pos;
} movingAverageStateUint16_t;

static uint16_t updateMovingAverageUint16(movingAverageStateUint16_t *state, uint16_t newValue)
{
    state->sum -= state->values[state->pos];
    state->values[state->pos] = newValue;
    state->sum += newValue;
    state->pos = (state->pos + 1) % state->size;

    return state->sum / state->size;
}

static uint16_t adcVrefintValue;
static uint16_t adcVrefintValues[8];
movingAverageStateUint16_t adcVrefintAverageState = { 0, adcVrefintValues, 8, 0 } ;

static uint16_t adcTempsensorValue;
static uint16_t adcTempsensorValues[8];
movingAverageStateUint16_t adcTempsensorAverageState = { 0, adcTempsensorValues, 8, 0 } ;

static int16_t coreTemperature;
static uint16_t vrefMv;

uint16_t getVrefMv(void)
{
#ifdef ADC_VOLTAGE_REFERENCE_MV
    return ADC_VOLTAGE_REFERENCE_MV;
#else
    return vrefMv;
#endif
}

int16_t getCoreTemperatureCelsius(void)
{
    return coreTemperature;
}

void adcInternalProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (adcInternalIsBusy()) {
        return;
    }

    uint16_t vrefintSample = adcInternalReadVrefint();
    uint16_t tempsensorSample = adcInternalReadTempsensor();

    adcVrefintValue = updateMovingAverageUint16(&adcVrefintAverageState, vrefintSample);
    adcTempsensorValue = updateMovingAverageUint16(&adcTempsensorAverageState, tempsensorSample);

    vrefMv = adcInternalCompensateVref(adcVrefintValue);
    coreTemperature = adcInternalComputeTemperature(adcTempsensorValue, vrefMv);

    DEBUG_SET(DEBUG_ADC_INTERNAL, 0, coreTemperature);
    DEBUG_SET(DEBUG_ADC_INTERNAL, 1, vrefintSample);
    DEBUG_SET(DEBUG_ADC_INTERNAL, 2, tempsensorSample);
    DEBUG_SET(DEBUG_ADC_INTERNAL, 3, vrefMv);

    adcInternalStartConversion(); // Start next conversion
}

void adcInternalInit(void)
{
    // Call adcInternalProcess repeatedly to fill moving average array
    for (int i = 0 ; i < 9 ; i++) {
        while (adcInternalIsBusy()) {
            // empty
        }
        adcInternalProcess(0);
    }
}
#else
uint16_t getVrefMv(void)
{
#ifdef ADC_VOLTAGE_REFERENCE_MV
    return ADC_VOLTAGE_REFERENCE_MV;
#else
    return 3300;
#endif
}
#endif
