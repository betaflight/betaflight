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
#include "common/utils.h"

#ifdef USE_ADC

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/adc_impl.h"
#include "drivers/io.h"

#include "pg/adc.h"

#include "adc.h"

//#define DEBUG_ADC_CHANNELS

adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];

#if defined(STM32F7)
volatile FAST_DATA_ZERO_INIT uint16_t adcValues[ADC_CHANNEL_COUNT];
#else
volatile uint16_t adcValues[ADC_CHANNEL_COUNT];
#endif

uint8_t adcChannelByTag(ioTag_t ioTag)
{
    for (uint8_t i = 0; i < ARRAYLEN(adcTagMap); i++) {
        if (ioTag == adcTagMap[i].tag)
            return adcTagMap[i].channel;
    }
    return 0;
}

ADCDevice adcDeviceByInstance(ADC_TypeDef *instance)
{
    if (instance == ADC1) {
        return ADCDEV_1;
    }

#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
    if (instance == ADC2) {
        return ADCDEV_2;
    }

    if (instance == ADC3) {
        return ADCDEV_3;
    }
#endif

#ifdef STM32F3
    if (instance == ADC4) {
        return ADCDEV_4;
    }
#endif

    return ADCINVALID;
}

uint16_t adcGetChannel(uint8_t channel)
{
    adcGetChannelValues();

#ifdef DEBUG_ADC_CHANNELS
    if (adcOperatingConfig[0].enabled) {
        debug[0] = adcValues[adcOperatingConfig[0].dmaIndex];
    }
    if (adcOperatingConfig[1].enabled) {
        debug[1] = adcValues[adcOperatingConfig[1].dmaIndex];
    }
    if (adcOperatingConfig[2].enabled) {
        debug[2] = adcValues[adcOperatingConfig[2].dmaIndex];
    }
    if (adcOperatingConfig[3].enabled) {
        debug[3] = adcValues[adcOperatingConfig[3].dmaIndex];
    }
#endif
    return adcValues[adcOperatingConfig[channel].dmaIndex];
}

// Verify a pin designated by tag has connection to an ADC instance designated by device

bool adcVerifyPin(ioTag_t tag, ADCDevice device)
{
    if (!tag) {
        return false;
    }

    for (int map = 0 ; map < ADC_TAG_MAP_COUNT ; map++) {
#if defined(STM32F1)
        UNUSED(device);
        if ((adcTagMap[map].tag == tag)) {
            return true;
        }
#else
        if ((adcTagMap[map].tag == tag) && (adcTagMap[map].devices & (1 << device))) {
            return true;
        }
#endif
    }

    return false;
}

#ifdef USE_ADC_INTERNAL

int32_t adcVREFINTCAL;      // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
int32_t adcTSCAL1;
int32_t adcTSCAL2;
int32_t adcTSSlopeK;

uint16_t adcInternalCompensateVref(uint16_t vrefAdcValue)
{
    // This is essentially a tuned version of
    // __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefAdcValue, ADC_RESOLUTION_12B);
    return (uint16_t)((uint32_t)(adcVREFINTCAL * VREFINT_CAL_VREF) / vrefAdcValue);
}

int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue)
{
    // This is essentially a tuned version of
    // __HAL_ADC_CALC_TEMPERATURE(vrefValue, tempAdcValue, ADC_RESOLUTION_12B);

    return ((((int32_t)((tempAdcValue * vrefValue) / TEMPSENSOR_CAL_VREFANALOG) - adcTSCAL1) * adcTSSlopeK) + 500) / 1000 + TEMPSENSOR_CAL1_TEMP;
}
#endif // USE_ADC_INTERNAL

#else
uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}
#endif
