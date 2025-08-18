/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>

#include "drivers/adc.h"
#include "platform/adc_impl.h"
#ifdef DEBUG_ADC_CHANNELS
#include "build/debug.h"
#include "common/maths.h"
#endif

#ifdef USE_ADC_INTERNAL
int32_t adcVREFINTCAL;      // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
int32_t adcTSCAL1;
int32_t adcTSCAL2;
int32_t adcTSSlopeK;
#endif

// Verify a pin designated by tag has connection to an ADC instance designated by device
bool adcVerifyPin(ioTag_t tag, adcDevice_e device)
{
    if (!tag) {
        return false;
    }

    /* Defensive guard: prevent negative/out-of-range device values causing undefined shifts */
    if (device < ADCDEV_1 || device >= ADCDEV_COUNT) {
        return false;
    }

    for (unsigned map = 0; map < ARRAYLEN(adcTagMap); map++) {
        if ((adcTagMap[map].tag == tag) && (adcTagMap[map].devices & (1 << device))) {
            return true;
        }
    }

    return false;
}

uint32_t adcChannelByTag(ioTag_t ioTag)
{
    for (unsigned i = 0; i < ARRAYLEN(adcTagMap); i++) {
        if (ioTag == adcTagMap[i].tag) {
            return adcTagMap[i].channel;
        }
    }
    return 0;
}

adcDevice_e adcDeviceByInstance(const ADC_TypeDef *instance)
{
    if (instance == ADC1) {
        return ADCDEV_1;
    }

#if defined(ADC2)
    if (instance == ADC2) {
        return ADCDEV_2;
    }
#endif
#if defined(ADC3)
    if (instance == ADC3) {
        return ADCDEV_3;
    }
#endif
#if defined(ADC4)
    if (instance == ADC4) {
        return ADCDEV_4;
    }
#endif
#if defined(ADC5)
    if (instance == ADC5) {
        return ADCDEV_5;
    }
#endif

    return ADCINVALID;
}

adcOperatingConfig_t adcOperatingConfig[ADC_SOURCE_COUNT];
volatile DMA_DATA_ZERO_INIT uint16_t adcValues[ADC_SOURCE_COUNT];

uint16_t adcGetValue(adcSource_e source)
{
    adcGetChannelValues();

#ifdef DEBUG_ADC_CHANNELS
    for (int i = 0 ; i < MIN(4, ARRAYLEN(adcOperatingConfig)) ; i++) {
        if (adcOperatingConfig[i].enabled) {
            debug[i] = adcValues[adcOperatingConfig[i].dmaIndex];
        }
    }
#endif
    if ((unsigned)source >= ADC_SOURCE_COUNT || !adcOperatingConfig[source].enabled) {
        return 0;
    }
    return adcValues[adcOperatingConfig[source].dmaIndex];
}
