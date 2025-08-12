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

// Verify a pin designated by tag has connection to an ADC instance designated by device
bool adcVerifyPin(ioTag_t tag, adcDevice_e device)
{
    if (!tag) {
        return false;
    }

    for (int map = 0 ; map < ADC_TAG_MAP_COUNT ; map++) {
        if ((adcTagMap[map].tag == tag) && (adcTagMap[map].devices & (1 << device))) {
            return true;
        }
    }

    return false;
}

uint8_t adcChannelByTag(ioTag_t ioTag)
{
    for (uint8_t i = 0; i < ARRAYLEN(adcTagMap); i++) {
        if (ioTag == adcTagMap[i].tag)
            return adcTagMap[i].channel;
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
volatile FAST_DATA_ZERO_INIT uint16_t adcValues[ADC_SOURCE_COUNT];

uint16_t adcGetValue(adcSource_e source)
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
    return adcValues[adcOperatingConfig[source].dmaIndex];
}
