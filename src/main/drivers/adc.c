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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/time.h"

#include "drivers/io.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "common/utils.h"

#ifndef ADC_INSTANCE
#define ADC_INSTANCE                ADC1
#endif

#ifndef ADC_CHANNEL_1_INSTANCE
#define ADC_CHANNEL_1_INSTANCE  ADC_INSTANCE
#endif
#ifndef ADC_CHANNEL_2_INSTANCE
#define ADC_CHANNEL_2_INSTANCE  ADC_INSTANCE
#endif
#ifndef ADC_CHANNEL_3_INSTANCE
#define ADC_CHANNEL_3_INSTANCE  ADC_INSTANCE
#endif
#ifndef ADC_CHANNEL_4_INSTANCE
#define ADC_CHANNEL_4_INSTANCE  ADC_INSTANCE
#endif

#ifdef USE_ADC

static int adcFunctionMap[ADC_FUNCTION_COUNT];
adc_config_t adcConfig[ADC_CHN_COUNT];  // index 0 is dummy for ADC_CHN_NONE
volatile uint16_t adcValues[ADCDEV_COUNT][ADC_CHN_COUNT];

uint8_t adcChannelByTag(ioTag_t ioTag)
{
    for (uint8_t i = 0; i < ARRAYLEN(adcTagMap); i++) {
        if (ioTag == adcTagMap[i].tag)
            return adcTagMap[i].channel;
    }
    return 0;
}

int adcGetFunctionChannelAllocation(uint8_t function)
{
    return adcFunctionMap[function];
}

bool adcIsFunctionAssigned(uint8_t function)
{
    // Map function to ADC channel
    return (adcFunctionMap[function] != ADC_CHN_NONE);
}

uint16_t adcGetChannel(uint8_t function)
{
    int channel = adcFunctionMap[function];
    if (channel == ADC_CHN_NONE)
        return 0;

    if (adcConfig[channel].adcDevice != ADCINVALID && adcConfig[channel].enabled) {
        return adcValues[adcConfig[channel].adcDevice][adcConfig[channel].dmaIndex];
    } else {
        return 0;
    }
}

static bool isChannelInUse(int channel)
{
    for (int i = 0; i < ADC_FUNCTION_COUNT; i++) {
        if (adcFunctionMap[i] == channel)
            return true;
    }

    return false;
}

#if !defined(ADC_CHANNEL_1_PIN) || !defined(ADC_CHANNEL_2_PIN) || !defined(ADC_CHANNEL_3_PIN) || !defined(ADC_CHANNEL_4_PIN)
static void disableChannelMapping(int channel)
{
    for (int i = 0; i < ADC_FUNCTION_COUNT; i++) {
        if (adcFunctionMap[i] == channel) {
            adcFunctionMap[i] = ADC_CHN_NONE;
        }
    }
}
#endif

void adcInit(drv_adc_config_t *init)
{
    memset(&adcConfig, 0, sizeof(adcConfig));

    // Remember ADC function to ADC channel mapping
    for (int i = 0; i < ADC_FUNCTION_COUNT; i++) {
        if (init->adcFunctionChannel[i] >= ADC_CHN_1 && init->adcFunctionChannel[i] <= ADC_CHN_MAX) {
            adcFunctionMap[i] = init->adcFunctionChannel[i];
        }
        else {
            adcFunctionMap[i] = ADC_CHN_NONE;
        }
    }

#ifdef ADC_CHANNEL_1_PIN
    if (isChannelInUse(ADC_CHN_1)) {
        adcConfig[ADC_CHN_1].adcDevice = adcDeviceByInstance(ADC_CHANNEL_1_INSTANCE);
        if (adcConfig[ADC_CHN_1].adcDevice != ADCINVALID) {
            adcConfig[ADC_CHN_1].tag = IO_TAG(ADC_CHANNEL_1_PIN);
        }
    }
#else
    disableChannelMapping(ADC_CHN_1);
#endif

#ifdef ADC_CHANNEL_2_PIN
    if (isChannelInUse(ADC_CHN_2)) {
        adcConfig[ADC_CHN_2].adcDevice = adcDeviceByInstance(ADC_CHANNEL_2_INSTANCE);
        if (adcConfig[ADC_CHN_2].adcDevice != ADCINVALID) {
            adcConfig[ADC_CHN_2].tag = IO_TAG(ADC_CHANNEL_2_PIN);
        }
    }
#else
    disableChannelMapping(ADC_CHN_2);
#endif

#ifdef ADC_CHANNEL_3_PIN
    if (isChannelInUse(ADC_CHN_3)) {
        adcConfig[ADC_CHN_3].adcDevice = adcDeviceByInstance(ADC_CHANNEL_3_INSTANCE);
        if (adcConfig[ADC_CHN_3].adcDevice != ADCINVALID) {
            adcConfig[ADC_CHN_3].tag = IO_TAG(ADC_CHANNEL_3_PIN);
        }
    }
#else
    disableChannelMapping(ADC_CHN_3);
#endif

#ifdef ADC_CHANNEL_4_PIN
    if (isChannelInUse(ADC_CHN_4)) {
        adcConfig[ADC_CHN_4].adcDevice = adcDeviceByInstance(ADC_CHANNEL_4_INSTANCE);
        if (adcConfig[ADC_CHN_4].adcDevice != ADCINVALID) {
            adcConfig[ADC_CHN_4].tag = IO_TAG(ADC_CHANNEL_4_PIN);
        }
    }
#else
    disableChannelMapping(ADC_CHN_4);
#endif


    adcHardwareInit(init);
}

#else

uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}

#endif
