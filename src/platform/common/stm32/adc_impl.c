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
#include "pg/adc.h"

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
#if defined(USE_ADC_DEVICE_0)
    if (device < ADCDEV_0 || device >= ADCDEV_COUNT) {
#else
    if (device < ADCDEV_1 || device >= ADCDEV_COUNT) {
#endif
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

#if PLATFORM_TRAIT_ADC_DEVICE
adcDevice_e adcDeviceByInstance(const ADC_TypeDef *instance)
{
#if defined(USE_ADC_DEVICE_0)
    if (instance == ADC0) {
        return ADCDEV_0;
    }
#endif

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
#endif

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

#if PLATFORM_TRAIT_ADC_DEVICE
void platform_pgResetFn_adcConfig(adcConfig_t *config)
{
    config->device = ADC_DEV_TO_CFG(adcDeviceByInstance(ADC_INSTANCE));
#if defined(USE_DMA_SPEC)
#if defined(USE_ADC_DEVICE_0)
    config->dmaopt[ADCDEV_0] = ADC0_DMA_OPT;
#endif
    config->dmaopt[ADCDEV_1] = ADC1_DMA_OPT;
// These conditionals need to match the ones used in 'src/main/drivers/adc.h'.
#if defined(ADC2)
    config->dmaopt[ADCDEV_2] = ADC2_DMA_OPT;
#endif
#if defined(ADC3)
    config->dmaopt[ADCDEV_3] = ADC3_DMA_OPT;
#endif
#if defined(ADC4)
    config->dmaopt[ADCDEV_4] = ADC4_DMA_OPT;
#endif
#if defined(ADC5)
    config->dmaopt[ADCDEV_5] = ADC5_DMA_OPT;
#endif
#endif
#ifdef ADC_RSSI_INSTANCE
    config->rssi.device = ADC_DEV_TO_CFG(adcDeviceByInstance(ADC_RSSI_INSTANCE));
#else
    config->rssi.device = config->device;
#endif
#ifdef ADC_CURR_INSTANCE
    config->current.device = ADC_DEV_TO_CFG(adcDeviceByInstance(ADC_CURR_INSTANCE));
#else
    config->current.device = config->device;
#endif
#ifdef ADC_EXTERNAL1_INSTANCE
    config->external1.device = ADC_DEV_TO_CFG(adcDeviceByInstance(ADC_EXTERNAL1_INSTANCE));
#else
    config->external1.device = config->device;
#endif
#ifdef ADC_VBAT_INSTANCE
    config->vbat.device = ADC_DEV_TO_CFG(adcDeviceByInstance(ADC_VBAT_INSTANCE));
#else
    config->vbat.device = config->device;
#endif
}
#endif
