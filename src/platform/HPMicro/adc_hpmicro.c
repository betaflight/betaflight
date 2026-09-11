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

// ADC driver for HPMicro (ADC12 on HPM6750, ADC16 on HPM6360).
// Uses periodic conversion mode; adcGetChannelValues() reads the latest result
// of each configured source.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ADC

#include "drivers/io.h"
#include "io_hpmicro.h"
#include "drivers/adc.h"
#include "platform/adc_impl.h"
#ifdef DEBUG_ADC_CHANNELS
#include "build/debug.h"
#endif
#ifdef HPM6750
#include "hpm_adc12_drv.h"
#elif defined(HPM6360)
#include "hpm_adc16_drv.h"
#endif
#include "hpm_clock_drv.h"
#include "pg/adc.h"

#define ADC_PERIOD_PRESCALE 15U
#define ADC_PERIOD_COUNT     5U

adcOperatingConfig_t adcOperatingConfig[ADC_SOURCE_COUNT];
const adcDevice_t adcHardware[] = {
    {
        .ADCx = HPM_ADC0,
        .rccADC = clock_adc0,
    },
    {
        .ADCx = HPM_ADC1,
        .rccADC = clock_adc1,
    },
    {
        .ADCx = HPM_ADC2,
        .rccADC = clock_adc2,
    },
};

adcDevice_t adcDevice[ADCDEV_COUNT];
volatile DMA_DATA_ZERO_INIT uint16_t adcValues[ADC_SOURCE_COUNT];
/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
#ifdef HPM6750
    { DEFIO_TAG_E__PE14, ADC_DEVICES_1, 0, },
    { DEFIO_TAG_E__PE15, ADC_DEVICES_1, 1, },
    { DEFIO_TAG_E__PE16, ADC_DEVICES_1, 2, },
    { DEFIO_TAG_E__PE17, ADC_DEVICES_1, 3, },
    { DEFIO_TAG_E__PE18, ADC_DEVICES_1, 4, },
    { DEFIO_TAG_E__PE19, ADC_DEVICES_1, 5, },
    { DEFIO_TAG_E__PE20, ADC_DEVICES_1, 6, },
    { DEFIO_TAG_E__PE21, ADC_DEVICES_1, 7, },
    { DEFIO_TAG_E__PE22, ADC_DEVICES_1, 8, },
    { DEFIO_TAG_E__PE23, ADC_DEVICES_1, 9, },
    { DEFIO_TAG_E__PE24, ADC_DEVICES_1, 10, },
    { DEFIO_TAG_E__PE25, ADC_DEVICES_1, 11, },
    { DEFIO_TAG_E__PE26, ADC_DEVICES_1, 12 },
    { DEFIO_TAG_E__PE27, ADC_DEVICES_1, 13 },
    { DEFIO_TAG_E__PE28, ADC_DEVICES_1, 14 },
    { DEFIO_TAG_E__PE29, ADC_DEVICES_1, 15 },
    { DEFIO_TAG_E__PE30, ADC_DEVICES_1, 16 },
    { DEFIO_TAG_E__PE31, ADC_DEVICES_2, 13, },
    { DEFIO_TAG_E__PF0, ADC_DEVICES_2, 14, },
    { DEFIO_TAG_E__PF1, ADC_DEVICES_2, 15, },
    { DEFIO_TAG_E__PF2, ADC_DEVICES_3, 12, },
    { DEFIO_TAG_E__PF3, ADC_DEVICES_3, 13, },
    { DEFIO_TAG_E__PF4, ADC_DEVICES_3, 14, },
#endif
#ifdef HPM6360
    /* ADC0: PC4-PC19 -> ch0-15 */
    { DEFIO_TAG_E__PC4, ADC_DEVICES_1, 0, },
    { DEFIO_TAG_E__PC5, ADC_DEVICES_1, 1, },
    { DEFIO_TAG_E__PC6, ADC_DEVICES_1, 2, },
    { DEFIO_TAG_E__PC7, ADC_DEVICES_1, 3, },
    { DEFIO_TAG_E__PC8, ADC_DEVICES_1, 4, },
    { DEFIO_TAG_E__PC9, ADC_DEVICES_1, 5, },
    { DEFIO_TAG_E__PC10, ADC_DEVICES_1, 6, },
    { DEFIO_TAG_E__PC11, ADC_DEVICES_1, 7, },
    { DEFIO_TAG_E__PC12, ADC_DEVICES_2, 4, },
    { DEFIO_TAG_E__PC13, ADC_DEVICES_2, 5, },
    { DEFIO_TAG_E__PC14, ADC_DEVICES_2, 6, },
    { DEFIO_TAG_E__PC15, ADC_DEVICES_2, 7, },
    { DEFIO_TAG_E__PC16, ADC_DEVICES_2, 8, },
    { DEFIO_TAG_E__PC17, ADC_DEVICES_2, 9, },
    { DEFIO_TAG_E__PC18, ADC_DEVICES_2, 10, },
    { DEFIO_TAG_E__PC19, ADC_DEVICES_2, 11, },
    { DEFIO_TAG_E__PC20, ADC_DEVICES_3, 8, },
    { DEFIO_TAG_E__PC21, ADC_DEVICES_3, 9, },
    { DEFIO_TAG_E__PC22, ADC_DEVICES_3, 10, },
    { DEFIO_TAG_E__PC23, ADC_DEVICES_3, 11, },
    { DEFIO_TAG_E__PC24, ADC_DEVICES_3, 12, },
    { DEFIO_TAG_E__PC25, ADC_DEVICES_3, 13, },
    { DEFIO_TAG_E__PC26, ADC_DEVICES_3, 14, },
    { DEFIO_TAG_E__PC27, ADC_DEVICES_3, 15, },
#endif
};

int adcFindTagMapEntry(ioTag_t tag)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].tag == tag) {
            return i;
        }
    }
    return -1;
}

static bool adcInitDevice(ADC_TypeDef *adcdev)
{
#ifdef HPM6750
    adc12_config_t cfg;

    /* initialize an ADC instance */
    adc12_get_default_config(&cfg);

    cfg.res = adc12_res_12_bits;
    cfg.conv_mode = adc12_conv_mode_period;
    cfg.adc_clk_div = adc12_clock_divider_4;
    cfg.sel_sync_ahb = true;

    /* adc12 initialization */
    if (adc12_init(adcdev, &cfg) != status_success) {
        return false;
    }
#elif defined(HPM6360)
    adc16_config_t cfg;

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);

    cfg.res = adc16_res_16_bits;
    cfg.conv_mode = adc16_conv_mode_period;
    cfg.adc_clk_div = adc16_clock_divider_4;
    cfg.sel_sync_ahb = true;

    /* adc16 initialization */
    if (adc16_init(adcdev, &cfg) != status_success) {
        return false;
    }
#endif

    return true;
}

void adcInit(const adcConfig_t *config)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memcpy(adcDevice, adcHardware, sizeof(adcDevice));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
        adcOperatingConfig[ADC_BATTERY].adcDevice = config->vbat.device;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;  // RSSI_ADC_CHANNEL;
        adcOperatingConfig[ADC_RSSI].adcDevice = config->rssi.device;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag;        // EXTERNAL1_ADC_CHANNEL;
        adcOperatingConfig[ADC_EXTERNAL1].adcDevice = config->external1.device;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;    // CURRENT_METER_ADC_CHANNEL;
        adcOperatingConfig[ADC_CURRENT].adcDevice = config->current.device;
    }
    for (int i = 0; i < ADC_SOURCE_COUNT; i++) {
        int map;
        int dev;

        if (!adcOperatingConfig[i].tag) {
            continue;
        }

        map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
        if (map < 0) {
            continue;
        }
        // Prefer the configured ADC instance when it supports this pin.
        // Otherwise, fall back to the first compatible instance in the tag map.
        dev = ADC_CFG_TO_DEV(adcOperatingConfig[i].adcDevice);
        if ((unsigned) dev >= ADCDEV_COUNT || !adcDevice[dev].ADCx || !(adcTagMap[map].devices & (1U << dev))) {
            for (dev = 0; dev < ADCDEV_COUNT; dev++) {
                if (adcDevice[dev].ADCx && (adcTagMap[map].devices & (1U << dev))) {
                    break;
                }
            }
        }

        if (dev == ADCDEV_COUNT) {
            // No valid device found, go next channel.
            continue;
        }
        // At this point, map is an entry for the input pin and dev is a valid ADCx for the pin for input i

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].sampleTime = 20U;
        adcOperatingConfig[i].enabled = true;

        adcDevice[dev].channelBits |= (1 << adcTagMap[map].channel);

        // Configure a pin for ADC
        if (adcOperatingConfig[i].tag) {
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIOAF(IOGetByTag(adcOperatingConfig[i].tag), IOCFG_ANALOG, IOC_PAD_FUNC_CTL_ANALOG_MASK);
        }
    }
    // DeInit ADCx with inputs
    // We have to batch call DeInit() for all devices as DeInit() initializes ADCx_COMMON register.

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }
        clock_add_to_group(adc->rccADC, 0);
        clock_set_adc_source(adc->rccADC, clk_adc_src_ahb0);
#ifdef HPM6750
        if (adc12_deinit(adc->ADCx) != status_success) {
            for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {
                if (adcOperatingConfig[adcChan].adcDevice == dev) {
                    adcOperatingConfig[adcChan].enabled = false;
                }
            }
            adc->channelBits = 0;
            continue;
        }
#elif defined(HPM6360)
        if (adc16_deinit(adc->ADCx) != status_success) {
            for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {
                if (adcOperatingConfig[adcChan].adcDevice == dev) {
                    adcOperatingConfig[adcChan].enabled = false;
                }
            }
            adc->channelBits = 0;
            continue;
        }
#endif
    }
    // Configure ADCx with inputs

    int dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        if (!adcInitDevice(adc->ADCx)) {
            for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {
                if (adcOperatingConfig[adcChan].adcDevice == dev) {
                    adcOperatingConfig[adcChan].enabled = false;
                }
            }
            adc->channelBits = 0;
            continue;
        }
        // Configure channels

        for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {
            if (!adcOperatingConfig[adcChan].enabled) {
                continue;
            }

            if (adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }
#ifdef HPM6750
            adc12_channel_config_t chCfg;
            /* get a default channel config */
            adc12_get_channel_default_config(&chCfg);
            chCfg.ch = adcOperatingConfig[adcChan].adcChannel;
            chCfg.sample_cycle = adcOperatingConfig[adcChan].sampleTime;
            adc12_prd_config_t prdCfg;

            prdCfg.ch = adcOperatingConfig[adcChan].adcChannel;
            prdCfg.prescale = ADC_PERIOD_PRESCALE;
            prdCfg.period_count = ADC_PERIOD_COUNT;

            const bool configured = adc12_init_channel(adc->ADCx, &chCfg) == status_success
                                    && adc12_set_prd_config(adc->ADCx, &prdCfg) == status_success;
#elif defined(HPM6360)
            adc16_channel_config_t chCfg;
            /* get a default channel config */
            adc16_get_channel_default_config(&chCfg);
            chCfg.ch = adcOperatingConfig[adcChan].adcChannel;
            chCfg.sample_cycle = adcOperatingConfig[adcChan].sampleTime;
            adc16_prd_config_t prdCfg;

            prdCfg.ch = adcOperatingConfig[adcChan].adcChannel;
            prdCfg.prescale = ADC_PERIOD_PRESCALE;
            prdCfg.period_count = ADC_PERIOD_COUNT;

            const bool configured = adc16_init_channel(adc->ADCx, &chCfg) == status_success
                                    && adc16_set_prd_config(adc->ADCx, &prdCfg) == status_success;
#endif

            if (!configured) {
                adcOperatingConfig[adcChan].enabled = false;
                continue;
            }

            adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;
        }
    }
}

FAST_CODE void adcGetChannelValues(void)
{
    for (int i = 0; i < ADC_SOURCE_COUNT; i++) {
        if (adcOperatingConfig[i].enabled) {
            uint16_t result;
            hpm_stat_t status;
            adcDevice_t *adc = &adcDevice[adcOperatingConfig[i].adcDevice];
#ifdef HPM6750
            status = adc12_get_prd_result(adc->ADCx, adcOperatingConfig[i].adcChannel, &result);
            if (status == status_success) {
                adcValues[adcOperatingConfig[i].dmaIndex] = result;
            }
#elif defined(HPM6360)
            status = adc16_get_prd_result(adc->ADCx, adcOperatingConfig[i].adcChannel, &result);
            if (status == status_success) {
                adcValues[adcOperatingConfig[i].dmaIndex] = result >> 4;
            }
#endif
        }
    }
}

#if PLATFORM_TRAIT_ADC_DEVICE
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
    return ADCINVALID;
}
#endif

uint16_t adcGetValue(adcSource_e source)
{
    adcGetChannelValues();

#ifdef DEBUG_ADC_CHANNELS
    for (int i = 0; i < MIN(4, ARRAYLEN(adcOperatingConfig)); i++) {
        if (adcOperatingConfig[i].enabled) {
            debug[i] = adcValues[adcOperatingConfig[i].dmaIndex];
        }
    }
#endif

    if ((unsigned) source >= ADC_SOURCE_COUNT || !adcOperatingConfig[source].enabled) {
        return 0;
    }

    const unsigned dmaIndex = adcOperatingConfig[source].dmaIndex;

    return (dmaIndex < ARRAYLEN(adcValues)) ? adcValues[dmaIndex] : 0;
}

#if PLATFORM_TRAIT_ADC_DEVICE
void platform_pgResetFn_adcConfig(adcConfig_t *config)
{
    config->device = ADC_DEV_TO_CFG(adcDeviceByInstance(ADC_INSTANCE));
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

#endif
