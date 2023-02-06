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
#include <string.h>

#include "platform.h"

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/resource.h"
#include "drivers/dma.h"

#include "drivers/sensor.h"

#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "pg/adc.h"


const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_APB2(ADC1),
        .dmaResource = NULL
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_APB2(ADC2),
        .dmaResource = NULL

    },
    {
        .ADCx = ADC3,
        .rccADC = RCC_APB2(ADC3),
        .dmaResource = NULL
    },
};

adcDevice_t adcDevice[ADCDEV_COUNT];

#define ADC_CHANNEL_VREFINT         ADC_CHANNEL_17
#define ADC_CHANNEL_TEMPSENSOR_ADC1 ADC_CHANNEL_16

const adcTagMap_t adcTagMap[] = {
#ifdef USE_ADC_INTERNAL
#define ADC_TAG_MAP_VREFINT    0
#define ADC_TAG_MAP_TEMPSENSOR 1
    { DEFIO_TAG_E__NONE, ADC_DEVICES_1,   ADC_CHANNEL_VREFINT,         17 },
    { DEFIO_TAG_E__NONE, ADC_DEVICES_1,   ADC_CHANNEL_TEMPSENSOR_ADC1, 16 },
#endif

    { DEFIO_TAG_E__PA0,  ADC_DEVICES_123,  ADC_CHANNEL_0,   0  },
    { DEFIO_TAG_E__PA1,  ADC_DEVICES_123,  ADC_CHANNEL_1,   1  },
    { DEFIO_TAG_E__PA2,  ADC_DEVICES_123,  ADC_CHANNEL_2,   2  },
    { DEFIO_TAG_E__PA3,  ADC_DEVICES_123,  ADC_CHANNEL_3,   3  },
    { DEFIO_TAG_E__PA4,  ADC_DEVICES_12,   ADC_CHANNEL_4,   4  },
    { DEFIO_TAG_E__PA5,  ADC_DEVICES_12,   ADC_CHANNEL_5,   5  },
    { DEFIO_TAG_E__PA6,  ADC_DEVICES_12,   ADC_CHANNEL_6,   6  },
    { DEFIO_TAG_E__PA7,  ADC_DEVICES_12,   ADC_CHANNEL_7,   7  },
    { DEFIO_TAG_E__PB0,  ADC_DEVICES_12,   ADC_CHANNEL_8,   8  },
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_12,   ADC_CHANNEL_9,   9  },
    { DEFIO_TAG_E__PC0,  ADC_DEVICES_123,  ADC_CHANNEL_10,  10 },
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_123,  ADC_CHANNEL_11,  11 },
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_123,  ADC_CHANNEL_12,  12 },
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_123,  ADC_CHANNEL_13,  13 },
    { DEFIO_TAG_E__PC4,  ADC_DEVICES_12,   ADC_CHANNEL_14,  14 },
    { DEFIO_TAG_E__PC5,  ADC_DEVICES_12,   ADC_CHANNEL_15,  15 },
};

void adcInitDevice(adcDevice_t *adcdev, int channelCount)
{
    adc_base_config_type adc_base_struct;
    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = TRUE;
    adc_base_struct.repeat_mode = TRUE;
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
    adc_base_struct.ordinary_channel_length = channelCount;
    adc_base_config(adcdev->ADCx, &adc_base_struct);
    adc_resolution_set(adcdev->ADCx, ADC_RESOLUTION_12B);
}

int adcFindTagMapEntry(ioTag_t tag)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].tag == tag) {
            return i;
        }
    }
    return -1;
}

volatile DMA_DATA uint32_t adcConversionBuffer[ADC_CHANNEL_COUNT];

void adcInit(const adcConfig_t *config)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memcpy(adcDevice, adcHardware, sizeof(adcDevice));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;
    }

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        int map;
        int dev;

        if (i == ADC_TEMPSENSOR) {
            map = ADC_TAG_MAP_TEMPSENSOR;
            dev = ADCDEV_1;
        } else if (i == ADC_VREFINT) {
            map = ADC_TAG_MAP_VREFINT;
            dev = ADCDEV_1;
        } else {
            if (!adcOperatingConfig[i].tag) {
                continue;
            }

            map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
            if (map < 0) {
                continue;
            }

            for (dev = 0; dev < ADCDEV_COUNT; dev++) {
#ifndef USE_DMA_SPEC
                if (!adcDevice[dev].ADCx || !adcDevice[dev].dmaResource) {
                    continue;
                }
#else
                if (!adcDevice[dev].ADCx) {
                    continue;
                }
#endif

                if (adcTagMap[map].devices & (1 << dev)) {
                    break;
                }

                if (dev == ADCDEV_COUNT) {
                    continue;
                }
            }
        }

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].sampleTime = ADC_SAMPLING_INTERVAL_5CYCLES;
        adcOperatingConfig[i].enabled = true;

        adcDevice[dev].channelBits |= (1 << adcTagMap[map].channelOrdinal);

        if (adcOperatingConfig[i].tag) {
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG,GPIO_DRIVE_STRENGTH_MODERATE, 0, GPIO_PULL_NONE));
        }
    }

    int  dmaBufferIndex = 0;
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        RCC_ClockCmd(adc->rccADC, ENABLE);

        adc_reset();
        if (adc->ADCx == ADC1) {
            adc_common_config_type adc_common_struct;
            adc_common_default_para_init(&adc_common_struct);
            adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
            adc_common_struct.div = ADC_HCLK_DIV_4;
            adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
            adc_common_struct.common_dma_request_repeat_state = FALSE;
            adc_common_struct.sampling_interval = ADC_SAMPLING_INTERVAL_5CYCLES;
            adc_common_struct.tempervintrv_state = TRUE;
            adc_common_struct.vbat_state = TRUE;
            adc_common_config(&adc_common_struct);
        }

        int configuredAdcChannels = BITCOUNT(adc->channelBits);
        adcInitDevice(adc, configuredAdcChannels);

#ifdef USE_DMA_SPEC
        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
        dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);
        if (!dmaSpec || !dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
            return;
        }

        dmaEnable(dmaIdentifier);
        xDMA_DeInit(dmaSpec->ref);

        adc->dmaResource=dmaSpec->ref;

        dma_init_type dma_init_struct;
        dma_default_para_init(&dma_init_struct);
        dma_init_struct.buffer_size = BITCOUNT(adc->channelBits);
        dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
        dma_init_struct.memory_inc_enable = TRUE;
        dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
        dma_init_struct.peripheral_inc_enable = FALSE;
        dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
        dma_init_struct.loop_mode_enable = TRUE;

        dma_init_struct.memory_base_addr = (uint32_t)&(adcConversionBuffer[dmaBufferIndex]);
        dma_init_struct.peripheral_base_addr = (uint32_t)&(adc->ADCx->odt);

        xDMA_Init(dmaSpec->ref, &dma_init_struct);
        dmaMuxEnable(dmaIdentifier, dmaSpec->dmaMuxId);

        /* enable dma transfer complete interrupt */
        xDMA_ITConfig(dmaSpec->ref,DMA_IT_TCIF,ENABLE);
        xDMA_Cmd(dmaSpec->ref,ENABLE);

        adc_dma_mode_enable(adc->ADCx, TRUE);
        adc_dma_request_repeat_enable(adc->ADCx, TRUE);
#endif //end of USE_DMA_SPEC

        for (int adcChan = 0; adcChan < ADC_CHANNEL_COUNT; adcChan++) {

            if (!adcOperatingConfig[adcChan].enabled) {
                continue;
            }

            if (adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }

            adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;
            adc_ordinary_channel_set(adcDevice[dev].ADCx,
                adcOperatingConfig[adcChan].adcChannel,
                adcOperatingConfig[adcChan].dmaIndex+1,
                ADC_SAMPLETIME_92_5);
        }


        if (adc->ADCx==ADC1) {
            adc_voltage_monitor_threshold_value_set(ADC1, 0x100, 0x000);
            adc_voltage_monitor_single_channel_select(ADC1, adcOperatingConfig[ADC_BATTERY].adcChannel);
            adc_voltage_monitor_enable(ADC1, ADC_VMONITOR_SINGLE_ORDINARY);
        }

        adc_interrupt_enable(adc->ADCx, ADC_OCCO_INT, TRUE);

        adc_enable(adc->ADCx, TRUE);
        while (adc_flag_get(adc->ADCx, ADC_RDY_FLAG) == RESET);

        dmaBufferIndex += BITCOUNT(adc->channelBits);

        adc_calibration_init(adc->ADCx);
        while (adc_calibration_init_status_get(adc->ADCx));
        adc_calibration_start(adc->ADCx);
        while (adc_calibration_status_get(adc->ADCx));

        adc_enable(adc->ADCx, TRUE);
        adc_ordinary_software_trigger_enable(adc->ADCx, TRUE);
    }
}

void adcGetChannelValues(void)
{
    for (int i = 0; i < ADC_CHANNEL_INTERNAL_FIRST_ID; i++) {
        if (adcOperatingConfig[i].enabled) {
            adcValues[adcOperatingConfig[i].dmaIndex] = adcConversionBuffer[adcOperatingConfig[i].dmaIndex];
        }
    }
}

#ifdef USE_ADC_INTERNAL

bool adcInternalIsBusy(void)
{
    return false;
}

void adcInternalStartConversion(void)
{
    return;
}

uint16_t adcInternalRead(int channel)
{
    int dmaIndex = adcOperatingConfig[channel].dmaIndex;
    return adcConversionBuffer[dmaIndex];
}

int adcPrivateVref = -1;
int adcPrivateTemp = -1;

uint16_t adcInternalReadVrefint(void)
{
    uint16_t value = adcInternalRead(ADC_VREFINT);
    adcPrivateVref =((double)1.2 * 4095) / value * 1000;
    return adcPrivateVref;
}

uint16_t adcInternalReadTempsensor(void)
{
    uint16_t value = adcInternalRead(ADC_TEMPSENSOR);
    adcPrivateTemp = (((ADC_TEMP_BASE- value * ADC_VREF / 4096) / ADC_TEMP_SLOPE) + 25);
    return adcPrivateTemp;
}

void ADC1_2_3_IRQHandler(void)
{
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }
        if(adc_flag_get(adc->ADCx, ADC_OCCO_FLAG) != RESET) {
            adc_flag_clear(adc->ADCx, ADC_OCCO_FLAG);
        }
    }
}

#endif // USE_ADC_INTERNAL
#endif // USE_ADC
