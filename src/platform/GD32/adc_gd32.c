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
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/rcc.h"
#include "drivers/dma.h"
#include "drivers/sensor.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "pg/adc.h"

const adcDevice_t adcHardware[] = {
    {
        .ADCx = (ADC_TypeDef *)ADC0,
        .rccADC = RCC_APB2(ADC0),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC0_DMA_STREAM,
        .channel = DMA_SUBPERI0
#endif
    },
    {
        .ADCx = (ADC_TypeDef *)ADC1,
        .rccADC = RCC_APB2(ADC1),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = DMA_SUBPERI1
#endif
    },
    {
        .ADCx = (ADC_TypeDef *)ADC2,
        .rccADC = RCC_APB2(ADC2),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = DMA_SUBPERI2
#endif
    }
};

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {

    { DEFIO_TAG_E__PC0, ADC_DEVICES_012, ADC_CHANNEL_10 },
    { DEFIO_TAG_E__PC1, ADC_DEVICES_012, ADC_CHANNEL_11 },
    { DEFIO_TAG_E__PC2, ADC_DEVICES_012, ADC_CHANNEL_12 },
    { DEFIO_TAG_E__PC3, ADC_DEVICES_012, ADC_CHANNEL_13 },
    { DEFIO_TAG_E__PC4, ADC_DEVICES_01,  ADC_CHANNEL_14 },
    { DEFIO_TAG_E__PC5, ADC_DEVICES_01,  ADC_CHANNEL_15 },
    { DEFIO_TAG_E__PB0, ADC_DEVICES_01,  ADC_CHANNEL_8  },
    { DEFIO_TAG_E__PB1, ADC_DEVICES_01,  ADC_CHANNEL_9  },
    { DEFIO_TAG_E__PA0, ADC_DEVICES_012, ADC_CHANNEL_0  },
    { DEFIO_TAG_E__PA1, ADC_DEVICES_012, ADC_CHANNEL_1  },
    { DEFIO_TAG_E__PA2, ADC_DEVICES_012, ADC_CHANNEL_2  },
    { DEFIO_TAG_E__PA3, ADC_DEVICES_012, ADC_CHANNEL_3  },
    { DEFIO_TAG_E__PA4, ADC_DEVICES_01,  ADC_CHANNEL_4  },
    { DEFIO_TAG_E__PA5, ADC_DEVICES_01,  ADC_CHANNEL_5  },
    { DEFIO_TAG_E__PA6, ADC_DEVICES_01,  ADC_CHANNEL_6  },
    { DEFIO_TAG_E__PA7, ADC_DEVICES_01,  ADC_CHANNEL_7  },
};


static void adcInitDevice(uint32_t adc_periph, int channelCount)
{
    // Multiple injected channel seems to require scan conversion mode to be
    // enabled even if main (non-injected) channel count is 1.
#ifdef USE_ADC_INTERNAL
    adc_special_function_config(adc_periph, ADC_SCAN_MODE, ENABLE);
#else
    if(channelCount > 1) 
        adc_special_function_config(adc_periph, ADC_SCAN_MODE, ENABLE);
    else 
        adc_special_function_config(adc_periph, ADC_SCAN_MODE, DISABLE);
#endif

    adc_special_function_config(adc_periph, ADC_CONTINUOUS_MODE, ENABLE);
    adc_resolution_config(adc_periph, ADC_RESOLUTION_12B);
    adc_data_alignment_config(adc_periph, ADC_DATAALIGN_RIGHT);

    /* routine channel config */
    adc_external_trigger_source_config(adc_periph, ADC_ROUTINE_CHANNEL, ADC_EXTTRIG_ROUTINE_T0_CH0);
    adc_external_trigger_config(adc_periph, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_channel_length_config(adc_periph, ADC_ROUTINE_CHANNEL, channelCount);
}

#ifdef USE_ADC_INTERNAL

static void adcInitInternalInjected(const adcConfig_t *config)
{
    adc_channel_16_to_18(ADC_TEMP_VREF_CHANNEL_SWITCH, ENABLE);
    adc_discontinuous_mode_config(ADC0, ADC_CHANNEL_DISCON_DISABLE, 0);
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 2);

    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_17, ADC_SAMPLETIME_480); // ADC_Channel_Vrefint
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_16, ADC_SAMPLETIME_480); // ADC_Channel_TempSensor

    adcVREFINTCAL = config->vrefIntCalibration ? config->vrefIntCalibration : VREFINT_EXPECTED;
    adcTSCAL1 = config->tempSensorCalibration1 ? config->tempSensorCalibration1 : ((TEMPSENSOR_CAL1_V * 4095.0f) / 3.3f);

    adcTSSlopeK = lrintf(3300.0f*1000.0f/4095.0f/TEMPSENSOR_SLOPE);
}

// Note on sampling time for temperature sensor and vrefint:
// Both sources have minimum sample time of 10us.
// With prescaler = 8:
// 168MHz : fAPB2 = 84MHz, fADC = 10.5MHz, tcycle = 0.090us, 10us = 105cycle < 144cycle
// 240MHz : fAPB2 = 120MHz, fADC = 15.0MHz, tcycle = 0.067usk 10us = 150cycle < 480cycle
//
// 480cycles@15.0MHz = 32us

static bool adcInternalConversionInProgress = false;

bool adcInternalIsBusy(void)
{
    if (adcInternalConversionInProgress) {
        if (adc_flag_get(ADC0, ADC_FLAG_EOIC) != RESET) {
            adcInternalConversionInProgress = false;
        }
    }

    return adcInternalConversionInProgress;
}

void adcInternalStartConversion(void)
{
    adc_flag_clear(ADC0, ADC_FLAG_EOIC);
    adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);

    adcInternalConversionInProgress = true;
}

uint16_t adcInternalReadVrefint(void)
{
    return adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_1);
}

uint16_t adcInternalReadTempsensor(void)
{
    return adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
}
#endif

void adcInit(const adcConfig_t *config)
{
    uint8_t i;
    uint8_t configuredAdcChannels = 0;

    memset(&adcOperatingConfig, 0, sizeof(adcOperatingConfig));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;           //RSSI_ADC_CHANNEL;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag; //EXTERNAL1_ADC_CHANNEL;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;     //CURRENT_METER_ADC_CHANNEL;
    }

    ADCDevice device = ADC_CFG_TO_DEV(config->device);

    if (device == ADCINVALID) {
        return;
    }

    adcDevice_t adc = adcHardware[device];

    bool adcActive = false;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcVerifyPin(adcOperatingConfig[i].tag, device)) {
            continue;
        }

        adcActive = true;
        IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
        IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_OTYPE_OD, GPIO_PUPD_NONE));
        adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
        adcOperatingConfig[i].dmaIndex = configuredAdcChannels++;
        adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_480;
        adcOperatingConfig[i].enabled = true;
    }

#ifndef USE_ADC_INTERNAL
    if (!adcActive) {
        return;
    }
#endif

    RCC_ClockCmd(adc.rccADC, ENABLE);

    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    adc_clock_config(ADC_ADCCK_PCLK2_DIV8);
    adc_sync_dma_config(ADC_SYNC_DMA_DISABLE);
    adc_sync_delay_config(ADC_SYNC_DELAY_5CYCLE);

#ifdef USE_ADC_INTERNAL
    // If device is not ADC0 or there's no active channel, then initialize ADC0 separately
    if (device != ADCDEV_0 || !adcActive) {
        RCC_ClockCmd(adcHardware[ADCDEV_0].rccADC, ENABLE);
        adcInitDevice(ADC0, 2);
        adc_enable(ADC0);
    }

    // Initialize for injected conversion
    adcInitInternalInjected(config);

    if (!adcActive) {
        return;
    }
#endif

    adcInitDevice((uint32_t)(adc.ADCx), configuredAdcChannels); // Note type conversion.

    uint8_t rank = 0;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcOperatingConfig[i].enabled) {
            continue;
        }
        adc_routine_channel_config((uint32_t)(adc.ADCx), rank++, adcOperatingConfig[i].adcChannel, adcOperatingConfig[i].sampleTime);
    }

    adc_dma_request_after_last_enable((uint32_t)(adc.ADCx));

    adc_dma_mode_enable((uint32_t)(adc.ADCx));
    adc_enable((uint32_t)(adc.ADCx));

#ifdef USE_DMA_SPEC
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, device, config->dmaopt[device]);

    if (!dmaSpec || !dmaAllocate(dmaGetIdentifier(dmaSpec->ref), OWNER_ADC, RESOURCE_INDEX(device))) {
        return;
    }

    dmaEnable(dmaGetIdentifier(dmaSpec->ref));

    xDMA_DeInit(dmaSpec->ref);
#else
    if (!dmaAllocate(dmaGetIdentifier(adc.dmaResource), OWNER_ADC, 0)) {
        return;
    }

    dmaEnable(dmaGetIdentifier(adc.dmaResource));

    xDMA_DeInit(adc.dmaResource);
#endif

    dma_single_data_parameter_struct dma_init_struct;
    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr = (uint32_t)(&ADC_RDATA((uint32_t)(adc.ADCx)));

    uint32_t temp_dma_periph;
    int temp_dma_channel;
#ifdef USE_DMA_SPEC
    gd32_dma_chbase_parse((uint32_t)dmaSpec->ref, &temp_dma_periph, &temp_dma_channel);
    dma_channel_subperipheral_select(temp_dma_periph, temp_dma_channel, dmaSpec->channel);
#else
    gd32_dma_chbase_parse((uint32_t)adc.dmaResource, &temp_dma_periph, &temp_dma_channel);
    dma_channel_subperipheral_select(temp_dma_periph, temp_dma_channel, adc.channel);
#endif

    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory0_addr = (uint32_t)(adcValues);
    dma_init_struct.memory_inc = configuredAdcChannels > 1 ? DMA_MEMORY_INCREASE_ENABLE : DMA_MEMORY_INCREASE_DISABLE;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.number = configuredAdcChannels;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;

#ifdef USE_DMA_SPEC
    gd32_dma_init((uint32_t)dmaSpec->ref, &dma_init_struct);
    xDMA_Cmd(dmaSpec->ref, ENABLE);
#else
    gd32_dma_init((uint32_t)adc.dmaResource, &dma_init_struct);
    xDMA_Cmd(adc.dmaResource, ENABLE);
#endif

    adc_software_trigger_enable((uint32_t)(adc.ADCx), ADC_ROUTINE_CHANNEL);
}

void adcGetChannelValues(void)
{
    // Nothing to do
}
#endif
