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
#include "platform/dma.h"
#include "drivers/sensor.h"
#include "drivers/adc.h"
#include "platform/adc_impl.h"

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

    { DEFIO_TAG_E__PC0, ADC_DEVICES_012, ADC_CHANNEL_10 }, // PC0,H7-ADC012_IN10
    { DEFIO_TAG_E__PC1, ADC_DEVICES_012, ADC_CHANNEL_11 }, // PC1,H7-ADC012_IN11
    { DEFIO_TAG_E__PC2, ADC_DEVICES_2,   ADC_CHANNEL_0  }, // PC2,H7-ADC2_IN0
    { DEFIO_TAG_E__PC3, ADC_DEVICES_2,   ADC_CHANNEL_1  }, // PC3,H7-ADC2_IN1
    { DEFIO_TAG_E__PC4, ADC_DEVICES_01,  ADC_CHANNEL_4  }, // PC4,H7-ADC01_IN4
    { DEFIO_TAG_E__PC5, ADC_DEVICES_01,  ADC_CHANNEL_8  }, // PC5,H7-ADC01_IN8
    { DEFIO_TAG_E__PB0, ADC_DEVICES_01,  ADC_CHANNEL_9  }, // PB0,H7-ADC01_IN9
    { DEFIO_TAG_E__PB1, ADC_DEVICES_01,  ADC_CHANNEL_5  }, // PB1,H7-ADC01_IN5
    { DEFIO_TAG_E__PA0, ADC_DEVICES_0,   ADC_CHANNEL_16 }, // PA0,H7-ADC0_IN16
    { DEFIO_TAG_E__PA1, ADC_DEVICES_0,   ADC_CHANNEL_17 }, // PA1,H7-ADC0_IN17
    { DEFIO_TAG_E__PA2, ADC_DEVICES_01,  ADC_CHANNEL_14 }, // PA2,H7-ADC01_IN14
    { DEFIO_TAG_E__PA3, ADC_DEVICES_01,  ADC_CHANNEL_15 }, // PA3,H7-ADC01_IN15
    { DEFIO_TAG_E__PA4, ADC_DEVICES_01,  ADC_CHANNEL_18 }, // PA4,H7-ADC01_IN18
    { DEFIO_TAG_E__PA5, ADC_DEVICES_01,  ADC_CHANNEL_19 }, // PA5,H7-ADC01_IN19
    { DEFIO_TAG_E__PA6, ADC_DEVICES_01,  ADC_CHANNEL_3  }, // PA6,H7-ADC01_IN3
    { DEFIO_TAG_E__PA7, ADC_DEVICES_01,  ADC_CHANNEL_7  }, // PA6,H7-ADC01_IN7
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
    adc_external_trigger_config(adc_periph, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_channel_length_config(adc_periph, ADC_ROUTINE_CHANNEL, channelCount);

    if(PERIPH_INT(ADC0) == adc_periph)
        trigsel_init(TRIGSEL_OUTPUT_ADC0_REGTRG, TRIGSEL_INPUT_TIMER0_CH0);
    else if(PERIPH_INT(ADC1) == adc_periph)
        trigsel_init(TRIGSEL_OUTPUT_ADC1_REGTRG, TRIGSEL_INPUT_TIMER0_CH0);
    else // if(PERIPH_INT(ADC2) == adc_periph)
        trigsel_init(TRIGSEL_OUTPUT_ADC2_REGTRG, TRIGSEL_INPUT_TIMER0_CH0);

    adc_external_trigger_config(adc_periph, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_channel_length_config(adc_periph, ADC_ROUTINE_CHANNEL, channelCount);
}

#ifdef USE_ADC_INTERNAL

static void adcInitInternalInjected(const adcConfig_t *config)
{
    uint32_t adc_periph = PERIPH_INT(ADC2);    // Note: Only H7-ADC2 have temperature sensor, different with F4

    /* ADC clock config */
    rcu_adc_clock_config(IDX_ADC2, RCU_ADCSRC_PER);

    adc_internal_channel_config(ADC_CHANNEL_INTERNAL_TEMPSENSOR, ENABLE);
    /* enable internal reference voltage channel */
    adc_internal_channel_config(ADC_CHANNEL_INTERNAL_VREFINT, ENABLE);
    /* enable high precision temperature sensor channel */
    adc_internal_channel_config(ADC_CHANNEL_INTERNAL_HP_TEMPSENSOR, ENABLE);

    /* ADC contineous function disable */
    adc_special_function_config(adc_periph, ADC_CONTINUOUS_MODE, DISABLE);

    adc_channel_length_config(adc_periph, ADC_INSERTED_CHANNEL, 2);

    adc_inserted_channel_config(adc_periph, 0, ADC_CHANNEL_18, 638); // ADC_Channel_TempSensor
    adc_inserted_channel_config(adc_periph, 1, ADC_CHANNEL_19, 638); // ADC_Channel_Vrefint

    /* ADC trigger config */
    adc_external_trigger_config(adc_periph, ADC_INSERTED_CHANNEL, EXTERNAL_TRIGGER_DISABLE);

    adcVREFINTCAL = config->vrefIntCalibration ? config->vrefIntCalibration : VREFINT_EXPECTED;
    adcTSCAL1 = config->tempSensorCalibration1 ? config->tempSensorCalibration1 : (*(uint16_t *)TEMPSENSOR_CAL1_ADDR & 0x0FFF);
    adcTSCAL2 = config->tempSensorCalibration2 ? config->tempSensorCalibration2 : (*(uint16_t *)TEMPSENSOR_CAL2_ADDR & 0x0FFF);

    // adcTSSlopeK = lrintf(3300.0f*1000.0f/4095.0f/TEMPSENSOR_SLOPE);
    adcTSSlopeK = ((TEMPSENSOR_CAL1_TEMP - TEMPSENSOR_CAL2_TEMP) * 1000) / (adcTSCAL1 - adcTSCAL2);
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
        if (adc_flag_get(PERIPH_INT(ADC2), ADC_FLAG_EOIC) != RESET) {
            adcInternalConversionInProgress = false;
        }
    }

    return adcInternalConversionInProgress;
}

void adcInternalStartConversion(void)
{
    uint32_t adc_periph = PERIPH_INT(ADC2);
    adc_flag_clear(adc_periph, ADC_FLAG_EOIC);
    adc_software_trigger_enable(adc_periph, ADC_INSERTED_CHANNEL);

    adcInternalConversionInProgress = true;
}

uint16_t adcInternalRead(adcSource_e source)
{
    switch (source) {
    case ADC_VREFINT:
        return adc_inserted_data_read(PERIPH_INT(ADC2), ADC_INSERTED_CHANNEL_1);
    case ADC_TEMPSENSOR:
        return adc_inserted_data_read(PERIPH_INT(ADC2), ADC_INSERTED_CHANNEL_0);
    default:
        return 0;
    }
}
#endif

void adcInit(const adcConfig_t *config)
{
    uint8_t i;
    uint8_t configuredAdcChannels = 0;

    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));

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

    adcDevice_e device = ADC_CFG_TO_DEV(config->device);

    if (device == ADCINVALID) {
        return;
    }

    adcDevice_t adc = adcHardware[device];

    bool adcActive = false;
    for (int i = 0; i < ADC_SOURCE_COUNT; i++) {
        if (!adcVerifyPin(adcOperatingConfig[i].tag, device)) {
            continue;
        }

        adcActive = true;
        IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
        IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_OTYPE_OD, GPIO_PUPD_NONE));
        adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
        adcOperatingConfig[i].dmaIndex = configuredAdcChannels++;
        adcOperatingConfig[i].sampleTime = 638;
        adcOperatingConfig[i].enabled = true;
    }

#ifndef USE_ADC_INTERNAL
    if (!adcActive) {
        return;
    }
#endif

    RCC_ClockCmd(adc.rccADC, ENABLE);

    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    adc_clock_config(PERIPH_INT(adc.ADCx), ADC_CLK_SYNC_HCLK_DIV6);
    adc_sync_dma_config(ADC_SYNC_DMA_DISABLE);
    adc_sync_delay_config(ADC_SYNC_DELAY_5CYCLE);

#ifdef USE_ADC_INTERNAL
    uint32_t adc_periph = PERIPH_INT(ADC2);
    // If device is not ADC2 or there's no active channel, then initialize ADC2 separately
    if (device != ADCDEV_2 || !adcActive) {
        // adc_clock_config(adc_periph, ADC_CLK_ASYNC_DIV64);
        RCC_ClockCmd(adcHardware[ADCDEV_2].rccADC, ENABLE);
        adc_clock_config(adc_periph, ADC_CLK_ASYNC_DIV64);
        adcInitDevice(adc_periph, 2);
        adc_enable(adc_periph);
    }

    // Initialize for injected conversion
    adcInitInternalInjected(config);

    adcOperatingConfig[ADC_VREFINT].enabled = true;
    adcOperatingConfig[ADC_TEMPSENSOR].enabled = true;

    if (!adcActive) {
        return;
    }
#endif

    adcInitDevice((uint32_t)(adc.ADCx), configuredAdcChannels); // Note type conversion.

    uint8_t rank = 0;
    for (i = 0; i < ADC_EXTERNAL_COUNT; i++) {
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

    // dma_single_data_parameter_struct dma_init_struct;
    DMA_InitTypeDef dma_init_struct;
    dma_single_data_para_struct_init(&dma_init_struct.config.init_struct_s);
    dma_init_struct.config.init_struct_s.periph_addr = (uint32_t)(&ADC_RDATA((uint32_t)(adc.ADCx)));

#ifdef USE_DMA_SPEC
    dma_init_struct.config.init_struct_s.request = dmaSpec->channel;
#else
    dma_init_struct.config.init_struct_s.request = adc.channel;
#endif

    dma_init_struct.config.init_struct_s.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.config.init_struct_s.memory0_addr = (uint32_t)(adcValues);
    dma_init_struct.config.init_struct_s.memory_inc = configuredAdcChannels > 1 ? DMA_MEMORY_INCREASE_ENABLE : DMA_MEMORY_INCREASE_DISABLE;
    dma_init_struct.config.init_struct_s.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_init_struct.config.init_struct_s.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct.config.init_struct_s.direction = DMA_PERIPH_TO_MEMORY;
    dma_init_struct.config.init_struct_s.number = configuredAdcChannels;
    dma_init_struct.config.init_struct_s.priority = DMA_PRIORITY_HIGH;

#ifdef USE_DMA_SPEC
    gd32_dma_general_init((uint32_t)dmaSpec->ref, &dma_init_struct);
    xDMA_Cmd(dmaSpec->ref, ENABLE);
#else
    gd32_dma_general_init((uint32_t)adc.dmaResource, &dma_init_struct);
    xDMA_Cmd(adc.dmaResource, ENABLE);
#endif

    adc_software_trigger_enable((uint32_t)(adc.ADCx), ADC_ROUTINE_CHANNEL);
}

void adcGetChannelValues(void)
{
    // Nothing to do
}
#endif
