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
#include "platform/rcc.h"
#include "drivers/dma.h"
#include "drivers/sensor.h"
#include "drivers/adc.h"
#include "platform/adc_impl.h"

#include "pg/adc.h"

// These are missing from STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h
#ifdef STM32F446xx
#define ADC_Channel_TempSensor ADC_Channel_18
#endif

const adcDevice_t adcHardware[] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_APB2(ADC1),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = DMA_Channel_0
#endif
    },
#if !defined(STM32F411xE)
    {
        .ADCx = ADC2,
        .rccADC = RCC_APB2(ADC2),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = DMA_Channel_1
#endif
    },
    {
        .ADCx = ADC3,
        .rccADC = RCC_APB2(ADC3),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC3_DMA_STREAM,
        .channel = DMA_Channel_2
#endif
    }
#endif
};

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
/*
    { DEFIO_TAG_E__PF3, ADC_DEVICES_3,   ADC_Channel_9  },
    { DEFIO_TAG_E__PF4, ADC_DEVICES_3,   ADC_Channel_14 },
    { DEFIO_TAG_E__PF5, ADC_DEVICES_3,   ADC_Channel_15 },
    { DEFIO_TAG_E__PF6, ADC_DEVICES_3,   ADC_Channel_4  },
    { DEFIO_TAG_E__PF7, ADC_DEVICES_3,   ADC_Channel_5  },
    { DEFIO_TAG_E__PF8, ADC_DEVICES_3,   ADC_Channel_6  },
    { DEFIO_TAG_E__PF9, ADC_DEVICES_3,   ADC_Channel_7  },
    { DEFIO_TAG_E__PF10,ADC_DEVICES_3,   ADC_Channel_8  },
*/
#if defined(STM32F411xE)
    { DEFIO_TAG_E__PC0, ADC_DEVICES_1,   ADC_Channel_10 },
    { DEFIO_TAG_E__PC1, ADC_DEVICES_1,   ADC_Channel_11 },
    { DEFIO_TAG_E__PC2, ADC_DEVICES_1,   ADC_Channel_12 },
    { DEFIO_TAG_E__PC3, ADC_DEVICES_1,   ADC_Channel_13 },
    { DEFIO_TAG_E__PC4, ADC_DEVICES_1,   ADC_Channel_14 },
    { DEFIO_TAG_E__PC5, ADC_DEVICES_1,   ADC_Channel_15 },
    { DEFIO_TAG_E__PB0, ADC_DEVICES_1,   ADC_Channel_8  },
    { DEFIO_TAG_E__PB1, ADC_DEVICES_1,   ADC_Channel_9  },
    { DEFIO_TAG_E__PA0, ADC_DEVICES_1,   ADC_Channel_0  },
    { DEFIO_TAG_E__PA1, ADC_DEVICES_1,   ADC_Channel_1  },
    { DEFIO_TAG_E__PA2, ADC_DEVICES_1,   ADC_Channel_2  },
    { DEFIO_TAG_E__PA3, ADC_DEVICES_1,   ADC_Channel_3  },
    { DEFIO_TAG_E__PA4, ADC_DEVICES_1,   ADC_Channel_4  },
    { DEFIO_TAG_E__PA5, ADC_DEVICES_1,   ADC_Channel_5  },
    { DEFIO_TAG_E__PA6, ADC_DEVICES_1,   ADC_Channel_6  },
    { DEFIO_TAG_E__PA7, ADC_DEVICES_1,   ADC_Channel_7  },
#else
    { DEFIO_TAG_E__PC0, ADC_DEVICES_123, ADC_Channel_10 },
    { DEFIO_TAG_E__PC1, ADC_DEVICES_123, ADC_Channel_11 },
    { DEFIO_TAG_E__PC2, ADC_DEVICES_123, ADC_Channel_12 },
    { DEFIO_TAG_E__PC3, ADC_DEVICES_123, ADC_Channel_13 },
    { DEFIO_TAG_E__PC4, ADC_DEVICES_12,  ADC_Channel_14 },
    { DEFIO_TAG_E__PC5, ADC_DEVICES_12,  ADC_Channel_15 },
    { DEFIO_TAG_E__PB0, ADC_DEVICES_12,  ADC_Channel_8  },
    { DEFIO_TAG_E__PB1, ADC_DEVICES_12,  ADC_Channel_9  },
    { DEFIO_TAG_E__PA0, ADC_DEVICES_123, ADC_Channel_0  },
    { DEFIO_TAG_E__PA1, ADC_DEVICES_123, ADC_Channel_1  },
    { DEFIO_TAG_E__PA2, ADC_DEVICES_123, ADC_Channel_2  },
    { DEFIO_TAG_E__PA3, ADC_DEVICES_123, ADC_Channel_3  },
    { DEFIO_TAG_E__PA4, ADC_DEVICES_12,  ADC_Channel_4  },
    { DEFIO_TAG_E__PA5, ADC_DEVICES_12,  ADC_Channel_5  },
    { DEFIO_TAG_E__PA6, ADC_DEVICES_12,  ADC_Channel_6  },
    { DEFIO_TAG_E__PA7, ADC_DEVICES_12,  ADC_Channel_7  },
#endif
};

#define VREFINT_CAL_ADDR  0x1FFF7A2A
#define TS_CAL1_ADDR      0x1FFF7A2C
#define TS_CAL2_ADDR      0x1FFF7A2E

static void adcInitDevice(ADC_TypeDef *adcdev, int channelCount)
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_ContinuousConvMode       = ENABLE;
    ADC_InitStructure.ADC_Resolution               = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConv         = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_ExternalTrigConvEdge     = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign                = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion          = channelCount;

    // Multiple injected channel seems to require scan conversion mode to be
    // enabled even if main (non-injected) channel count is 1.
#ifdef USE_ADC_INTERNAL
    ADC_InitStructure.ADC_ScanConvMode             = ENABLE;
#else
    ADC_InitStructure.ADC_ScanConvMode             = channelCount > 1 ? ENABLE : DISABLE; // 1=scan more that one channel in group
#endif
    ADC_Init(adcdev, &ADC_InitStructure);
}

#ifdef USE_ADC_INTERNAL
static void adcInitInternalInjected(const adcConfig_t *config)
{
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_InjectedDiscModeCmd(ADC1, DISABLE);
    ADC_InjectedSequencerLengthConfig(ADC1, 2);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_480Cycles);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_TempSensor, 2, ADC_SampleTime_480Cycles);

    adcVREFINTCAL = config->vrefIntCalibration ? config->vrefIntCalibration : *(uint16_t *)VREFINT_CAL_ADDR;
    adcTSCAL1 = config->tempSensorCalibration1 ? config->tempSensorCalibration1 : *(uint16_t *)TS_CAL1_ADDR;
    adcTSCAL2 = config->tempSensorCalibration2 ? config->tempSensorCalibration2 : *(uint16_t *)TS_CAL2_ADDR;

    adcTSSlopeK = (110 - 30) * 1000 / (adcTSCAL2 - adcTSCAL1);
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
        if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) != RESET) {
            adcInternalConversionInProgress = false;
        }
    }

    return adcInternalConversionInProgress;
}

void adcInternalStartConversion(void)
{
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_SoftwareStartInjectedConv(ADC1);

    adcInternalConversionInProgress = true;
}

uint16_t adcInternalReadVrefint(void)
{
    return ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
}

uint16_t adcInternalReadTempsensor(void)
{
    return ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
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
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;  //RSSI_ADC_CHANNEL;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag; //EXTERNAL1_ADC_CHANNEL;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;  //CURRENT_METER_ADC_CHANNEL;
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
        IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
        adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
        adcOperatingConfig[i].dmaIndex = configuredAdcChannels++;
        adcOperatingConfig[i].sampleTime = ADC_SampleTime_480Cycles;
        adcOperatingConfig[i].enabled = true;
    }

#ifndef USE_ADC_INTERNAL
    if (!adcActive) {
        return;
    }
#endif

    RCC_ClockCmd(adc.rccADC, ENABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

#ifdef USE_ADC_INTERNAL
    // If device is not ADC1 or there's no active channel, then initialize ADC1 separately
    if (device != ADCDEV_1 || !adcActive) {
        RCC_ClockCmd(adcHardware[ADCDEV_1].rccADC, ENABLE);
        adcInitDevice(ADC1, 2);
        ADC_Cmd(ADC1, ENABLE);
    }

    // Initialize for injected conversion
    adcInitInternalInjected(config);

    if (!adcActive) {
        return;
    }
#endif

    adcInitDevice(adc.ADCx, configuredAdcChannels);

    uint8_t rank = 1;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcOperatingConfig[i].enabled) {
            continue;
        }
        ADC_RegularChannelConfig(adc.ADCx, adcOperatingConfig[i].adcChannel, rank++, adcOperatingConfig[i].sampleTime);
    }
    ADC_DMARequestAfterLastTransferCmd(adc.ADCx, ENABLE);

    ADC_DMACmd(adc.ADCx, ENABLE);
    ADC_Cmd(adc.ADCx, ENABLE);

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

    DMA_InitTypeDef DMA_InitStructure;

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&adc.ADCx->DR;

#ifdef USE_DMA_SPEC
    DMA_InitStructure.DMA_Channel = dmaSpec->channel;
#else
    DMA_InitStructure.DMA_Channel = adc.channel;
#endif

    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = configuredAdcChannels;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = configuredAdcChannels > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

#ifdef USE_DMA_SPEC
    xDMA_Init(dmaSpec->ref, &DMA_InitStructure);
    xDMA_Cmd(dmaSpec->ref, ENABLE);
#else
    xDMA_Init(adc.dmaResource, &DMA_InitStructure);
    xDMA_Cmd(adc.dmaResource, ENABLE);
#endif

    ADC_SoftwareStartConv(adc.ADCx);
}

void adcGetChannelValues(void)
{
    // Nothing to do
}
#endif
