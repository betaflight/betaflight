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

#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/system.h"

#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"
#include "dma.h"

#include "drivers/sensor.h"

#include "adc.h"
#include "adc_impl.h"

const adcDevice_t adcHardware[] = {
    { .ADCx = ADC1, .rccADC = RCC_APB2(ADC1), .DMAy_Streamx = ADC1_DMA_STREAM, .channel = DMA_Channel_0 },
#if !defined(STM32F411xE)
    { .ADCx = ADC2, .rccADC = RCC_APB2(ADC2), .DMAy_Streamx = ADC2_DMA_STREAM, .channel = DMA_Channel_1 },
    { .ADCx = ADC3, .rccADC = RCC_APB2(ADC3), .DMAy_Streamx = ADC3_DMA_STREAM, .channel = DMA_Channel_2 }
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

#define VREFINT_CAL_ADDR 0x1FFF7A2A
#define TSCAL1_LOCATION 0x1FFF7A2C
#define TSCAL2_LOCATION 0x1FFF7A2E

#ifdef USE_VREFINT
static uint16_t adc_TSCAL1;
static uint16_t adc_TSCAL2;
static uint16_t adc_VREFIN_CAL;
static bool isVrefintStatic;

void adcGetCalibrationData(void)
{
    adc_VREFIN_CAL = *(uint16_t *)VREFINT_CAL_ADDR;
    adc_TSCAL1 = *(uint16_t *)TSCAL1_LOCATION;
    adc_TSCAL2 = *(uint16_t *)TSCAL2_LOCATION;
}

// Initialize a pin for ADC

void adcSimpleConfigurePin(ioTag_t iotag) {
    IOConfigGPIO(IOGetByTag(iotag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
}

// Initialize ADC1 for single conversion

void adcSimpleConfigure(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;    // conversion is synchronous with TIM1 and CC1 (actually I'm not sure about this one :/)
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_NbrOfConversion = 1;    //I think this one is clear :p
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;    //The scan is configured in one channel

    ADC_Init(ADC1, &ADC_InitStructure);

    //Enable ADC conversion
    ADC_Cmd(ADC1,ENABLE);
}

uint16_t adcSimpleConversion(uint16_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_480Cycles);

    ADC_SoftwareStartConv(ADC1);//Start the conversion

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion

    return ADC_GetConversionValue(ADC1); //Return the converted data
}

#define ADC_VREFINT_SAMPLES 16

void adcSampleVrefintStatic(void)
{
    if (!isVrefintStatic) {
        return;
    }

    adcSimpleConfigure();
    ADC_TempSensorVrefintCmd(ENABLE);
    delayMicroseconds(1000);

    uint16_t vrefint = 0;

    for (int i = 0 ; i < ADC_VREFINT_SAMPLES ; i++) {
        vrefint += adcSimpleConversion(ADC_Channel_Vrefint);
    }

    vrefint /= ADC_VREFINT_SAMPLES;

    adcVrefInternalStatic = vrefint;
    adcVrefInternal = &adcVrefInternalStatic;
}
#endif

void adcInitCommon(adcDevice_t *adc)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    RCC_ClockCmd(adc->rccADC, ENABLE);

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
}

void adcInit(const adcConfig_t *config)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

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
    if (device == ADCINVALID)
        return;

    adcDevice_t adc = adcHardware[device];

    bool adcActive = false;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (i < ADC_CHANNEL_COUNT_EXTERNAL) {
            // ADC channels with input pins
            if (!adcVerifyPin(adcOperatingConfig[i].tag, device)) {
                continue;
            }

            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
            adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
#ifdef USE_VREFINT
        } else if (device == ADCDEV_1) {
            // Internal ADC channels, only supported on ADC1
            if (i == ADC_TEMPERATURE) {
                adcOperatingConfig[i].adcChannel = ADC_Channel_TempSensor;
            } else if (i == ADC_VREFINT) {
                adcOperatingConfig[i].adcChannel = ADC_Channel_Vrefint;
            }
            ADC_TempSensorVrefintCmd(ENABLE);
#endif
        } else {
            continue;
        }

        adcOperatingConfig[i].dmaIndex = configuredAdcChannels++;
        adcOperatingConfig[i].sampleTime = ADC_SampleTime_480Cycles;
        adcOperatingConfig[i].enabled = true;

        adcActive = true;
    }

    if (!adcActive) {
        return;
    }

#ifdef USE_VREFINT
    adcGetCalibrationData();
    adcVrefCalibration = adc_VREFIN_CAL;

    // If configured adc device is ADC1, then DMA will update the VREFINT along with
    // other channels. Otherwise, VREFINT is sampled here once, and probably upon arm.
    if (device == ADCDEV_1) {
        isVrefintStatic = false;
        adcVrefInternal = &adcValues[adcOperatingConfig[ADC_VREFINT].dmaIndex];
    } else {
        isVrefintStatic = true;
        adcSampleVrefintStatic();
    }
#endif

    RCC_ClockCmd(adc.rccADC, ENABLE);

    dmaInit(dmaGetIdentifier(adc.DMAy_Streamx), OWNER_ADC, 0);

    DMA_DeInit(adc.DMAy_Streamx);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&adc.ADCx->DR;
    DMA_InitStructure.DMA_Channel = adc.channel;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = configuredAdcChannels;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = configuredAdcChannels > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(adc.DMAy_Streamx, &DMA_InitStructure);

    DMA_Cmd(adc.DMAy_Streamx, ENABLE);

    adcInitCommon(&adc);

    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_ContinuousConvMode       = ENABLE;
    ADC_InitStructure.ADC_Resolution               = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConv         = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_ExternalTrigConvEdge     = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign                = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion          = configuredAdcChannels;
    ADC_InitStructure.ADC_ScanConvMode             = configuredAdcChannels > 1 ? ENABLE : DISABLE; // 1=scan more that one channel in group

    ADC_Init(adc.ADCx, &ADC_InitStructure);

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

    ADC_SoftwareStartConv(adc.ADCx);
}
