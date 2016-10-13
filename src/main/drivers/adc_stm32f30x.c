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

#include <platform.h>
#include "system.h"

#include "gpio.h"

#include "sensor.h"
#include "accgyro.h"

#include "adc.h"
#include "adc_impl.h"

#ifdef USE_ADC

void adcInit(drv_adc_config_t *init)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    uint8_t i;
    uint8_t adcChannelCount = 0;

    memset(&adcConfig, 0, sizeof(adcConfig));

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;

#ifdef ADC0_GPIO
    if (init->channelMask & ADC_CHANNEL0_ENABLE) {
        GPIO_InitStructure.GPIO_Pin   = ADC0_GPIO_PIN;
        GPIO_Init(ADC0_GPIO, &GPIO_InitStructure);

        adcConfig[ADC_CHANNEL0].adcChannel = ADC0_CHANNEL;
        adcConfig[ADC_CHANNEL0].dmaIndex = adcChannelCount;
        adcConfig[ADC_CHANNEL0].sampleTime = ADC_SampleTime_601Cycles5;
        adcConfig[ADC_CHANNEL0].enabled = true;
        adcChannelCount++;
    }
#endif

#ifdef ADC1_GPIO
    if (init->channelMask & ADC_CHANNEL1_ENABLE) {
        GPIO_InitStructure.GPIO_Pin = ADC1_GPIO_PIN;
        GPIO_Init(ADC1_GPIO, &GPIO_InitStructure);

        adcConfig[ADC_CHANNEL1].adcChannel = ADC1_CHANNEL;
        adcConfig[ADC_CHANNEL1].dmaIndex = adcChannelCount;
        adcConfig[ADC_CHANNEL1].sampleTime = ADC_SampleTime_601Cycles5;
        adcConfig[ADC_CHANNEL1].enabled = true;
        adcChannelCount++;
    }
#endif

#ifdef ADC2_GPIO
    if (init->channelMask & ADC_CHANNEL2_ENABLE) {
        GPIO_InitStructure.GPIO_Pin = ADC2_GPIO_PIN;
        GPIO_Init(ADC2_GPIO, &GPIO_InitStructure);

        adcConfig[ADC_CHANNEL2].adcChannel = ADC2_CHANNEL;
        adcConfig[ADC_CHANNEL2].dmaIndex = adcChannelCount;
        adcConfig[ADC_CHANNEL2].sampleTime = ADC_SampleTime_601Cycles5;
        adcConfig[ADC_CHANNEL2].enabled = true;
        adcChannelCount++;
    }
#endif

#ifdef ADC3_GPIO
    if (init->channelMask & ADC_CHANNEL3_ENABLE) {
        GPIO_InitStructure.GPIO_Pin   = ADC3_GPIO_PIN;
        GPIO_Init(ADC3_GPIO, &GPIO_InitStructure);

        adcConfig[ADC_CHANNEL3].adcChannel = ADC3_CHANNEL;
        adcConfig[ADC_CHANNEL3].dmaIndex = adcChannelCount;
        adcConfig[ADC_CHANNEL3].sampleTime = ADC_SampleTime_601Cycles5;
        adcConfig[ADC_CHANNEL3].enabled = true;
        adcChannelCount++;
    }
#endif

#ifdef ADC4_GPIO
    if (init->channelMask & ADC_CHANNEL4_ENABLE) {
        GPIO_InitStructure.GPIO_Pin = ADC4_GPIO_PIN;
        GPIO_Init(ADC4_GPIO, &GPIO_InitStructure);

        adcConfig[ADC_CHANNEL4].adcChannel = ADC4_CHANNEL;
        adcConfig[ADC_CHANNEL4].dmaIndex = adcChannelCount;
        adcConfig[ADC_CHANNEL4].sampleTime = ADC_SampleTime_601Cycles5;
        adcConfig[ADC_CHANNEL4].enabled = true;
        adcChannelCount++;
    }
#endif

#ifdef ADC5_GPIO
    if (init->channelMask & ADC_CHANNEL5_ENABLE) {
        GPIO_InitStructure.GPIO_Pin = ADC5_GPIO_PIN;
        GPIO_Init(ADC5_GPIO, &GPIO_InitStructure);

        adcConfig[ADC_CHANNEL5].adcChannel = ADC5_CHANNEL;
        adcConfig[ADC_CHANNEL5].dmaIndex = adcChannelCount;
        adcConfig[ADC_CHANNEL5].sampleTime = ADC_SampleTime_601Cycles5;
        adcConfig[ADC_CHANNEL5].enabled = true;
        adcChannelCount++;
    }
#endif

    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div256);  // 72 MHz divided by 256 = 281.25 kHz
    RCC_AHBPeriphClockCmd(ADC_AHB_PERIPHERAL | RCC_AHBPeriph_ADC12, ENABLE);

    DMA_DeInit(ADC_DMA_CHANNEL);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC_INSTANCE->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = adcChannelCount;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = adcChannelCount > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(ADC_DMA_CHANNEL, &DMA_InitStructure);

    DMA_Cmd(ADC_DMA_CHANNEL, ENABLE);


    // calibrate

    ADC_VoltageRegulatorCmd(ADC_INSTANCE, ENABLE);
    delay(10);
    ADC_SelectCalibrationMode(ADC_INSTANCE, ADC_CalibrationMode_Single);
    ADC_StartCalibration(ADC_INSTANCE);
    while(ADC_GetCalibrationStatus(ADC_INSTANCE) != RESET);
    ADC_VoltageRegulatorCmd(ADC_INSTANCE, DISABLE);


    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
    ADC_CommonInit(ADC_INSTANCE, &ADC_CommonInitStructure);

    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Enable;
    ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OverrunMode           = ADC_OverrunMode_Disable;
    ADC_InitStructure.ADC_AutoInjMode           = ADC_AutoInjec_Disable;
    ADC_InitStructure.ADC_NbrOfRegChannel       = adcChannelCount;

    ADC_Init(ADC_INSTANCE, &ADC_InitStructure);

    uint8_t rank = 1;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcConfig[i].enabled) {
            continue;
        }
        ADC_RegularChannelConfig(ADC_INSTANCE, adcConfig[i].adcChannel, rank++, adcConfig[i].sampleTime);
    }

    ADC_Cmd(ADC_INSTANCE, ENABLE);

    while(!ADC_GetFlagStatus(ADC_INSTANCE, ADC_FLAG_RDY));

    ADC_DMAConfig(ADC_INSTANCE, ADC_DMAMode_Circular);

    ADC_DMACmd(ADC_INSTANCE, ENABLE);

    ADC_StartConversion(ADC_INSTANCE);
}

#endif
