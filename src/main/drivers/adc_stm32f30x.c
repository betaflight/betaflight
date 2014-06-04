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
#include "system.h"

#include "gpio.h"

#include "sensors/sensors.h" // FIXME dependency into the main code

#include "accgyro.h"

#include "adc.h"

extern adc_config_t adcConfig[ADC_CHANNEL_COUNT];
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

void adcInit(drv_adc_config_t *init)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    uint8_t i;
    uint8_t adcChannelCount = 0;

    memset(&adcConfig, 0, sizeof(adcConfig));

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;

    adcConfig[ADC_BATTERY].adcChannel = ADC_Channel_6;
    adcConfig[ADC_BATTERY].dmaIndex = adcChannelCount;
    adcConfig[ADC_BATTERY].sampleTime = ADC_SampleTime_601Cycles5;
    adcConfig[ADC_BATTERY].enabled = true;
    adcChannelCount++;

    if (init->enableCurrentMeter) {
        GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_1;

        adcConfig[ADC_CURRENT].adcChannel = ADC_Channel_7;
        adcConfig[ADC_CURRENT].dmaIndex = adcChannelCount;
        adcConfig[ADC_CURRENT].sampleTime = ADC_SampleTime_601Cycles5;
        adcConfig[ADC_CURRENT].enabled = true;
        adcChannelCount++;

    }

    if (init->enableRSSI) {
        GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_2;

		adcConfig[ADC_RSSI].adcChannel = ADC_Channel_8;
		adcConfig[ADC_RSSI].dmaIndex = adcChannelCount;
		adcConfig[ADC_RSSI].sampleTime = ADC_SampleTime_601Cycles5;
		adcConfig[ADC_RSSI].enabled = true;
		adcChannelCount++;
    }

    adcConfig[ADC_EXTERNAL1].adcChannel = ADC_Channel_9;
    adcConfig[ADC_EXTERNAL1].dmaIndex = adcChannelCount;
    adcConfig[ADC_EXTERNAL1].sampleTime = ADC_SampleTime_601Cycles5;
    adcConfig[ADC_EXTERNAL1].enabled = true;
    adcChannelCount++;

    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div256);  // 72 MHz divided by 256 = 281.25 kHz
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_ADC12, ENABLE);

    DMA_DeInit(DMA1_Channel1);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
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

    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel1, ENABLE);

    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // calibrate

    ADC_VoltageRegulatorCmd(ADC1, ENABLE);
    delay(10);
    ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) != RESET);
    ADC_VoltageRegulatorCmd(ADC1, DISABLE);


    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
    ADC_CommonInit(ADC1, &ADC_CommonInitStructure);

    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Enable;
    ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OverrunMode           = ADC_OverrunMode_Disable;
    ADC_InitStructure.ADC_AutoInjMode           = ADC_AutoInjec_Disable;
    ADC_InitStructure.ADC_NbrOfRegChannel       = adcChannelCount;

    ADC_Init(ADC1, &ADC_InitStructure);

    uint8_t rank = 1;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcConfig[i].enabled) {
            continue;
        }
        ADC_RegularChannelConfig(ADC1, adcConfig[i].adcChannel, rank++, adcConfig[i].sampleTime);
    }

    ADC_Cmd(ADC1, ENABLE);

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));

    ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);

    ADC_DMACmd(ADC1, ENABLE);

    ADC_StartConversion(ADC1);
}

