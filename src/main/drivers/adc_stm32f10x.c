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

#include "build/build_config.h"

#include "config/parameter_group.h"

#include "system.h"

#include "sensors/sensors.h" // FIXME dependency into the main code

#include "sensor.h"
#include "accgyro.h"

#include "adc.h"
#include "adc_impl.h"

// Driver for STM32F103CB onboard ADC
//
// Naze32
// Battery Voltage (VBAT) is connected to PA4 (ADC1_IN4) with 10k:1k divider
// RSSI ADC uses CH2 (PA1, ADC1_IN1)
// Current ADC uses CH8 (PB1, ADC1_IN9)
//
// NAZE rev.5 hardware has PA5 (ADC1_IN5) on breakout pad on bottom of board
//

#ifdef USE_ADC

void adcInit(drv_adc_config_t *init)
{
#if defined(CJMCU) || defined(CC3D)
    UNUSED(init);
#endif

    uint8_t i;
    uint8_t configuredAdcChannels = 0;

    memset(&adcConfig, 0, sizeof(adcConfig));

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;

#ifdef ADC0_GPIO
    if (init->channelMask & ADC_CHANNEL0_ENABLE) {
        GPIO_InitStructure.GPIO_Pin = ADC0_GPIO_PIN;
        GPIO_Init(ADC0_GPIO, &GPIO_InitStructure);
        adcConfig[ADC_CHANNEL0].adcChannel = ADC0_CHANNEL;
        adcConfig[ADC_CHANNEL0].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_CHANNEL0].enabled = true;
        adcConfig[ADC_CHANNEL0].sampleTime = ADC_SampleTime_239Cycles5;
    }
#endif

#ifdef ADC1_GPIO
    if (init->channelMask & ADC_CHANNEL1_ENABLE) {
        GPIO_InitStructure.GPIO_Pin = ADC1_GPIO_PIN;
        GPIO_Init(ADC1_GPIO, &GPIO_InitStructure);
        adcConfig[ADC_CHANNEL1].adcChannel = ADC1_CHANNEL;
        adcConfig[ADC_CHANNEL1].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_CHANNEL1].enabled = true;
        adcConfig[ADC_CHANNEL1].sampleTime = ADC_SampleTime_239Cycles5;
    }
#endif

#ifdef ADC2_GPIO
    if (init->channelMask & ADC_CHANNEL2_ENABLE) {
        GPIO_InitStructure.GPIO_Pin = ADC2_GPIO_PIN;
        GPIO_Init(ADC2_GPIO, &GPIO_InitStructure);
        adcConfig[ADC_CHANNEL2].adcChannel = ADC2_CHANNEL;
        adcConfig[ADC_CHANNEL2].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_CHANNEL2].enabled = true;
        adcConfig[ADC_CHANNEL2].sampleTime = ADC_SampleTime_239Cycles5;
    }
#endif

#ifdef ADC3_GPIO
    if (init->channelMask & ADC_CHANNEL3_ENABLE) {
        GPIO_InitStructure.GPIO_Pin   = ADC3_GPIO_PIN;
        GPIO_Init(ADC3_GPIO, &GPIO_InitStructure);
        adcConfig[ADC_CHANNEL3].adcChannel = ADC3_CHANNEL;
        adcConfig[ADC_CHANNEL3].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_CHANNEL3].enabled = true;
        adcConfig[ADC_CHANNEL3].sampleTime = ADC_SampleTime_239Cycles5;
    }
#endif

    RCC_ADCCLKConfig(RCC_PCLK2_Div8);  // 9MHz from 72MHz APB2 clock(HSE), 8MHz from 64MHz (HSI)
    RCC_AHBPeriphClockCmd(ADC_AHB_PERIPHERAL, ENABLE);
    RCC_APB2PeriphClockCmd(ADC_ABP2_PERIPHERAL, ENABLE);

    // FIXME ADC driver assumes all the GPIO was already placed in 'AIN' mode

    DMA_DeInit(ADC_DMA_CHANNEL);
    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC_INSTANCE->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = configuredAdcChannels;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = configuredAdcChannels > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(ADC_DMA_CHANNEL, &DMA_InitStructure);
    DMA_Cmd(ADC_DMA_CHANNEL, ENABLE);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = configuredAdcChannels > 1 ? ENABLE : DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = configuredAdcChannels;
    ADC_Init(ADC_INSTANCE, &ADC_InitStructure);

    uint8_t rank = 1;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcConfig[i].enabled) {
            continue;
        }
        ADC_RegularChannelConfig(ADC_INSTANCE, adcConfig[i].adcChannel, rank++, adcConfig[i].sampleTime);
    }

    ADC_DMACmd(ADC_INSTANCE, ENABLE);
    ADC_Cmd(ADC_INSTANCE, ENABLE);

    ADC_ResetCalibration(ADC_INSTANCE);
    while(ADC_GetResetCalibrationStatus(ADC_INSTANCE));
    ADC_StartCalibration(ADC_INSTANCE);
    while(ADC_GetCalibrationStatus(ADC_INSTANCE));

    ADC_SoftwareStartConvCmd(ADC_INSTANCE, ENABLE);
}

#endif
