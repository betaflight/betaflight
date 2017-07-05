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

#ifdef USE_ADC

#include "build/build_config.h"

#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "adc.h"
#include "adc_impl.h"
#include "drivers/io.h"
#include "rcc.h"

static adcDevice_t adcHardware[ADCDEV_COUNT] = {
    { .ADCx = ADC1, .rccADC = RCC_APB2(ADC1), .rccDMA = RCC_AHB(DMA1), .DMAy_Channelx = DMA1_Channel1, .enabled = false, .usedChannelCount = 0 }
};

ADCDevice adcDeviceByInstance(ADC_TypeDef *instance)
{
    if (instance == ADC1)
        return ADCDEV_1;

/* TODO -- ADC2 available on large 10x devices.
    if (instance == ADC2)
        return ADCDEV_2;
*/
    return ADCINVALID;
}

const adcTagMap_t adcTagMap[] = {
    { DEFIO_TAG_E__PA0, ADC_Channel_0 }, // ADC12
    { DEFIO_TAG_E__PA1, ADC_Channel_1 }, // ADC12
    { DEFIO_TAG_E__PA2, ADC_Channel_2 }, // ADC12
    { DEFIO_TAG_E__PA3, ADC_Channel_3 }, // ADC12
    { DEFIO_TAG_E__PA4, ADC_Channel_4 }, // ADC12
    { DEFIO_TAG_E__PA5, ADC_Channel_5 }, // ADC12
    { DEFIO_TAG_E__PA6, ADC_Channel_6 }, // ADC12
    { DEFIO_TAG_E__PA7, ADC_Channel_7 }, // ADC12
    { DEFIO_TAG_E__PB0, ADC_Channel_8 }, // ADC12
    { DEFIO_TAG_E__PB1, ADC_Channel_9 }, // ADC12
};

static void adcInstanceInit(ADCDevice adcDevice)
{
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    adcDevice_t * adc = &adcHardware[adcDevice];

    RCC_ClockCmd(adc->rccADC, ENABLE);
    RCC_ClockCmd(adc->rccDMA, ENABLE);

    DMA_DeInit(adc->DMAy_Channelx);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&adc->ADCx->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adcValues[adcDevice];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = adc->usedChannelCount;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = adc->usedChannelCount > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(adc->DMAy_Channelx, &DMA_InitStructure);
    DMA_Cmd(adc->DMAy_Channelx, ENABLE);

    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = adc->usedChannelCount > 1 ? ENABLE : DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = adc->usedChannelCount;
    ADC_Init(adc->ADCx, &ADC_InitStructure);

    uint8_t rank = 1;
    for (int i = ADC_CHN_1; i < ADC_CHN_COUNT; i++) {
        if (!adcConfig[i].enabled || adcConfig[i].adcDevice != adcDevice) {
            continue;
        }

        ADC_RegularChannelConfig(adc->ADCx, adcConfig[i].adcChannel, rank++, adcConfig[i].sampleTime);
    }

    ADC_DMACmd(adc->ADCx, ENABLE);
    ADC_Cmd(adc->ADCx, ENABLE);

    ADC_ResetCalibration(adc->ADCx);
    while (ADC_GetResetCalibrationStatus(adc->ADCx));

    ADC_StartCalibration(adc->ADCx);
    while (ADC_GetCalibrationStatus(adc->ADCx));

    ADC_SoftwareStartConvCmd(adc->ADCx, ENABLE);
}

void adcHardwareInit(drv_adc_config_t *init)
{
    UNUSED(init);
    int configuredAdcChannels = 0;

    for (int i = ADC_CHN_1; i < ADC_CHN_COUNT; i++) {
        if (!adcConfig[i].tag)
            continue;

        adcDevice_t * adc = &adcHardware[adcConfig[i].adcDevice];

        IOInit(IOGetByTag(adcConfig[i].tag), OWNER_ADC, RESOURCE_ADC_CH1 + (i - ADC_CHN_1), 0);
        IOConfigGPIO(IOGetByTag(adcConfig[i].tag), IO_CONFIG(GPIO_Mode_AIN, 0));

        adcConfig[i].adcChannel = adcChannelByTag(adcConfig[i].tag);
        adcConfig[i].dmaIndex = adc->usedChannelCount++;
        adcConfig[i].sampleTime = ADC_SampleTime_239Cycles5;
        adcConfig[i].enabled = true;

        adc->enabled = true;
        configuredAdcChannels++;
    }

    if (configuredAdcChannels == 0)
        return;

    RCC_ADCCLKConfig(RCC_PCLK2_Div8);  // 9MHz from 72MHz APB2 clock(HSE), 8MHz from 64MHz (HSI)

    for (int i = 0; i < ADCDEV_COUNT; i++) {
        if (adcHardware[i].enabled) {
            adcInstanceInit(i);
        }
    }
}
#endif
