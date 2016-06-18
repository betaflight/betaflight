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

#include "io.h"

#include "sensors/sensors.h" // FIXME dependency into the main code

#include "sensor.h"
#include "accgyro.h"

#include "adc.h"
#include "adc_impl.h"

void adcInit(drv_adc_config_t *init)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    uint8_t i;
    uint8_t configuredAdcChannels = 0;

    memset(&adcConfig, 0, sizeof(adcConfig));

#if !defined(VBAT_ADC_PIN) && !defined(EXTERNAL1_ADC_PIN) && !defined(RSSI_ADC_PIN) && !defined(CURRENT_METER_ADC_PIN)
    UNUSED(init);
#endif

#ifdef VBAT_ADC_PIN
    if (init->enableVBat) {
        IOInit(IOGetByTag(IO_TAG(VBAT_ADC_PIN)), OWNER_SYSTEM, RESOURCE_ADC);
        IOConfigGPIO(IOGetByTag(IO_TAG(VBAT_ADC_PIN)), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
        adcConfig[ADC_BATTERY].adcChannel = VBAT_ADC_CHANNEL;
        adcConfig[ADC_BATTERY].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_BATTERY].enabled = true;
        adcConfig[ADC_BATTERY].sampleTime = ADC_SampleTime_480Cycles;
    }
#endif

#ifdef EXTERNAL1_ADC_PIN
    if (init->enableExternal1) {
        IOInit(IOGetByTag(IO_TAG(EXTERNAL1_ADC_PIN)), OWNER_SYSTEM, RESOURCE_ADC);
	    IOConfigGPIO(IOGetByTag(IO_TAG(EXTERNAL1_ADC_PIN)), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
        adcConfig[ADC_EXTERNAL1].adcChannel = EXTERNAL1_ADC_CHANNEL;
        adcConfig[ADC_EXTERNAL1].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_EXTERNAL1].enabled = true;
        adcConfig[ADC_EXTERNAL1].sampleTime = ADC_SampleTime_480Cycles;
    }
#endif

#ifdef RSSI_ADC_PIN
    if (init->enableRSSI) {
        IOInit(IOGetByTag(IO_TAG(RSSI_ADC_PIN)), OWNER_SYSTEM, RESOURCE_ADC);
	    IOConfigGPIO(IOGetByTag(IO_TAG(RSSI_ADC_PIN)), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
        adcConfig[ADC_RSSI].adcChannel = RSSI_ADC_CHANNEL;
        adcConfig[ADC_RSSI].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_RSSI].enabled = true;
        adcConfig[ADC_RSSI].sampleTime = ADC_SampleTime_480Cycles;
    }
#endif

#ifdef CURRENT_METER_ADC_PIN
    if (init->enableCurrentMeter) {
        IOInit(IOGetByTag(IO_TAG(CURRENT_METER_ADC_PIN)), OWNER_SYSTEM, RESOURCE_ADC);
	    IOConfigGPIO(IOGetByTag(IO_TAG(CURRENT_METER_ADC_PIN)), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
        adcConfig[ADC_CURRENT].adcChannel = CURRENT_METER_ADC_CHANNEL;
        adcConfig[ADC_CURRENT].dmaIndex = configuredAdcChannels++;
        adcConfig[ADC_CURRENT].enabled = true;
        adcConfig[ADC_CURRENT].sampleTime = ADC_SampleTime_480Cycles;
    }
#endif

    //RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div256);  // 72 MHz divided by 256 = 281.25 kHz
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    DMA_DeInit(DMA2_Stream4);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = configuredAdcChannels;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = configuredAdcChannels > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream4, ENABLE);

    // calibrate

    /*
    ADC_VoltageRegulatorCmd(ADC1, ENABLE);
    delay(10);
    ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) != RESET);
    ADC_VoltageRegulatorCmd(ADC1, DISABLE);
    */

    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_ContinuousConvMode    = ENABLE;
    ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConv 		= ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_ExternalTrigConvEdge 	= ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion       = configuredAdcChannels;
    ADC_InitStructure.ADC_ScanConvMode 			= configuredAdcChannels > 1 ? ENABLE : DISABLE; // 1=scan more that one channel in group

    ADC_Init(ADC1, &ADC_InitStructure);

    uint8_t rank = 1;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcConfig[i].enabled) {
            continue;
        }
        ADC_RegularChannelConfig(ADC1, adcConfig[i].adcChannel, rank++, adcConfig[i].sampleTime);
    }
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_SoftwareStartConv(ADC1);
}
