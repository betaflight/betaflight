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
#include "io_impl.h"
#include "rcc.h"

#include "sensor.h"
#include "accgyro.h"

#include "adc.h"
#include "adc_impl.h"

#ifndef ADC_INSTANCE
#define ADC_INSTANCE                ADC1
#endif

const adcDevice_t adcHardware[] = { 
    { .ADCx = ADC1, .rccADC = RCC_APB2(ADC1), .rccDMA = RCC_AHB1(DMA2), .DMAy_Streamx = DMA2_Stream4, .channel = DMA_CHANNEL_0 },
    //{ .ADCx = ADC2, .rccADC = RCC_APB2(ADC2), .rccDMA = RCC_AHB1(DMA2), .DMAy_Streamx = DMA2_Stream1, .channel = DMA_Channel_0 }  
};

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = { 
/*
    { DEFIO_TAG_E__PF3,  ADC_Channel_9  },
    { DEFIO_TAG_E__PF4,  ADC_Channel_14 },
    { DEFIO_TAG_E__PF5,  ADC_Channel_15 },
    { DEFIO_TAG_E__PF6,  ADC_Channel_4  },
    { DEFIO_TAG_E__PF7,  ADC_Channel_5  },
    { DEFIO_TAG_E__PF8,  ADC_Channel_6  },
    { DEFIO_TAG_E__PF9,  ADC_Channel_7  },
    { DEFIO_TAG_E__PF10, ADC_Channel_8  },
*/
    { DEFIO_TAG_E__PC0, ADC_CHANNEL_10 },
    { DEFIO_TAG_E__PC1, ADC_CHANNEL_11 },
    { DEFIO_TAG_E__PC2, ADC_CHANNEL_12 },
    { DEFIO_TAG_E__PC3, ADC_CHANNEL_13 },
    { DEFIO_TAG_E__PC4, ADC_CHANNEL_14 },
    { DEFIO_TAG_E__PC5, ADC_CHANNEL_15 },
    { DEFIO_TAG_E__PB0, ADC_CHANNEL_8  },
    { DEFIO_TAG_E__PB1, ADC_CHANNEL_9  },
    { DEFIO_TAG_E__PA0, ADC_CHANNEL_0  },
    { DEFIO_TAG_E__PA1, ADC_CHANNEL_1  },
    { DEFIO_TAG_E__PA2, ADC_CHANNEL_2  },
    { DEFIO_TAG_E__PA3, ADC_CHANNEL_3  },
    { DEFIO_TAG_E__PA4, ADC_CHANNEL_4  },
    { DEFIO_TAG_E__PA5, ADC_CHANNEL_5  },
    { DEFIO_TAG_E__PA6, ADC_CHANNEL_6  },
    { DEFIO_TAG_E__PA7, ADC_CHANNEL_7  },
};

ADCDevice adcDeviceByInstance(ADC_TypeDef *instance)
{
    if (instance == ADC1)
        return ADCDEV_1;
/*
    if (instance == ADC2) // TODO add ADC2 and 3
        return ADCDEV_2;
*/
    return ADCINVALID;
}

void adcInit(drv_adc_config_t *init)
{
    DMA_HandleTypeDef DmaHandle;
    ADC_HandleTypeDef ADCHandle;

    uint8_t i;
    uint8_t configuredAdcChannels = 0;

    memset(&adcConfig, 0, sizeof(adcConfig));

#if !defined(VBAT_ADC_PIN) && !defined(EXTERNAL1_ADC_PIN) && !defined(RSSI_ADC_PIN) && !defined(CURRENT_METER_ADC_PIN)
    UNUSED(init);
#endif

#ifdef VBAT_ADC_PIN
    if (init->enableVBat) {
        adcConfig[ADC_BATTERY].tag = IO_TAG(VBAT_ADC_PIN); //VBAT_ADC_CHANNEL;
    }
#endif

#ifdef RSSI_ADC_PIN
    if (init->enableRSSI) {
        adcConfig[ADC_RSSI].tag = IO_TAG(RSSI_ADC_PIN);  //RSSI_ADC_CHANNEL;
    }
#endif

#ifdef EXTERNAL1_ADC_PIN
    if (init->enableExternal1) {
        adcConfig[ADC_EXTERNAL1].tag = IO_TAG(EXTERNAL1_ADC_PIN); //EXTERNAL1_ADC_CHANNEL;
    }
#endif

#ifdef CURRENT_METER_ADC_PIN
    if (init->enableCurrentMeter) {
        adcConfig[ADC_CURRENT].tag = IO_TAG(CURRENT_METER_ADC_PIN);  //CURRENT_METER_ADC_CHANNEL;
    }
#endif

    ADCDevice device = adcDeviceByInstance(ADC_INSTANCE);
    if (device == ADCINVALID)
        return;

    adcDevice_t adc = adcHardware[device];

    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcConfig[i].tag)
            continue;

        IOInit(IOGetByTag(adcConfig[i].tag), OWNER_ADC, RESOURCE_ADC_BATTERY + i, 0);
        IOConfigGPIO(IOGetByTag(adcConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));
        adcConfig[i].adcChannel = adcChannelByTag(adcConfig[i].tag);
        adcConfig[i].dmaIndex = configuredAdcChannels++;
        adcConfig[i].sampleTime = ADC_SAMPLETIME_480CYCLES;
        adcConfig[i].enabled = true;
    }

    RCC_ClockCmd(adc.rccDMA, ENABLE);
    RCC_ClockCmd(adc.rccADC, ENABLE);

    ADCHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    ADCHandle.Init.ContinuousConvMode    = ENABLE;
    ADCHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    ADCHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    ADCHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    ADCHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    ADCHandle.Init.NbrOfConversion       = configuredAdcChannels;
    ADCHandle.Init.ScanConvMode          = configuredAdcChannels > 1 ? ENABLE : DISABLE; // 1=scan more that one channel in group
    ADCHandle.Init.DiscontinuousConvMode = DISABLE;
    ADCHandle.Init.NbrOfDiscConversion   = 0;
    ADCHandle.Init.DMAContinuousRequests = ENABLE;
    ADCHandle.Init.EOCSelection          = DISABLE;
    ADCHandle.Instance = adc.ADCx;

    /*##-1- Configure the ADC peripheral #######################################*/
    if (HAL_ADC_Init(&ADCHandle) != HAL_OK)
    {
      /* Initialization Error */
    }

    DmaHandle.Init.Channel = adc.channel;
    DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    DmaHandle.Init.MemInc = configuredAdcChannels > 1 ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DmaHandle.Init.Mode = DMA_CIRCULAR;
    DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
    DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
    DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
    DmaHandle.Instance = adc.DMAy_Streamx;

    /*##-2- Initialize the DMA stream ##########################################*/
    if (HAL_DMA_Init(&DmaHandle) != HAL_OK)
    {
        /* Initialization Error */
    }

    __HAL_LINKDMA(&ADCHandle, DMA_Handle, DmaHandle);

    uint8_t rank = 1;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcConfig[i].enabled) {
            continue;
        }
        ADC_ChannelConfTypeDef sConfig;
        sConfig.Channel      = adcConfig[i].adcChannel;
        sConfig.Rank         = rank++;
        sConfig.SamplingTime = adcConfig[i].sampleTime;
        sConfig.Offset       = 0;

        /*##-3- Configure ADC regular channel ######################################*/
        if (HAL_ADC_ConfigChannel(&ADCHandle, &sConfig) != HAL_OK)
        {
          /* Channel Configuration Error */
        }
    }

    /*##-4- Start the conversion process #######################################*/
    if(HAL_ADC_Start_DMA(&ADCHandle, (uint32_t*)&adcValues, configuredAdcChannels) != HAL_OK)
    {
        /* Start Conversation Error */
    }
}
