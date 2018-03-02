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

#include "drivers/accgyro/accgyro.h"
#include "drivers/system.h"

#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"
#include "dma.h"

#include "drivers/sensor.h"

#include "adc.h"
#include "adc_impl.h"

#include "pg/adc.h"


#ifndef ADC_INSTANCE
#define ADC_INSTANCE                ADC1
#endif

const adcDevice_t adcHardware[] = {
    { .ADCx = ADC1, .rccADC = RCC_APB2(ADC1), .DMAy_Streamx = ADC1_DMA_STREAM, .channel = DMA_CHANNEL_0 },
    { .ADCx = ADC2, .rccADC = RCC_APB2(ADC2), .DMAy_Streamx = ADC2_DMA_STREAM, .channel = DMA_CHANNEL_1 },
    { .ADCx = ADC3, .rccADC = RCC_APB2(ADC3), .DMAy_Streamx = ADC3_DMA_STREAM, .channel = DMA_CHANNEL_2 }
};

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
/*
    { DEFIO_TAG_E__PF3, ADC_DEVICES_3,   ADC_CHANNEL_9  },
    { DEFIO_TAG_E__PF4, ADC_DEVICES_3,   ADC_CHANNEL_14 },
    { DEFIO_TAG_E__PF5, ADC_DEVICES_3,   ADC_CHANNEL_15 },
    { DEFIO_TAG_E__PF6, ADC_DEVICES_3,   ADC_CHANNEL_4  },
    { DEFIO_TAG_E__PF7, ADC_DEVICES_3,   ADC_CHANNEL_5  },
    { DEFIO_TAG_E__PF8, ADC_DEVICES_3,   ADC_CHANNEL_6  },
    { DEFIO_TAG_E__PF9, ADC_DEVICES_3,   ADC_CHANNEL_7  },
    { DEFIO_TAG_E__PF10,ADC_DEVICES_3,   ADC_CHANNEL_8  },
*/
    { DEFIO_TAG_E__PC0, ADC_DEVICES_123, ADC_CHANNEL_10 },
    { DEFIO_TAG_E__PC1, ADC_DEVICES_123, ADC_CHANNEL_11 },
    { DEFIO_TAG_E__PC2, ADC_DEVICES_123, ADC_CHANNEL_12 },
    { DEFIO_TAG_E__PC3, ADC_DEVICES_123, ADC_CHANNEL_13 },
    { DEFIO_TAG_E__PC4, ADC_DEVICES_12,  ADC_CHANNEL_14 },
    { DEFIO_TAG_E__PC5, ADC_DEVICES_12,  ADC_CHANNEL_15 },
    { DEFIO_TAG_E__PB0, ADC_DEVICES_12,  ADC_CHANNEL_8  },
    { DEFIO_TAG_E__PB1, ADC_DEVICES_12,  ADC_CHANNEL_9  },
    { DEFIO_TAG_E__PA0, ADC_DEVICES_123, ADC_CHANNEL_0  },
    { DEFIO_TAG_E__PA1, ADC_DEVICES_123, ADC_CHANNEL_1  },
    { DEFIO_TAG_E__PA2, ADC_DEVICES_123, ADC_CHANNEL_2  },
    { DEFIO_TAG_E__PA3, ADC_DEVICES_123, ADC_CHANNEL_3  },
    { DEFIO_TAG_E__PA4, ADC_DEVICES_12,  ADC_CHANNEL_4  },
    { DEFIO_TAG_E__PA5, ADC_DEVICES_12,  ADC_CHANNEL_5  },
    { DEFIO_TAG_E__PA6, ADC_DEVICES_12,  ADC_CHANNEL_6  },
    { DEFIO_TAG_E__PA7, ADC_DEVICES_12,  ADC_CHANNEL_7  },
};

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

    ADCDevice device = adcDeviceByInstance(ADC_INSTANCE);
    if (device == ADCINVALID)
        return;

    adcDevice_t adc = adcHardware[device];

    bool adcActive = false;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcVerifyPin(adcOperatingConfig[i].tag, device)) {
            continue;
        }

        adcActive = true;
        IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
        IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));
        adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
        adcOperatingConfig[i].dmaIndex = configuredAdcChannels++;
        adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_480CYCLES;
        adcOperatingConfig[i].enabled = true;
    }

    if (!adcActive) {
        return;
    }

    RCC_ClockCmd(adc.rccADC, ENABLE);
    dmaInit(dmaGetIdentifier(adc.DMAy_Streamx), OWNER_ADC, 0);

    adc.ADCHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    adc.ADCHandle.Init.ContinuousConvMode    = ENABLE;
    adc.ADCHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    adc.ADCHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    adc.ADCHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc.ADCHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    adc.ADCHandle.Init.NbrOfConversion       = configuredAdcChannels;
    adc.ADCHandle.Init.ScanConvMode          = configuredAdcChannels > 1 ? ENABLE : DISABLE; // 1=scan more that one channel in group
    adc.ADCHandle.Init.DiscontinuousConvMode = DISABLE;
    adc.ADCHandle.Init.NbrOfDiscConversion   = 0;
    adc.ADCHandle.Init.DMAContinuousRequests = ENABLE;
    adc.ADCHandle.Init.EOCSelection          = DISABLE;
    adc.ADCHandle.Instance = adc.ADCx;

    /*##-1- Configure the ADC peripheral #######################################*/
    if (HAL_ADC_Init(&adc.ADCHandle) != HAL_OK)
    {
      /* Initialization Error */
    }

    adc.DmaHandle.Init.Channel = adc.channel;
    adc.DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    adc.DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    adc.DmaHandle.Init.MemInc = configuredAdcChannels > 1 ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    adc.DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    adc.DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    adc.DmaHandle.Init.Mode = DMA_CIRCULAR;
    adc.DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
    adc.DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    adc.DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    adc.DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
    adc.DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
    adc.DmaHandle.Instance = adc.DMAy_Streamx;

    /*##-2- Initialize the DMA stream ##########################################*/
    if (HAL_DMA_Init(&adc.DmaHandle) != HAL_OK)
    {
        /* Initialization Error */
    }

    __HAL_LINKDMA(&adc.ADCHandle, DMA_Handle, adc.DmaHandle);

    uint8_t rank = 1;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcOperatingConfig[i].enabled) {
            continue;
        }
        ADC_ChannelConfTypeDef sConfig;
        sConfig.Channel      = adcOperatingConfig[i].adcChannel;
        sConfig.Rank         = rank++;
        sConfig.SamplingTime = adcOperatingConfig[i].sampleTime;
        sConfig.Offset       = 0;

        /*##-3- Configure ADC regular channel ######################################*/
        if (HAL_ADC_ConfigChannel(&adc.ADCHandle, &sConfig) != HAL_OK)
        {
          /* Channel Configuration Error */
        }
    }

    //HAL_CLEANINVALIDATECACHE((uint32_t*)&adcValues, configuredAdcChannels);
    /*##-4- Start the conversion process #######################################*/
    if (HAL_ADC_Start_DMA(&adc.ADCHandle, (uint32_t*)&adcValues, configuredAdcChannels) != HAL_OK)
    {
        /* Start Conversation Error */
    }
}
#endif
