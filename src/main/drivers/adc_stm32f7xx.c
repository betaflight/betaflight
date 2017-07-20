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
#include "drivers/time.h"

#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"
#include "dma.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"

#include "adc.h"
#include "adc_impl.h"

static adcDevice_t adcHardware[ADCDEV_COUNT] = {
    { .ADCx = ADC1, .rccADC = RCC_APB2(ADC1), .rccDMA = RCC_AHB1(DMA2), .DMAy_Streamx = DMA2_Stream4, .channel = DMA_CHANNEL_0, .enabled = false, .usedChannelCount = 0 },
    //{ .ADCx = ADC2, .rccADC = RCC_APB2(ADC2), .rccDMA = RCC_AHB1(DMA2), .DMAy_Streamx = DMA2_Stream1, .channel = DMA_Channel_0, .enabled = false, .usedChannelCount = 0 }
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

static void adcInstanceInit(ADCDevice adcDevice)
{
    adcDevice_t * adc = &adcHardware[adcDevice];

    RCC_ClockCmd(adc->rccDMA, ENABLE);
    RCC_ClockCmd(adc->rccADC, ENABLE);

    adc->ADCHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    adc->ADCHandle.Init.ContinuousConvMode    = ENABLE;
    adc->ADCHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    adc->ADCHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    adc->ADCHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc->ADCHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    adc->ADCHandle.Init.NbrOfConversion       = adc->usedChannelCount;
    adc->ADCHandle.Init.ScanConvMode          = adc->usedChannelCount > 1 ? ENABLE : DISABLE; // 1=scan more that one channel in group
    adc->ADCHandle.Init.DiscontinuousConvMode = DISABLE;
    adc->ADCHandle.Init.NbrOfDiscConversion   = 0;
    adc->ADCHandle.Init.DMAContinuousRequests = ENABLE;
    adc->ADCHandle.Init.EOCSelection          = DISABLE;
    adc->ADCHandle.Instance = adc->ADCx;

    /*##-1- Configure the ADC peripheral #######################################*/
    if (HAL_ADC_Init(&adc->ADCHandle) != HAL_OK)
    {
        /* Initialization Error */
        return;
    }

    adc->DmaHandle.Init.Channel = adc->channel;
    adc->DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    adc->DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    adc->DmaHandle.Init.MemInc = adc->usedChannelCount > 1 ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    adc->DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    adc->DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    adc->DmaHandle.Init.Mode = DMA_CIRCULAR;
    adc->DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
    adc->DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    adc->DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    adc->DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
    adc->DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
    adc->DmaHandle.Instance = adc->DMAy_Streamx;

    /*##-2- Initialize the DMA stream ##########################################*/
    if (HAL_DMA_Init(&adc->DmaHandle) != HAL_OK)
    {
        /* Initialization Error */
        return;
    }

    __HAL_LINKDMA(&adc->ADCHandle, DMA_Handle, adc->DmaHandle);

    uint8_t rank = 1;
    for (int i = ADC_CHN_1; i < ADC_CHN_COUNT; i++) {
        if (!adcConfig[i].enabled || adcConfig[i].adcDevice != adcDevice) {
            continue;
        }

        ADC_ChannelConfTypeDef sConfig;
        sConfig.Channel      = adcConfig[i].adcChannel;
        sConfig.Rank         = rank++;
        sConfig.SamplingTime = adcConfig[i].sampleTime;
        sConfig.Offset       = 0;

        /*##-3- Configure ADC regular channel ######################################*/
        if (HAL_ADC_ConfigChannel(&adc->ADCHandle, &sConfig) != HAL_OK)
        {
            /* Channel Configuration Error */
            return;
        }
    }

    //HAL_CLEANINVALIDATECACHE((uint32_t*)&adcValues[adcDevice], configuredAdcChannels);
    /*##-4- Start the conversion process #######################################*/
    if (HAL_ADC_Start_DMA(&adc->ADCHandle, (uint32_t*)&adcValues[adcDevice], adc->usedChannelCount) != HAL_OK)
    {
        /* Start Conversation Error */
    }
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
        IOConfigGPIO(IOGetByTag(adcConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));

        adcConfig[i].adcChannel = adcChannelByTag(adcConfig[i].tag);
        adcConfig[i].dmaIndex = adc->usedChannelCount++;
        adcConfig[i].sampleTime = ADC_SAMPLETIME_480CYCLES;
        adcConfig[i].enabled = true;

        adc->enabled = true;
        configuredAdcChannels++;
    }

    if (configuredAdcChannels == 0)
        return;

    for (int i = 0; i < ADCDEV_COUNT; i++) {
        if (adcHardware[i].enabled) {
            adcInstanceInit(i);
        }
    }
}
