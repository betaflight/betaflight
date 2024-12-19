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

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/sensor.h"

#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "pg/adc.h"

const adcDevice_t adcHardware[] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_APB2(ADC1),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = DMA_CHANNEL_0
#endif
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_APB2(ADC2),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = DMA_CHANNEL_1
#endif
    },
    {
        .ADCx = ADC3,
        .rccADC = RCC_APB2(ADC3),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC3_DMA_STREAM,
        .channel = DMA_CHANNEL_2
#endif
    }
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

void adcInitDevice(adcDevice_t *adcdev, int channelCount)
{
    adcdev->ADCHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    adcdev->ADCHandle.Init.ContinuousConvMode    = ENABLE;
    adcdev->ADCHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    adcdev->ADCHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    adcdev->ADCHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adcdev->ADCHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    adcdev->ADCHandle.Init.NbrOfConversion       = channelCount;
#ifdef USE_ADC_INTERNAL
    // Multiple injected channel seems to require scan conversion mode to be
    // enabled even if main (non-injected) channel count is 1.
    adcdev->ADCHandle.Init.ScanConvMode          = ENABLE;
#else
    adcdev->ADCHandle.Init.ScanConvMode          = channelCount > 1 ? ENABLE : DISABLE; // 1=scan more that one channel in group
#endif
    adcdev->ADCHandle.Init.DiscontinuousConvMode = DISABLE;
    adcdev->ADCHandle.Init.NbrOfDiscConversion   = 0;
    adcdev->ADCHandle.Init.DMAContinuousRequests = ENABLE;
    adcdev->ADCHandle.Init.EOCSelection          = DISABLE;
    adcdev->ADCHandle.Instance = adcdev->ADCx;

    if (HAL_ADC_Init(&adcdev->ADCHandle) != HAL_OK)
    {
      /* Initialization Error */
    }
}

static adcDevice_t adc;

#ifdef USE_ADC_INTERNAL

static adcDevice_t adcInternal;
static ADC_HandleTypeDef *adcInternalHandle;

void adcInitInternalInjected(adcDevice_t *adcdev)
{
    adcInternalHandle = &adcdev->ADCHandle;

    ADC_InjectionConfTypeDef iConfig;

    iConfig.InjectedChannel      = ADC_CHANNEL_VREFINT;
    iConfig.InjectedRank         = 1;
    iConfig.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
    iConfig.InjectedOffset       = 0;
    iConfig.InjectedNbrOfConversion = 2;
    iConfig.InjectedDiscontinuousConvMode = DISABLE;
    iConfig.AutoInjectedConv     = DISABLE;
    iConfig.ExternalTrigInjecConv = 0;     // Don't care
    iConfig.ExternalTrigInjecConvEdge = 0; // Don't care

    if (HAL_ADCEx_InjectedConfigChannel(adcInternalHandle, &iConfig) != HAL_OK) {
        /* Channel Configuration Error */
    }

    iConfig.InjectedChannel      = ADC_CHANNEL_TEMPSENSOR;
    iConfig.InjectedRank         = 2;

    if (HAL_ADCEx_InjectedConfigChannel(adcInternalHandle, &iConfig) != HAL_OK) {
        /* Channel Configuration Error */
    }

    adcVREFINTCAL = *(uint16_t *)VREFINT_CAL_ADDR;
    adcTSCAL1 = *TEMPSENSOR_CAL1_ADDR;
    adcTSCAL2 = *TEMPSENSOR_CAL2_ADDR;
    adcTSSlopeK = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * 1000 / (adcTSCAL2 - adcTSCAL1);
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
        if (HAL_ADCEx_InjectedPollForConversion(adcInternalHandle, 0) == HAL_OK) {
            adcInternalConversionInProgress = false;
        }
    }

    return adcInternalConversionInProgress;
}

void adcInternalStartConversion(void)
{
    HAL_ADCEx_InjectedStart(adcInternalHandle);

    adcInternalConversionInProgress = true;
}

uint16_t adcInternalReadVrefint(void)
{
    return HAL_ADCEx_InjectedGetValue(adcInternalHandle, ADC_INJECTED_RANK_1);
}

uint16_t adcInternalReadTempsensor(void)
{
    return HAL_ADCEx_InjectedGetValue(adcInternalHandle, ADC_INJECTED_RANK_2);
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

    adc = adcHardware[device];

    bool adcActive = false;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (adcVerifyPin(adcOperatingConfig[i].tag, device)) {
            adcActive = true;
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));
            adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
            adcOperatingConfig[i].dmaIndex = configuredAdcChannels++;
            adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_480CYCLES;
            adcOperatingConfig[i].enabled = true;
        }
    }

#ifndef USE_ADC_INTERNAL
    if (!adcActive) {
        return;
    }
#endif

    RCC_ClockCmd(adc.rccADC, ENABLE);

    adcInitDevice(&adc, configuredAdcChannels);

#ifdef USE_ADC_INTERNAL
    // If device is not ADC1 or there's no active channel, then initialize ADC1  here.
    if (device != ADCDEV_1 || !adcActive) {
        adcInternal = adcHardware[ADCDEV_1];
        RCC_ClockCmd(adcInternal.rccADC, ENABLE);
        adcInitDevice(&adcInternal, 0);
        adcInitInternalInjected(&adcInternal);
    } else {
        adcInitInternalInjected(&adc);
    }
#endif

    uint8_t rank = 1;
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (adcOperatingConfig[i].enabled) {
            ADC_ChannelConfTypeDef sConfig;

            sConfig.Channel      = adcOperatingConfig[i].adcChannel;
            sConfig.Rank         = rank++;
            sConfig.SamplingTime = adcOperatingConfig[i].sampleTime;
            sConfig.Offset       = 0;

            if (HAL_ADC_ConfigChannel(&adc.ADCHandle, &sConfig) != HAL_OK)
            {
                /* Channel Configuration Error */
            }
        }
    }

#ifdef USE_DMA_SPEC
    const dmaChannelSpec_t *dmaspec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, device, config->dmaopt[device]);

    if (!dmaspec || !dmaAllocate(dmaGetIdentifier(dmaspec->ref), OWNER_ADC, 0)) {
        return;
    }

    dmaEnable(dmaGetIdentifier(dmaspec->ref));
    adc.DmaHandle.Init.Channel = dmaspec->channel;
    adc.DmaHandle.Instance = (DMA_ARCH_TYPE *)dmaspec->ref;
#else
    if (!dmaAllocate(dmaGetIdentifier(adc.dmaResource), OWNER_ADC, 0)) {
        return;
    }
    dmaEnable(dmaGetIdentifier(adc.dmaResource));
    adc.DmaHandle.Init.Channel = adc.channel;
    adc.DmaHandle.Instance = (DMA_ARCH_TYPE *)adc.dmaResource;
#endif

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

    if (HAL_DMA_Init(&adc.DmaHandle) != HAL_OK)
    {
        /* Initialization Error */
    }

    __HAL_LINKDMA(&adc.ADCHandle, DMA_Handle, adc.DmaHandle);

    //HAL_CLEANINVALIDATECACHE((uint32_t*)&adcValues, configuredAdcChannels);

    if (HAL_ADC_Start_DMA(&adc.ADCHandle, (uint32_t*)&adcValues, configuredAdcChannels) != HAL_OK)
    {
        /* Start Conversion Error */
    }
}

void adcGetChannelValues(void)
{
    // Nothing to do
}
#endif
