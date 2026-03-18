/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/rcc.h"
#include "drivers/resource.h"
#include "drivers/dma.h"

#include "drivers/sensor.h"

#include "drivers/adc.h"
#include "platform/adc_impl.h"

#include "pg/adc.h"

#ifndef ADC1_DMA_STREAM
#define ADC1_DMA_STREAM NULL
#endif
#ifndef ADC2_DMA_STREAM
#define ADC2_DMA_STREAM NULL
#endif

const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_AHB1(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = DMA_REQUEST_ADC1,
#endif
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_AHB1(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = DMA_REQUEST_ADC2,
#endif
    },
};

adcDevice_t adcDevice[ADCDEV_COUNT];

// N6 has internal channels on ADC2
#define ADC_DEVICE_FOR_INTERNAL ADC_DEVICES_2

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
#ifdef USE_ADC_INTERNAL
    // Pseudo entries for internal sensor.
    // Keep these at the beginning for easy indexing by ADC_TAG_MAP_{VREFINT,VBAT4}
#define ADC_TAG_MAP_VREFINT    0
#define ADC_TAG_MAP_TEMPSENSOR 1
#define ADC_TAG_MAP_VBAT4      2

    // N6 has VREFINT on ADC2 channel 19, VBAT/4 on ADC2 channel 17
    // N6 does not have an internal temperature sensor ADC channel
    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL,   ADC_CHANNEL_VREFINT,    19 }, // 19 VREFINT
    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL,   ADC_CHANNEL_VREFINT,    19 }, // placeholder for TEMPSENSOR (not available on N6)
    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL,   ADC_CHANNEL_VBAT,       17 }, // 17 VBAT/4

#endif // USE_ADC_INTERNAL

    // STM32N6 ADC pin mappings (ADC1 and ADC2)
    { DEFIO_TAG_E__PA0,  ADC_DEVICES_12,  ADC_CHANNEL_0,   0 },
    { DEFIO_TAG_E__PA1,  ADC_DEVICES_12,  ADC_CHANNEL_1,   1 },
    { DEFIO_TAG_E__PA2,  ADC_DEVICES_12,  ADC_CHANNEL_2,   2 },
    { DEFIO_TAG_E__PA3,  ADC_DEVICES_12,  ADC_CHANNEL_3,   3 },
    { DEFIO_TAG_E__PA4,  ADC_DEVICES_12,  ADC_CHANNEL_4,   4 },
    { DEFIO_TAG_E__PA5,  ADC_DEVICES_12,  ADC_CHANNEL_5,   5 },
    { DEFIO_TAG_E__PA6,  ADC_DEVICES_12,  ADC_CHANNEL_6,   6 },
    { DEFIO_TAG_E__PA7,  ADC_DEVICES_12,  ADC_CHANNEL_7,   7 },
    { DEFIO_TAG_E__PB0,  ADC_DEVICES_12,  ADC_CHANNEL_8,   8 },
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_12,  ADC_CHANNEL_9,   9 },
    { DEFIO_TAG_E__PC0,  ADC_DEVICES_12,  ADC_CHANNEL_10, 10 },
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_12,  ADC_CHANNEL_11, 11 },
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_12,  ADC_CHANNEL_12, 12 },
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_12,  ADC_CHANNEL_13, 13 },
    { DEFIO_TAG_E__PC4,  ADC_DEVICES_12,  ADC_CHANNEL_14, 14 },
    { DEFIO_TAG_E__PC5,  ADC_DEVICES_12,  ADC_CHANNEL_15, 15 },
};

// Translate rank number x to ADC_REGULAR_RANK_x (Note that array index is 0-origin)

#define RANK(n) ADC_REGULAR_RANK_ ## n

static uint32_t adcRegularRankMap[] = {
    RANK(1),
    RANK(2),
    RANK(3),
    RANK(4),
    RANK(5),
    RANK(6),
    RANK(7),
    RANK(8),
    RANK(9),
    RANK(10),
    RANK(11),
    RANK(12),
    RANK(13),
    RANK(14),
    RANK(15),
    RANK(16),
};

#undef RANK

static void errorHandler(void) { while (1) { } }

// Note on sampling time.
// N6 ADC sample times: 1.5, 2.5, 6.5, 11.5, 23.5, 46.5, 246.5, 1499.5 cycles

static void adcInitDevice(adcDevice_t *adcdev, int channelCount)
{
    ADC_HandleTypeDef *hadc = &adcdev->ADCHandle; // For clarity

    hadc->Instance = adcdev->ADCx;

    // DeInit is done in adcInit().

    // N6 ADC clock prescaler is configured via RCC, not in ADC_InitTypeDef
    hadc->Init.Resolution               = ADC_RESOLUTION_12B;
    hadc->Init.GainCompensation         = 0;                             // No gain compensation
    hadc->Init.ScanConvMode             = ENABLE;                        // Works with single channel, too
    hadc->Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait         = DISABLE;
    hadc->Init.ContinuousConvMode       = ENABLE;
    hadc->Init.NbrOfConversion          = channelCount;
    hadc->Init.DiscontinuousConvMode    = DISABLE;
    hadc->Init.NbrOfDiscConversion      = 1;                             // Don't care
    hadc->Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc->Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE; // Don't care
    hadc->Init.SamplingMode             = ADC_SAMPLING_MODE_NORMAL;
    hadc->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    hadc->Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    hadc->Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    hadc->Init.OversamplingMode         = DISABLE;

    // Initialize this ADC peripheral

    if (HAL_ADC_Init(hadc) != HAL_OK) {
      errorHandler();
    }

    // Execute calibration

    if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK) {
      errorHandler();
    }
}

static int adcFindTagMapEntry(ioTag_t tag)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].tag == tag) {
            return i;
        }
    }
    return -1;
}

#ifdef USE_ADC_INTERNAL
// N6 does not have TEMPSENSOR calibration addresses or VREFINT_CAL_ADDR in the same way.
// Provide stub calibration values for now.
static void adcInitCalibrationValues(void)
{
    // N6 does not provide factory calibration data for VREFINT / TEMPSENSOR
    // Use nominal values
    adcVREFINTCAL = 0;
    adcTSCAL1 = 0;
    adcTSCAL2 = 0;
    adcTSSlopeK = 0;
}
#endif

// ADC conversion result DMA buffer
// Need this separate from the main adcValue[] array, because channels are numbered
// by ADC instance order that is different from ADC_xxx numbering.

#define ADC_BUF_LENGTH ADC_SOURCE_COUNT
#define ADC_BUF_BYTES (ADC_BUF_LENGTH * sizeof(uint16_t))
#define ADC_BUF_CACHE_ALIGN_BYTES  ((ADC_BUF_BYTES + 0x20) & ~0x1f)
#define ADC_BUF_CACHE_ALIGN_LENGTH (ADC_BUF_CACHE_ALIGN_BYTES / sizeof(uint16_t))

static volatile DMA_RAM uint16_t adcConversionBuffer[ADC_BUF_CACHE_ALIGN_LENGTH] __attribute__((aligned(32)));

void adcInit(const adcConfig_t *config)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memcpy(adcDevice, adcHardware, sizeof(adcDevice));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
        adcOperatingConfig[ADC_BATTERY].adcDevice = config->vbat.device;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;
        adcOperatingConfig[ADC_RSSI].adcDevice = config->rssi.device;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag;
        adcOperatingConfig[ADC_EXTERNAL1].adcDevice = config->external1.device;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;
        adcOperatingConfig[ADC_CURRENT].adcDevice = config->current.device;
    }

#ifdef USE_ADC_INTERNAL
    adcInitCalibrationValues();
#endif

    for (unsigned i = 0; i < ADC_SOURCE_COUNT; i++) {
        int map = -1;
        int dev = -1;

#ifdef USE_ADC_INTERNAL
        if (i >= ADC_EXTERNAL_COUNT) {
            switch(i) {
            case ADC_TEMPSENSOR:
                map = ADC_TAG_MAP_TEMPSENSOR;
                break;
            case ADC_VREFINT:
                map = ADC_TAG_MAP_VREFINT;
                break;
#if ADC_INTERNAL_VBAT4_ENABLED
            case ADC_VBAT4:
                map = ADC_TAG_MAP_VBAT4;
                break;
#endif
            default:
                // Unknown internal source; skip to avoid using an uninitialized map
                continue;
            }
            dev = ffs(adcTagMap[map].devices) - 1;
            if (dev < 0) { continue; }
            adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_1499CYCLES_5;
        }
#endif
        if (i < ADC_EXTERNAL_COUNT) {
            dev = ADC_CFG_TO_DEV(adcOperatingConfig[i].adcDevice);

            if (dev < 0 || !adcOperatingConfig[i].tag) {
                continue;
            }

            map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
            if (map < 0) {
                continue;
            }

            adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_246CYCLES_5;

            // Found a tag map entry for this input pin
            // Find an ADC device that can handle this input pin

            bool useConfiguredDevice = (dev != ADCINVALID) && (adcTagMap[map].devices & (1 << dev));

            if (!useConfiguredDevice) {
                // If the ADC was configured to use a specific device, but that device was not active, then try and find another active instance that works for the pin.

                for (dev = 0; dev < ADCDEV_COUNT; dev++) {
                    if (!adcDevice[dev].ADCx) {
                        // Instance not activated
                        continue;
                    }

#ifdef USE_DMA_SPEC
                    // check that there is a valid spec for this dev
                    const dmaChannelSpec_t *spec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
                    if (!spec) {
                        continue;
                    }
#else
                    if (!adcDevice[dev].dmaResource) {
                        continue;
                    }
#endif

                    if (adcTagMap[map].devices & (1 << dev)) {
                        // Found an activated ADC instance for this input pin
                        break;
                    }
                }

                if (dev == ADCDEV_COUNT) {
                    // No valid device found, go next channel.
                    continue;
                }
            }
        }

        // At this point, map is an entry for the input pin and dev is a valid ADCx for the pin for input i

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].enabled = true;

        adcDevice[dev].channelBits |= (1 << adcTagMap[map].channelOrdinal);

        // Configure a pin for ADC
        if (adcOperatingConfig[i].tag) {
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));
        }
    }

    // DeInit ADCx with inputs
    // We have to batch call DeInit() for all devices as DeInit() initializes ADCx_COMMON register.

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        adc->ADCHandle.Instance = adc->ADCx;

        if (HAL_ADC_DeInit(&adc->ADCHandle) != HAL_OK) {
            // ADC de-initialization Error
            errorHandler();
        }
    }

    // Configure ADCx with inputs

    int dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!adc->channelBits) {
            continue;
        }

        RCC_ClockCmd(adc->rccADC, ENABLE);

        int configuredAdcChannels = popcount(adc->channelBits);

        adcInitDevice(adc, configuredAdcChannels);

        // Configure channels

        int rank = 0;

        for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {

            if (!adcOperatingConfig[adcChan].enabled) {
                continue;
            }

            if (adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }

            adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;

            ADC_ChannelConfTypeDef sConfig;

            sConfig.Channel      = adcOperatingConfig[adcChan].adcChannel;  /* Sampled channel number */
            sConfig.Rank         = adcRegularRankMap[rank++];               /* Rank of sampled channel number ADCx_CHANNEL */
            sConfig.SamplingTime = adcOperatingConfig[adcChan].sampleTime;  /* Sampling time (number of clock cycles unit) */
            sConfig.SingleDiff   = ADC_SINGLE_ENDED;                        /* Single-ended input channel */
            sConfig.OffsetNumber = ADC_OFFSET_NONE;                         /* No offset subtraction */
            sConfig.Offset = 0;                                             /* Parameter discarded because offset correction is disabled */

            if (HAL_ADC_ConfigChannel(&adc->ADCHandle, &sConfig) != HAL_OK) {
                errorHandler();
            }
        }

        // Configure DMA for this ADC peripheral

#ifdef USE_DMA_SPEC
        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
        dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);

        if (!dmaSpec || !dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
            return;
        }

        adc->DmaHandle.Instance                 = (DMA_Channel_TypeDef *)dmaSpec->ref;
        adc->DmaHandle.Init.Request             = dmaSpec->channel;
#else
        dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(adc->dmaResource);

        if (!dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
            return;
        }

        adc->DmaHandle.Instance                 = (DMA_ARCH_TYPE *)adc->dmaResource;
        adc->DmaHandle.Init.Request             = adc->channel;
#endif
        // N6 uses GPDMA with different init structure members
        adc->DmaHandle.Init.BlkHWRequest        = DMA_BREQ_SINGLE_BURST;
        adc->DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        adc->DmaHandle.Init.SrcInc              = DMA_SINC_FIXED;
        adc->DmaHandle.Init.DestInc             = DMA_DINC_INCREMENTED;
        adc->DmaHandle.Init.SrcDataWidth        = DMA_SRC_DATAWIDTH_HALFWORD;
        adc->DmaHandle.Init.DestDataWidth       = DMA_DEST_DATAWIDTH_HALFWORD;
        adc->DmaHandle.Init.Priority            = DMA_LOW_PRIORITY_LOW_WEIGHT;
        adc->DmaHandle.Init.SrcBurstLength      = 1;
        adc->DmaHandle.Init.DestBurstLength     = 1;
        adc->DmaHandle.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
        adc->DmaHandle.Init.TransferEventMode   = DMA_TCEM_BLOCK_TRANSFER;
        adc->DmaHandle.Init.Mode                = DMA_NORMAL;

        // Deinitialize  & Initialize the DMA for new transfer

        // dmaEnable must be called before calling HAL_DMA_Init,
        // to enable clock for associated DMA if not already done so.
        dmaEnable(dmaIdentifier);

        HAL_DMA_DeInit(&adc->DmaHandle);
        HAL_DMA_Init(&adc->DmaHandle);

        // Associate the DMA handle

        __HAL_LINKDMA(&adc->ADCHandle, DMA_Handle, adc->DmaHandle);
    }

    // Start channels.
    // This must be done after channel configuration is complete, as HAL_ADC_ConfigChannel
    // throws an error when configuring internal channels if ADC1 or ADC2 are already enabled.

    dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {

        adcDevice_t *adc = &adcDevice[dev];

        if (!adc->channelBits) {
            continue;
        }

        // Start conversion in DMA mode

        if (HAL_ADC_Start_DMA(&adc->ADCHandle, (uint32_t *)&adcConversionBuffer[dmaBufferIndex], popcount(adc->channelBits)) != HAL_OK) {
            errorHandler();
        }

        dmaBufferIndex += popcount(adc->channelBits);
    }
}

void adcGetChannelValues(void)
{
    // Transfer values in conversion buffer into adcValues[]
    // N6 Cortex-M55 does not require SCB_InvalidateDCache_by_Addr like H7's Cortex-M7
    for (unsigned i = 0; i < ADC_EXTERNAL_COUNT; i++) {
        if (adcOperatingConfig[i].enabled) {
            adcValues[adcOperatingConfig[i].dmaIndex] = adcConversionBuffer[adcOperatingConfig[i].dmaIndex];
        }
    }
}

#ifdef USE_ADC_INTERNAL

bool adcInternalIsBusy(void)
{
    return false;
}

void adcInternalStartConversion(void)
{
    return;
}

uint16_t adcInternalRead(adcSource_e source)
{
    switch (source) {
    case ADC_VREFINT:
    case ADC_TEMPSENSOR:
#if ADC_INTERNAL_VBAT4_ENABLED
    case ADC_VBAT4:
#endif
        const unsigned dmaIndex = adcOperatingConfig[source].dmaIndex;
        return dmaIndex < ADC_BUF_LENGTH ? adcConversionBuffer[dmaIndex] : 0;
    default:
        return 0;
    }
}
#endif // USE_ADC_INTERNAL

#endif // USE_ADC
