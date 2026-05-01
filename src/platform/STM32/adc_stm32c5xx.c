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

/*
 * ADC driver for STM32C5 (HAL2 / Cube 2.0).
 *
 * Uses LL API for ADC configuration and direct LPDMA register writes for
 * circular DMA.  HAL2's integrated ADC+DMA API requires linked-list DMA
 * infrastructure that adds unnecessary complexity; the LL approach is
 * consistent with the other C5 HAL2 driver forks (timer_hal2, bus_spi_hal2).
 *
 * C5 ADC: 14 external channels (IN0–IN13), VREFINT and TEMPSENSOR on ADC1.
 * No VBAT/4 internal channel.
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

/* HAL2: LL ADC header not auto-included through stm32c5xx_hal.h */
#include "stm32c5xx_ll_adc.h"

/* ---- ADC_CHANNEL_x aliases (HAL2 LL naming) ---- */

#define ADC_CHANNEL_0          LL_ADC_CHANNEL_0
#define ADC_CHANNEL_1          LL_ADC_CHANNEL_1
#define ADC_CHANNEL_2          LL_ADC_CHANNEL_2
#define ADC_CHANNEL_3          LL_ADC_CHANNEL_3
#define ADC_CHANNEL_4          LL_ADC_CHANNEL_4
#define ADC_CHANNEL_5          LL_ADC_CHANNEL_5
#define ADC_CHANNEL_6          LL_ADC_CHANNEL_6
#define ADC_CHANNEL_7          LL_ADC_CHANNEL_7
#define ADC_CHANNEL_8          LL_ADC_CHANNEL_8
#define ADC_CHANNEL_9          LL_ADC_CHANNEL_9
#define ADC_CHANNEL_10         LL_ADC_CHANNEL_10
#define ADC_CHANNEL_11         LL_ADC_CHANNEL_11
#define ADC_CHANNEL_12         LL_ADC_CHANNEL_12
#define ADC_CHANNEL_13         LL_ADC_CHANNEL_13
#define ADC_CHANNEL_VREFINT    LL_ADC_CHANNEL_VREFINT
#define ADC_CHANNEL_TEMPSENSOR LL_ADC_CHANNEL_TEMPSENSOR

/* ---- Hardware descriptors ---- */

#ifndef ADC1_DMA_STREAM
#define ADC1_DMA_STREAM NULL
#endif
#ifndef ADC2_DMA_STREAM
#define ADC2_DMA_STREAM NULL
#endif

const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_AHB2(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = DMA_REQUEST_ADC1,
#endif
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_AHB2(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = DMA_REQUEST_ADC2,
#endif
    },
};

adcDevice_t adcDevice[ADCDEV_COUNT];

/* C5: VREFINT and TEMPSENSOR are on ADC1 only.  No VBAT/4 channel. */
#define ADC_DEVICE_FOR_INTERNAL ADC_DEVICES_1

const adcTagMap_t adcTagMap[] = {
#ifdef USE_ADC_INTERNAL
#define ADC_TAG_MAP_VREFINT    0
#define ADC_TAG_MAP_TEMPSENSOR 1
#define ADC_TAG_MAP_VBAT4      2

    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL, ADC_CHANNEL_VREFINT,    13 },
    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL, ADC_CHANNEL_TEMPSENSOR, 12 },
    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL, ADC_CHANNEL_VREFINT,    13 }, // placeholder (no VBAT/4)
#endif

    /* STM32C5 ADC pin mappings: channels 0–13 on ADC1 and ADC2 */
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
};

/* ---- Sequencer rank map (LL constants) ---- */

static const uint32_t adcRegularRankMap[] = {
    LL_ADC_REG_RANK_1,  LL_ADC_REG_RANK_2,  LL_ADC_REG_RANK_3,  LL_ADC_REG_RANK_4,
    LL_ADC_REG_RANK_5,  LL_ADC_REG_RANK_6,  LL_ADC_REG_RANK_7,  LL_ADC_REG_RANK_8,
    LL_ADC_REG_RANK_9,  LL_ADC_REG_RANK_10, LL_ADC_REG_RANK_11, LL_ADC_REG_RANK_12,
    LL_ADC_REG_RANK_13, LL_ADC_REG_RANK_14, LL_ADC_REG_RANK_15, LL_ADC_REG_RANK_16,
};

/* ---- Sequencer length map (index = channelCount - 1) ---- */

static const uint32_t adcSeqLenMap[] = {
    LL_ADC_REG_SEQ_SCAN_DISABLE,
    LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_7RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_9RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_10RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_11RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_12RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_13RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_14RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_15RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_16RANKS,
};

/* ---- LPDMA circular linked-list node ---- */

typedef struct {
    uint32_t cbr1;  /* block transfer length */
    uint32_t cdar;  /* destination address (buffer start) */
    uint32_t cllr;  /* self-pointing link register */
} __attribute__((aligned(4))) adcDmaLLNode_t;

static adcDmaLLNode_t adcDmaNode[ADCDEV_COUNT];

/* ---- DMA conversion buffer ---- */

#define ADC_BUF_LENGTH ADC_SOURCE_COUNT
#define ADC_BUF_BYTES (ADC_BUF_LENGTH * sizeof(uint16_t))
#define ADC_BUF_CACHE_ALIGN_BYTES  ((ADC_BUF_BYTES + 0x20) & ~0x1f)
#define ADC_BUF_CACHE_ALIGN_LENGTH (ADC_BUF_CACHE_ALIGN_BYTES / sizeof(uint16_t))

static volatile DMA_DATA_ZERO_INIT uint16_t adcConversionBuffer[ADC_BUF_CACHE_ALIGN_LENGTH] __attribute__((aligned(32)));

/* ---- Helpers ---- */

static int adcFindTagMapEntry(ioTag_t tag)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].tag == tag) {
            return i;
        }
    }
    return -1;
}

/* ---- ADC LL init ---- */

static void adcInitDevice(ADC_TypeDef *ADCx, int channelCount)
{
    /* Exit deep power-down */
    LL_ADC_DisableDeepPowerDown(ADCx);

    /* Enable internal voltage regulator and wait for startup (~20 µs) */
    LL_ADC_EnableInternalRegulator(ADCx);
    {
        volatile uint32_t wait = SystemCoreClock / 50000;
        while (wait--) {}
    }

    /* Resolution, trigger, continuous mode, overrun, DMA unlimited */
    LL_ADC_SetResolution(ADCx, LL_ADC_RESOLUTION_12B);
    LL_ADC_REG_SetTriggerSource(ADCx, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode(ADCx, LL_ADC_REG_CONV_CONTINUOUS);
    LL_ADC_REG_SetOverrun(ADCx, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
    LL_ADC_REG_SetDataTransferMode(ADCx, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

    /* Sequencer length */
    if (channelCount >= 1 && channelCount <= 16) {
        LL_ADC_REG_SetSequencerLength(ADCx, adcSeqLenMap[channelCount - 1]);
    }

    /* Calibrate (single-ended) */
    LL_ADC_StartCalibration(ADCx, LL_ADC_IN_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADCx)) {}

    /* Enable ADC */
    LL_ADC_ClearFlag_ADRDY(ADCx);
    LL_ADC_Enable(ADCx);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADCx)) {}
}

/* Configure LPDMA channel for circular ADC transfer.
 * Sets up a self-linking linked-list node so the DMA automatically
 * restarts after each sequence conversion, providing continuous data. */
static void adcConfigDmaCircular(DMA_Channel_TypeDef *ch, uint32_t request,
                                 uint32_t srcAddr, volatile uint16_t *destBuf,
                                 uint32_t lengthBytes, adcDmaLLNode_t *node)
{
    /* Populate the linked-list node that reloads CBR1, CDAR, and CLLR */
    uint32_t nodeAddr = (uint32_t)node;
    uint32_t cllrVal = (nodeAddr & DMA_CLLR_LA)
                     | DMA_CLLR_UB1       /* reload block length */
                     | DMA_CLLR_UDA       /* reload dest address */
                     | DMA_CLLR_ULL;      /* reload link register */

    node->cbr1 = lengthBytes & DMA_CBR1_BNDT;
    node->cdar = (uint32_t)destBuf;
    node->cllr = cllrVal;

    /* Set linked-list base address (upper 16 bits of node address) */
    ch->CLBAR = nodeAddr & DMA_CLBAR_LBA;

    /* CTR1: halfword source (ADC DR) fixed, halfword dest incremented */
    ch->CTR1 = DMA_CTR1_DINC
             | (1U << DMA_CTR1_SDW_LOG2_Pos)   /* source halfword */
             | (1U << DMA_CTR1_DDW_LOG2_Pos);   /* dest halfword */

    /* CTR2: peripheral-to-memory, DMA request selection */
    ch->CTR2 = (request & DMA_CTR2_REQSEL);

    /* CBR1: block transfer length in bytes */
    ch->CBR1 = lengthBytes & DMA_CBR1_BNDT;

    /* Source = ADC data register, Dest = conversion buffer */
    ch->CSAR = srcAddr;
    ch->CDAR = (uint32_t)destBuf;

    /* Link to circular node */
    ch->CLLR = cllrVal;

    /* Enable the channel */
    ch->CCR |= DMA_CCR_EN;
}

#ifdef USE_ADC_INTERNAL
static void adcInitCalibrationValues(void)
{
    adcVREFINTCAL = *LL_ADC_VREFINT_CAL_ADDR;
    adcTSCAL1 = *LL_ADC_TEMPSENSOR_CAL1_ADDR;
    adcTSCAL2 = *LL_ADC_TEMPSENSOR_CAL2_ADDR;
    adcTSSlopeK = (LL_ADC_TEMPSENSOR_CAL2_TEMP - LL_ADC_TEMPSENSOR_CAL1_TEMP) * 1000
                / (adcTSCAL2 - adcTSCAL1);
}
#endif

/* ---- Main ADC init ---- */

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
            switch (i) {
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
                continue;
            }
            dev = ffs(adcTagMap[map].devices) - 1;
            if (dev < 0) { continue; }
            adcOperatingConfig[i].sampleTime = LL_ADC_SAMPLINGTIME_289CYCLES;
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

            adcOperatingConfig[i].sampleTime = LL_ADC_SAMPLINGTIME_139CYCLES;

            bool useConfiguredDevice = (dev != ADCINVALID) && (adcTagMap[map].devices & (1 << dev));

            if (!useConfiguredDevice) {
                for (dev = 0; dev < ADCDEV_COUNT; dev++) {
                    if (!adcDevice[dev].ADCx) {
                        continue;
                    }

#ifdef USE_DMA_SPEC
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
                        break;
                    }
                }

                if (dev == ADCDEV_COUNT) {
                    continue;
                }
            }
        }

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].enabled = true;

        adcDevice[dev].channelBits |= (1 << adcTagMap[map].channelOrdinal);

        if (adcOperatingConfig[i].tag) {
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));
        }
    }

#ifdef USE_ADC_INTERNAL
    /* Enable internal measurement paths on ADC_COMMON */
    ADC_Common_TypeDef *adcCommon = ADC12_COMMON;
    uint32_t intPaths = 0;
    for (unsigned i = ADC_EXTERNAL_COUNT; i < ADC_SOURCE_COUNT; i++) {
        if (!adcOperatingConfig[i].enabled) continue;
        uint32_t ch = adcOperatingConfig[i].adcChannel;
        if (ch == LL_ADC_CHANNEL_VREFINT)    intPaths |= LL_ADC_PATH_INTERNAL_VREFINT;
        if (ch == LL_ADC_CHANNEL_TEMPSENSOR) intPaths |= LL_ADC_PATH_INTERNAL_TEMPSENSOR;
    }
    if (intPaths) {
        LL_ADC_SetCommonPathInternalChAdd(adcCommon, intPaths);
        /* Stabilisation delay for TEMPSENSOR (~120 µs) */
        volatile uint32_t wait = SystemCoreClock / 8000;
        while (wait--) {}
    }
#endif

    /* Configure each active ADC device */

    int dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!adc->channelBits) {
            continue;
        }

        RCC_ClockCmd(adc->rccADC, ENABLE);

        int configuredAdcChannels = popcount(adc->channelBits);

        adcInitDevice(adc->ADCx, configuredAdcChannels);

        /* Configure sequencer ranks and sampling times */
        int rank = 0;

        for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {
            if (!adcOperatingConfig[adcChan].enabled) {
                continue;
            }
            if (adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }

            adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;

            LL_ADC_REG_SetSequencerRanks(adc->ADCx,
                                         adcRegularRankMap[rank],
                                         adcOperatingConfig[adcChan].adcChannel);
            LL_ADC_SetChannelSamplingTime(adc->ADCx,
                                          adcOperatingConfig[adcChan].adcChannel,
                                          adcOperatingConfig[adcChan].sampleTime);
            LL_ADC_SetChannelSingleDiff(adc->ADCx,
                                        adcOperatingConfig[adcChan].adcChannel,
                                        LL_ADC_IN_SINGLE_ENDED);
            rank++;
        }

        /* Configure DMA */

#ifdef USE_DMA_SPEC
        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
        if (!dmaSpec) {
            continue;
        }
        dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);

        if (!dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
            continue;
        }

        dmaEnable(dmaIdentifier);

        DMA_Channel_TypeDef *dmaCh = (DMA_Channel_TypeDef *)dmaSpec->ref;
        uint32_t dmaRequest = dmaSpec->channel;
#else
        dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(adc->dmaResource);

        if (!dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
            continue;
        }

        dmaEnable(dmaIdentifier);

        DMA_Channel_TypeDef *dmaCh = (DMA_Channel_TypeDef *)adc->dmaResource;
        uint32_t dmaRequest = adc->channel;
#endif

        uint32_t adcDrAddr = LL_ADC_DMA_GetRegAddr(adc->ADCx, LL_ADC_DMA_REG_REGULAR_DATA);
        uint32_t xferBytes = configuredAdcChannels * sizeof(uint16_t);

        adcConfigDmaCircular(dmaCh, dmaRequest, adcDrAddr,
                             &adcConversionBuffer[dmaBufferIndex - configuredAdcChannels],
                             xferBytes, &adcDmaNode[dev]);

        /* Start continuous conversion */
        LL_ADC_REG_StartConversion(adc->ADCx);
    }
}

void adcGetChannelValues(void)
{
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
        ;
        const unsigned dmaIndex = adcOperatingConfig[source].dmaIndex;
        return dmaIndex < ADC_BUF_LENGTH ? adcConversionBuffer[dmaIndex] : 0;
    default:
        return 0;
    }
}
#endif // USE_ADC_INTERNAL

#endif // USE_ADC
