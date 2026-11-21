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
#include <math.h>

#include "platform.h"

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/rcc.h"
#include "drivers/dma.h"
#include "platform/dma.h"
#include "drivers/sensor.h"
#include "drivers/adc.h"
#include "platform/adc_impl.h"

#include "pg/adc.h"

const adcDevice_t adcHardware[] = {
    {
        .ADCx = (ADC_TypeDef *)ADC0,
        .rccADC = RCC_APB2(ADC0),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC0_DMA_STREAM,
        .channel = DMA_SUBPERI0
#endif
    },
    {
        .ADCx = (ADC_TypeDef *)ADC1,
        .rccADC = RCC_APB2(ADC1),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = DMA_SUBPERI1
#endif
    },
    {
        .ADCx = (ADC_TypeDef *)ADC2,
        .rccADC = RCC_APB2(ADC2),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = DMA_SUBPERI2
#endif
    }
};

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {

    { DEFIO_TAG_E__PC0, ADC_DEVICES_012, ADC_CHANNEL_10 }, // PC0,H7-ADC012_IN10
    { DEFIO_TAG_E__PC1, ADC_DEVICES_012, ADC_CHANNEL_11 }, // PC1,H7-ADC012_IN11
    { DEFIO_TAG_E__PC2, ADC_DEVICES_2,   ADC_CHANNEL_0  }, // PC2,H7-ADC2_IN0
    { DEFIO_TAG_E__PC3, ADC_DEVICES_2,   ADC_CHANNEL_1  }, // PC3,H7-ADC2_IN1
    { DEFIO_TAG_E__PC4, ADC_DEVICES_01,  ADC_CHANNEL_4  }, // PC4,H7-ADC01_IN4
    { DEFIO_TAG_E__PC5, ADC_DEVICES_01,  ADC_CHANNEL_8  }, // PC5,H7-ADC01_IN8
    { DEFIO_TAG_E__PB0, ADC_DEVICES_01,  ADC_CHANNEL_9  }, // PB0,H7-ADC01_IN9
    { DEFIO_TAG_E__PB1, ADC_DEVICES_01,  ADC_CHANNEL_5  }, // PB1,H7-ADC01_IN5
    { DEFIO_TAG_E__PA0, ADC_DEVICES_0,   ADC_CHANNEL_16 }, // PA0,H7-ADC0_IN16
    { DEFIO_TAG_E__PA1, ADC_DEVICES_0,   ADC_CHANNEL_17 }, // PA1,H7-ADC0_IN17
    { DEFIO_TAG_E__PA2, ADC_DEVICES_01,  ADC_CHANNEL_14 }, // PA2,H7-ADC01_IN14
    { DEFIO_TAG_E__PA3, ADC_DEVICES_01,  ADC_CHANNEL_15 }, // PA3,H7-ADC01_IN15
    { DEFIO_TAG_E__PA4, ADC_DEVICES_01,  ADC_CHANNEL_18 }, // PA4,H7-ADC01_IN18
    { DEFIO_TAG_E__PA5, ADC_DEVICES_01,  ADC_CHANNEL_19 }, // PA5,H7-ADC01_IN19
    { DEFIO_TAG_E__PA6, ADC_DEVICES_01,  ADC_CHANNEL_3  }, // PA6,H7-ADC01_IN3
    { DEFIO_TAG_E__PA7, ADC_DEVICES_01,  ADC_CHANNEL_7  }, // PA6,H7-ADC01_IN7
};

static void adcInitDevice(uint32_t adc_periph, int channelCount)
{
    adc_special_function_config(adc_periph, ADC_SCAN_MODE, channelCount > 1 ? ENABLE : DISABLE);

    adc_special_function_config(adc_periph, ADC_CONTINUOUS_MODE, ENABLE);
    adc_resolution_config(adc_periph, ADC_RESOLUTION_12B);
    adc_data_alignment_config(adc_periph, ADC_DATAALIGN_RIGHT);

    /* routine channel config */
    adc_external_trigger_config(adc_periph, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_channel_length_config(adc_periph, ADC_ROUTINE_CHANNEL, channelCount);

    if(PERIPH_INT(ADC0) == adc_periph)
        trigsel_init(TRIGSEL_OUTPUT_ADC0_REGTRG, TRIGSEL_INPUT_TIMER0_CH0);
    else if(PERIPH_INT(ADC1) == adc_periph)
        trigsel_init(TRIGSEL_OUTPUT_ADC1_REGTRG, TRIGSEL_INPUT_TIMER0_CH0);
    else // if(PERIPH_INT(ADC2) == adc_periph)
        trigsel_init(TRIGSEL_OUTPUT_ADC2_REGTRG, TRIGSEL_INPUT_TIMER0_CH0);

    adc_external_trigger_config(adc_periph, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_channel_length_config(adc_periph, ADC_ROUTINE_CHANNEL, channelCount);
}

// ADC conversion result DMA buffer. Cache-aligned and placed in DMA_RAM to avoid
// D-cache coherency issues on GD32H7 (DMA writes memory, CPU reads via cache).
#define ADC_BUF_LENGTH ADC_SOURCE_COUNT
#define ADC_BUF_BYTES (ADC_BUF_LENGTH * sizeof(uint16_t))
#define ADC_BUF_CACHE_ALIGN_BYTES  ((ADC_BUF_BYTES + 0x1f) & ~0x1f)
#define ADC_BUF_CACHE_ALIGN_LENGTH (ADC_BUF_CACHE_ALIGN_BYTES / sizeof(uint16_t))

static volatile DMA_RAM uint16_t adcConversionBuffer[ADC_BUF_CACHE_ALIGN_LENGTH] __attribute__((aligned(32)));

#ifdef USE_ADC_INTERNAL

// Internal sensors live on ADC2 (TEMPSENSOR = IN18, VREFINT = CH19) and are sampled
// by ADC2's routine group. This only enables the sensors and loads the calibration values;
// sequence/DMA setup is done by the per-device loop in adcInit().
static void adcInitInternal(const adcConfig_t *config)
{
    RCC_ClockCmd(adcHardware[ADCDEV_2].rccADC, ENABLE);

    adc_internal_channel_config(ADC_CHANNEL_INTERNAL_TEMPSENSOR, ENABLE);
    adc_internal_channel_config(ADC_CHANNEL_INTERNAL_VREFINT, ENABLE);

    adcVREFINTCAL = config->vrefIntCalibration ? config->vrefIntCalibration : VREFINT_EXPECTED;
    adcTSCAL1 = config->tempSensorCalibration1 ? config->tempSensorCalibration1 : (*(uint16_t *)TEMPSENSOR_CAL1_ADDR & 0x0FFF);
    adcTSCAL2 = config->tempSensorCalibration2 ? config->tempSensorCalibration2 : (*(uint16_t *)TEMPSENSOR_CAL2_ADDR & 0x0FFF);
    adcTSSlopeK = ((TEMPSENSOR_CAL1_TEMP - TEMPSENSOR_CAL2_TEMP) * 1000) / (adcTSCAL1 - adcTSCAL2);
}

// Note on sampling time for temperature sensor and vrefint:
// Both sources have minimum sample time of 10us.
// With prescaler = 8:
// 168MHz : fAPB2 = 84MHz, fADC = 10.5MHz, tcycle = 0.090us, 10us = 105cycle < 144cycle
// 240MHz : fAPB2 = 120MHz, fADC = 15.0MHz, tcycle = 0.067usk 10us = 150cycle < 480cycle
//
// 480cycles@15.0MHz = 32us

// Internal sensors are converted by the free-running routine group DMA, so there is
// no conversion to start or wait for.
bool adcInternalIsBusy(void)
{
    return false;
}

void adcInternalStartConversion(void)
{
    return;
}

static uint16_t adcInternalReadBuffered(int channel)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)adcConversionBuffer, ADC_BUF_CACHE_ALIGN_BYTES);
    return adcConversionBuffer[adcOperatingConfig[channel].dmaIndex];
}

uint16_t adcInternalRead(adcSource_e source)
{
    switch (source) {
    case ADC_VREFINT:
    case ADC_TEMPSENSOR:
        return adcInternalReadBuffered(source);
    default:
        return 0;
    }
}
#endif // USE_ADC_INTERNAL

// Channel collection state per ADC device (ADC0/1/2). channelBits[n] set when
// ADCx has at least one input mapped to physical channel n.
static uint32_t adcDeviceChannelBits[ADCDEV_COUNT];

static int adcFindTagMapEntry(ioTag_t tag)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].tag == tag) {
            return i;
        }
    }
    return -1;
}

void adcInit(const adcConfig_t *config)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memset(adcDeviceChannelBits, 0, sizeof(adcDeviceChannelBits));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;           //RSSI_ADC_CHANNEL;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag; //EXTERNAL1_ADC_CHANNEL;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;    //CURRENT_METER_ADC_CHANNEL;
    }

    // Assign each external input to an ADC instance: prefer ADC_INSTANCE / `adc_device`
    // when it can service the pin, otherwise fall back to any instance whose channel map
    // covers it. This is what routes ADC2-only pins (PC2/PC3) to ADC2.
    for (int i = 0; i < ADC_EXTERNAL_COUNT; i++) {
        int map = -1;
        int dev = -1;

        if (!adcOperatingConfig[i].tag) {
            continue;
        }

        map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
        if (map < 0) {
            continue;
        }

        dev = ADC_CFG_TO_DEV(config->device);

        bool useConfiguredDevice = (dev != ADCINVALID) && (adcTagMap[map].devices & (1 << dev));

        if (!useConfiguredDevice) {
            for (dev = 0; dev < ADCDEV_COUNT; dev++) {
                if (!adcHardware[dev].ADCx) {
                    continue;
                }
                if (adcTagMap[map].devices & (1 << dev)) {
                    break;
                }
            }

            if (dev == ADCDEV_COUNT) {
                // No ADC instance covers this pin.
                continue;
            }
        }

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].sampleTime = 638;
        adcOperatingConfig[i].enabled = true;

        adcDeviceChannelBits[dev] |= (1 << adcTagMap[map].channel);

        IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
        IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_OTYPE_OD, GPIO_PUPD_NONE));
    }

#ifdef USE_ADC_INTERNAL
    // Internal sensors are fixed to ADC2 routine channels 18/19.
    adcOperatingConfig[ADC_TEMPSENSOR].adcDevice  = ADCDEV_2;
    adcOperatingConfig[ADC_TEMPSENSOR].adcChannel = ADC_CHANNEL_18;
    adcOperatingConfig[ADC_TEMPSENSOR].sampleTime = 638;
    adcOperatingConfig[ADC_TEMPSENSOR].enabled    = true;

    adcOperatingConfig[ADC_VREFINT].adcDevice  = ADCDEV_2;
    adcOperatingConfig[ADC_VREFINT].adcChannel = ADC_CHANNEL_19;
    adcOperatingConfig[ADC_VREFINT].sampleTime = 638;
    adcOperatingConfig[ADC_VREFINT].enabled    = true;

    adcDeviceChannelBits[ADCDEV_2] |= (1 << ADC_CHANNEL_18) | (1 << ADC_CHANNEL_19);
#endif

    bool adcActive = false;
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        if (adcDeviceChannelBits[dev]) {
            adcActive = true;
            break;
        }
    }

    if (!adcActive) {
        return;
    }

    // Configure ADC COMMON once using the (first) active ADC for clocking.
    RCC_ClockCmd(adcHardware[0].rccADC, ENABLE);
    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    adc_sync_dma_config(ADC_SYNC_DMA_DISABLE);
    adc_sync_delay_config(ADC_SYNC_DELAY_5CYCLE);

#ifdef USE_ADC_INTERNAL
    adcInitInternal(config);
#endif

    // Per-ADC configuration & DMA allocation. Each ADC that has at least one channel
    // assigned is initialized separately and gets its own DMA channel via
    // config->dmaopt[dev].
    int dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        if (!adcDeviceChannelBits[dev]) {
            continue;
        }

        const adcDevice_t *adc = &adcHardware[dev];
        uint32_t adc_periph = PERIPH_INT(adc->ADCx);

        RCC_ClockCmd(adc->rccADC, ENABLE);
        adc_clock_config(adc_periph, ADC_CLK_SYNC_HCLK_DIV6);

        int configuredAdcChannels = 0;
        for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {
            if (adcOperatingConfig[adcChan].enabled && adcOperatingConfig[adcChan].adcDevice == dev) {
                adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;
                configuredAdcChannels++;
            }
        }

        adcInitDevice(adc_periph, configuredAdcChannels);

        uint8_t rank = 0;
        for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {
            if (!adcOperatingConfig[adcChan].enabled || adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }
            adc_routine_channel_config(adc_periph, rank++,
                adcOperatingConfig[adcChan].adcChannel, adcOperatingConfig[adcChan].sampleTime);
        }

        adc_dma_request_after_last_enable(adc_periph);
        adc_dma_mode_enable(adc_periph);
        adc_enable(adc_periph);

        // Offset calibration, per GD32H7 SPL examples: enable, wait for stability, calibrate.
        delayMicroseconds(1000);
        adc_calibration_mode_config(adc_periph, ADC_CALIBRATION_OFFSET);
        adc_calibration_number(adc_periph, ADC_CALIBRATION_NUM1);
        adc_calibration_enable(adc_periph);

#ifdef USE_DMA_SPEC
        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
        if (!dmaSpec || !dmaAllocate(dmaGetIdentifier(dmaSpec->ref), OWNER_ADC, RESOURCE_INDEX(dev))) {
            return;
        }
        dmaEnable(dmaGetIdentifier(dmaSpec->ref));
        xDMA_DeInit(dmaSpec->ref);
#else
        if (!dmaAllocate(dmaGetIdentifier(adc->dmaResource), OWNER_ADC, RESOURCE_INDEX(dev))) {
            return;
        }
        dmaEnable(dmaGetIdentifier(adc->dmaResource));
        xDMA_DeInit(adc->dmaResource);
#endif

        DMA_InitTypeDef dma_init_struct;
        dma_single_data_para_struct_init(&dma_init_struct.config.init_struct_s);
        dma_init_struct.config.init_struct_s.periph_addr = (uint32_t)(&ADC_RDATA(adc_periph));

#ifdef USE_DMA_SPEC
        dma_init_struct.config.init_struct_s.request = dmaSpec->channel;
#else
        dma_init_struct.config.init_struct_s.request = adc->channel;
#endif

        dma_init_struct.config.init_struct_s.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        dma_init_struct.config.init_struct_s.memory0_addr = (uint32_t)&adcConversionBuffer[dmaBufferIndex - configuredAdcChannels];
        dma_init_struct.config.init_struct_s.memory_inc = configuredAdcChannels > 1 ? DMA_MEMORY_INCREASE_ENABLE : DMA_MEMORY_INCREASE_DISABLE;
        dma_init_struct.config.init_struct_s.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
        dma_init_struct.config.init_struct_s.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
        dma_init_struct.config.init_struct_s.direction = DMA_PERIPH_TO_MEMORY;
        dma_init_struct.config.init_struct_s.number = configuredAdcChannels;
        dma_init_struct.config.init_struct_s.priority = DMA_PRIORITY_HIGH;

#ifdef USE_DMA_SPEC
        gd32_dma_general_init((uint32_t)dmaSpec->ref, &dma_init_struct);
        xDMA_Cmd(dmaSpec->ref, ENABLE);
#else
        gd32_dma_general_init((uint32_t)adc->dmaResource, &dma_init_struct);
        xDMA_Cmd(adc->dmaResource, ENABLE);
#endif

        adc_software_trigger_enable(adc_periph, ADC_ROUTINE_CHANNEL);
    }
}

void adcGetChannelValues(void)
{
    // DMA wrote into adcConversionBuffer (DMA_RAM). Invalidate the cache line
    // region so the CPU observes DMA-written data, then copy to adcValues.
    SCB_InvalidateDCache_by_Addr((uint32_t *)adcConversionBuffer, ADC_BUF_CACHE_ALIGN_BYTES);

    for (int i = 0; i < ADC_EXTERNAL_COUNT; i++) {
        if (adcOperatingConfig[i].enabled) {
            adcValues[adcOperatingConfig[i].dmaIndex] = adcConversionBuffer[adcOperatingConfig[i].dmaIndex];
        }
    }
}
#endif
