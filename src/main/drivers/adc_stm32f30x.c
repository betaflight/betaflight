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

#include "common/utils.h"

#include "drivers/adc_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "pg/adc.h"

#include "adc.h"

const adcDevice_t adcHardware[] = {
    { .ADCx = ADC1, .rccADC = RCC_AHB(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)DMA1_Channel1,
#endif
    },
#ifdef ADC24_DMA_REMAP
    { .ADCx = ADC2, .rccADC = RCC_AHB(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)DMA2_Channel3,
#endif
    },
#else
    { .ADCx = ADC2, .rccADC = RCC_AHB(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)DMA2_Channel1,
#endif
    },
#endif
    { .ADCx = ADC3, .rccADC = RCC_AHB(ADC34),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)DMA2_Channel5,
#endif
    }
};

const adcTagMap_t adcTagMap[] = {
    { DEFIO_TAG_E__PA0,  ADC_DEVICES_1,  ADC_Channel_1  }, // ADC1
    { DEFIO_TAG_E__PA1,  ADC_DEVICES_1,  ADC_Channel_2  }, // ADC1
    { DEFIO_TAG_E__PA2,  ADC_DEVICES_1,  ADC_Channel_3  }, // ADC1
    { DEFIO_TAG_E__PA3,  ADC_DEVICES_1,  ADC_Channel_4  }, // ADC1
    { DEFIO_TAG_E__PA4,  ADC_DEVICES_2,  ADC_Channel_1  }, // ADC2
    { DEFIO_TAG_E__PA5,  ADC_DEVICES_2,  ADC_Channel_2  }, // ADC2
    { DEFIO_TAG_E__PA6,  ADC_DEVICES_2,  ADC_Channel_3  }, // ADC2
    { DEFIO_TAG_E__PA7,  ADC_DEVICES_4,  ADC_Channel_4  }, // ADC2
    { DEFIO_TAG_E__PB0,  ADC_DEVICES_3,  ADC_Channel_12 }, // ADC3
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_3,  ADC_Channel_1  }, // ADC3
    { DEFIO_TAG_E__PB2,  ADC_DEVICES_2,  ADC_Channel_12 }, // ADC2
    { DEFIO_TAG_E__PB12, ADC_DEVICES_4,  ADC_Channel_3  }, // ADC4
    { DEFIO_TAG_E__PB13, ADC_DEVICES_3,  ADC_Channel_5  }, // ADC3
    { DEFIO_TAG_E__PB14, ADC_DEVICES_4,  ADC_Channel_4  }, // ADC4
    { DEFIO_TAG_E__PB15, ADC_DEVICES_4,  ADC_Channel_5  }, // ADC4
    { DEFIO_TAG_E__PC0,  ADC_DEVICES_12, ADC_Channel_6  }, // ADC12
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_12, ADC_Channel_7  }, // ADC12
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_12, ADC_Channel_8  }, // ADC12
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_12, ADC_Channel_9  }, // ADC12
    { DEFIO_TAG_E__PC4,  ADC_DEVICES_2,  ADC_Channel_5  }, // ADC2
    { DEFIO_TAG_E__PC5,  ADC_DEVICES_2,  ADC_Channel_11 }, // ADC2
    { DEFIO_TAG_E__PD8,  ADC_DEVICES_4,  ADC_Channel_12 }, // ADC4
    { DEFIO_TAG_E__PD9,  ADC_DEVICES_4,  ADC_Channel_13 }, // ADC4
    { DEFIO_TAG_E__PD10, ADC_DEVICES_34, ADC_Channel_7  }, // ADC34
    { DEFIO_TAG_E__PD11, ADC_DEVICES_34, ADC_Channel_8  }, // ADC34
    { DEFIO_TAG_E__PD12, ADC_DEVICES_34, ADC_Channel_9  }, // ADC34
    { DEFIO_TAG_E__PD13, ADC_DEVICES_34, ADC_Channel_10 }, // ADC34
    { DEFIO_TAG_E__PD14, ADC_DEVICES_34, ADC_Channel_11 }, // ADC34
    { DEFIO_TAG_E__PE7,  ADC_DEVICES_3,  ADC_Channel_13 }, // ADC3
    { DEFIO_TAG_E__PE8,  ADC_DEVICES_34, ADC_Channel_6  }, // ADC34
    { DEFIO_TAG_E__PE9,  ADC_DEVICES_3,  ADC_Channel_2  }, // ADC3
    { DEFIO_TAG_E__PE10, ADC_DEVICES_3,  ADC_Channel_14 }, // ADC3
    { DEFIO_TAG_E__PE11, ADC_DEVICES_3,  ADC_Channel_15 }, // ADC3
    { DEFIO_TAG_E__PE12, ADC_DEVICES_3,  ADC_Channel_16 }, // ADC3
    { DEFIO_TAG_E__PE13, ADC_DEVICES_3,  ADC_Channel_3  }, // ADC3
    { DEFIO_TAG_E__PE14, ADC_DEVICES_4,  ADC_Channel_1  }, // ADC4
    { DEFIO_TAG_E__PE15, ADC_DEVICES_4,  ADC_Channel_2  }, // ADC4
    { DEFIO_TAG_E__PF2,  ADC_DEVICES_12, ADC_Channel_10 }, // ADC12
    { DEFIO_TAG_E__PF4,  ADC_DEVICES_1,  ADC_Channel_5  }, // ADC1
};

void adcInit(const adcConfig_t *config)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    uint8_t adcChannelCount = 0;

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

#ifdef ADC24_DMA_REMAP
    SYSCFG_DMAChannelRemapConfig(SYSCFG_DMARemap_ADC2ADC4, ENABLE);
#endif
    adcDevice_t adc = adcHardware[device];

    bool adcActive = false;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcVerifyPin(adcOperatingConfig[i].tag, device)) {
            continue;
        }

        adcActive = true;
        IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
        IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));
        adcOperatingConfig[i].adcChannel = adcChannelByTag(adcOperatingConfig[i].tag);
        adcOperatingConfig[i].dmaIndex = adcChannelCount++;
        adcOperatingConfig[i].sampleTime = ADC_SampleTime_601Cycles5;
        adcOperatingConfig[i].enabled = true;
    }

    if (!adcActive) {
        return;
    }

    if ((device == ADCDEV_1) || (device == ADCDEV_2)) {
        // enable clock for ADC1+2
        RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div256);  // 72 MHz divided by 256 = 281.25 kHz
    } else {
        // enable clock for ADC3+4
        RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div256);  // 72 MHz divided by 256 = 281.25 kHz
    }

    RCC_ClockCmd(adc.rccADC, ENABLE);

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, device, config->dmaopt[device]);

    if (!dmaSpec || !dmaAllocate(dmaGetIdentifier(dmaSpec->ref), OWNER_ADC, RESOURCE_INDEX(device))) {
        return;
    }

    dmaEnable(dmaGetIdentifier(dmaSpec->ref));

    DMA_DeInit((DMA_ARCH_TYPE *)dmaSpec->ref);
#else
    if (!dmaAllocate(dmaGetIdentifier(adc.dmaResource), OWNER_ADC, 0)) {
        return;
    }

    dmaEnable(dmaGetIdentifier(adc.dmaResource));

    xDMA_DeInit(adc.dmaResource);
#endif

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&adc.ADCx->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adcValues;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = adcChannelCount;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = adcChannelCount > 1 ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

#if defined(USE_DMA_SPEC)
    xDMA_Init(dmaSpec->ref, &DMA_InitStructure);
    xDMA_Cmd(dmaSpec->ref, ENABLE);
#else
    xDMA_Init(adc.dmaResource, &DMA_InitStructure);
    xDMA_Cmd(adc.dmaResource, ENABLE);
#endif

    // calibrate

    ADC_VoltageRegulatorCmd(adc.ADCx, ENABLE);
    delay(10);
    ADC_SelectCalibrationMode(adc.ADCx, ADC_CalibrationMode_Single);
    ADC_StartCalibration(adc.ADCx);
    while (ADC_GetCalibrationStatus(adc.ADCx) != RESET);
    ADC_VoltageRegulatorCmd(adc.ADCx, DISABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
    ADC_CommonInit(adc.ADCx, &ADC_CommonInitStructure);

    ADC_StructInit(&ADC_InitStructure);

    ADC_InitStructure.ADC_ContinuousConvMode    = ADC_ContinuousConvMode_Enable;
    ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OverrunMode           = ADC_OverrunMode_Disable;
    ADC_InitStructure.ADC_AutoInjMode           = ADC_AutoInjec_Disable;
    ADC_InitStructure.ADC_NbrOfRegChannel       = adcChannelCount;

    ADC_Init(adc.ADCx, &ADC_InitStructure);

    uint8_t rank = 1;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (!adcOperatingConfig[i].enabled) {
            continue;
        }
        ADC_RegularChannelConfig(adc.ADCx, adcOperatingConfig[i].adcChannel, rank++, adcOperatingConfig[i].sampleTime);
    }

    ADC_Cmd(adc.ADCx, ENABLE);

    while (!ADC_GetFlagStatus(adc.ADCx, ADC_FLAG_RDY));

    ADC_DMAConfig(adc.ADCx, ADC_DMAMode_Circular);

    ADC_DMACmd(adc.ADCx, ENABLE);

    ADC_StartConversion(adc.ADCx);
}

void adcGetChannelValues(void)
{
    // Nothing to do
}
#endif
