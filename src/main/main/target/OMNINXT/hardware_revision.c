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
#include <stdlib.h>

#include "platform.h"

#ifdef USE_HARDWARE_REVISION_DETECTION

#include "build/debug.h"

#include "drivers/adc_impl.h"
#include "drivers/io_types.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/time.h"

#include "hardware_revision.h"

#undef DEBUG_HARDWARE_REVISION_ADC
#undef DEBUG_HARDWARE_REVISION_TABLE

uint8_t hardwareRevision = 0;

// Do ADC on IDDetectPin and determine revision
// If VREFINT is used, we can (probably) get a pretty good precision
// that we can distinguish tens of different voltages.

#define ADC_ID_DETECT_PIN PC2

typedef struct idDetect_s {
    uint32_t ratio;
    uint8_t revision;
} idDetect_t;

// To deploy the analog ID detection in production:
// - Need some theoretical evaluation and experimentation to determine
//   IDDET_ERROR value (ADC with VREFINT compensation is quite accurate).
// - Do some planning on revision numbering scheme.
// - Divider value planning for the scheme (separation).

#define IDDET_RATIO(highside, lowside) ((lowside) * 1000 / ((lowside) + (highside)))
#define IDDET_ERROR 12

static idDetect_t idDetectTable[] = {
#ifdef OMNINXT7
    { IDDET_RATIO(10000, 10000), 1 },
#endif
#ifdef OMNINXT4
    { IDDET_RATIO(10000, 10000), 1 },
#endif
};

ioTag_t idDetectTag;

#if defined(OMNINXT4)

#define VREFINT_CAL_ADDR  0x1FFF7A2A

static void adcIDDetectInit(void)
{
    idDetectTag = IO_TAG(ADC_ID_DETECT_PIN);
    IOConfigGPIO(IOGetByTag(idDetectTag), IO_CONFIG(GPIO_Mode_AN, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL));

    RCC_ClockCmd(RCC_APB2(ADC1), ENABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_ContinuousConvMode       = ENABLE;
    ADC_InitStructure.ADC_Resolution               = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConv         = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_ExternalTrigConvEdge     = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign                = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion          = 2; // Not used
    ADC_InitStructure.ADC_ScanConvMode             = ENABLE;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    ADC_TempSensorVrefintCmd(ENABLE);
    delayMicroseconds(10); // Maximum startup time for internal sensors (DM00037051 5.3.22 & 24)

    uint32_t channel = adcChannelByTag(idDetectTag);

    ADC_InjectedDiscModeCmd(ADC1, DISABLE);
    ADC_InjectedSequencerLengthConfig(ADC1, 2);
    ADC_InjectedChannelConfig(ADC1, channel, 1, ADC_SampleTime_480Cycles);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_Vrefint, 2, ADC_SampleTime_480Cycles);
}

static void adcIDDetectDeinit(void)
{
    ADC_Cmd(ADC1, DISABLE);
    ADC_DeInit();
    IOConfigGPIO(IOGetByTag(idDetectTag), IOCFG_IPU);
}

static void adcIDDetectStart(void)
{
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_SoftwareStartInjectedConv(ADC1);
}

static void adcIDDetectWait(void)
{
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET) {
        // Empty
    }
}

static uint16_t adcIDDetectReadIDDet(void)
{
    return ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
}

static uint16_t adcIDDetectReadVrefint(void)
{
    return ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
}
#endif

#if defined(OMNINXT7)
#define VREFINT_CAL_ADDR  0x1FF07A2A

#include "drivers/adc_impl.h"

static adcDevice_t adcIDDetHardware = 
    {
        .ADCx = ADC1,
        .rccADC = RCC_APB2(ADC1),
#if !defined(USE_DMA_SPEC)
        .DMAy_Streamx = ADC1_DMA_STREAM,
        .channel = DMA_CHANNEL_0
#endif
    };

// XXX adcIDDetectInitDevice is an exact copy of adcInitDevice() from adc_stm32f7xx.c. Export and use?

static void adcIDDetectInitDevice(adcDevice_t *adcdev, int channelCount)
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

static void adcIDDetectInit(void)
{
    idDetectTag = IO_TAG(ADC_ID_DETECT_PIN);

    IOConfigGPIO(IOGetByTag(idDetectTag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));

    adcIDDetectInitDevice(&adcIDDetHardware, 2);

    ADC_InjectionConfTypeDef iConfig;

    iConfig.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
    iConfig.InjectedOffset       = 0;
    iConfig.InjectedNbrOfConversion = 2;
    iConfig.InjectedDiscontinuousConvMode = DISABLE;
    iConfig.AutoInjectedConv     = DISABLE;
    iConfig.ExternalTrigInjecConv = 0;     // Don't care
    iConfig.ExternalTrigInjecConvEdge = 0; // Don't care

    iConfig.InjectedChannel      = ADC_CHANNEL_VREFINT;
    iConfig.InjectedRank         = 1;

    if (HAL_ADCEx_InjectedConfigChannel(&adcIDDetHardware.ADCHandle, &iConfig) != HAL_OK) {
        /* Channel Configuration Error */
    }

    iConfig.InjectedChannel      = adcChannelByTag(idDetectTag);
    iConfig.InjectedRank         = 2;

    if (HAL_ADCEx_InjectedConfigChannel(&adcIDDetHardware.ADCHandle, &iConfig) != HAL_OK) {
        /* Channel Configuration Error */
    }
}

static void adcIDDetectDeinit(void)
{
    HAL_ADC_DeInit(&adcIDDetHardware.ADCHandle);
    IOConfigGPIO(IOGetByTag(idDetectTag), IOCFG_IPU);
}

static void adcIDDetectStart(void)
{
    HAL_ADCEx_InjectedStart(&adcIDDetHardware.ADCHandle);
}

static void adcIDDetectWait(void)
{
    while (HAL_ADCEx_InjectedPollForConversion(&adcIDDetHardware.ADCHandle, 0) != HAL_OK) {
        // Empty
    }
}

static uint16_t adcIDDetectReadVrefint(void)
{
    return HAL_ADCEx_InjectedGetValue(&adcIDDetHardware.ADCHandle, ADC_INJECTED_RANK_1);
}

static uint16_t adcIDDetectReadIDDet(void)
{
    return HAL_ADCEx_InjectedGetValue(&adcIDDetHardware.ADCHandle, ADC_INJECTED_RANK_2);
}
#endif

void detectHardwareRevision(void)
{        
    adcIDDetectInit();

    uint32_t vrefintValue = 0;
    uint32_t iddetValue = 0;

    for (int i = 0 ; i < 16 ; i++) {
        adcIDDetectStart();
        adcIDDetectWait();
        iddetValue += adcIDDetectReadIDDet();
        vrefintValue += adcIDDetectReadVrefint();
    }

    vrefintValue /= 16;
    iddetValue /= 16;

    uint32_t iddetRatio = (iddetValue * vrefintValue) / *(uint16_t *)VREFINT_CAL_ADDR;
    iddetRatio = iddetRatio * 1000 / 4096;

#ifdef DEBUG_HARDWARE_REVISION_ADC
    debug[0] = *(uint16_t *)VREFINT_CAL_ADDR;
    debug[1] = vrefintValue;
    debug[2] = iddetValue;
    debug[3] = iddetRatio;
#endif

    for (size_t entry = 0; entry < ARRAYLEN(idDetectTable); entry++) {
#ifdef DEBUG_HARDWARE_REVISION_TABLE
        debug[0] = iddetRatio;
        debug[1] = idDetectTable[entry].ratio - IDDET_ERROR;
        debug[2] = idDetectTable[entry].ratio + IDDET_ERROR;
#endif
        if (idDetectTable[entry].ratio - IDDET_ERROR < iddetRatio && iddetRatio < idDetectTable[entry].ratio + IDDET_ERROR) {
            hardwareRevision = idDetectTable[entry].revision;
            break;
        }
    }

    adcIDDetectDeinit();
}

void updateHardwareRevision(void)
{
    // Empty
}

// XXX Can be gone as sensors/gyro.c is not calling this anymore
ioTag_t selectMPUIntExtiConfigByHardwareRevision(void)
{
    return IO_TAG_NONE;
}
#endif
