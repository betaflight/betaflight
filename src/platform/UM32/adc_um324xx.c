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

#include "build/debug.h"

#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/rcc.h"
#include "drivers/dma.h"
#include "drivers/sensor.h"
#include "drivers/adc.h"
#include "platform/adc_impl.h"

#include "pg/adc.h"


void Error_Handler(void);
uint32_t dmaGetInstance(dmaIdentifier_e identifier);

#define ADC_REGULAR_RANK_1    ((uint32_t)0x00000001)       /*!< ADC regular conversion rank 1  */
#define ADC_REGULAR_RANK_2    ((uint32_t)0x00000002)       /*!< ADC regular conversion rank 2  */
#define ADC_REGULAR_RANK_3    ((uint32_t)0x00000003)       /*!< ADC regular conversion rank 3  */
#define ADC_REGULAR_RANK_4    ((uint32_t)0x00000004)       /*!< ADC regular conversion rank 4  */
#define ADC_REGULAR_RANK_5    ((uint32_t)0x00000005)       /*!< ADC regular conversion rank 5  */
#define ADC_REGULAR_RANK_6    ((uint32_t)0x00000006)       /*!< ADC regular conversion rank 6  */
#define ADC_REGULAR_RANK_7    ((uint32_t)0x00000007)       /*!< ADC regular conversion rank 7  */
#define ADC_REGULAR_RANK_8    ((uint32_t)0x00000008)       /*!< ADC regular conversion rank 8  */
#define ADC_REGULAR_RANK_9    ((uint32_t)0x00000009)       /*!< ADC regular conversion rank 9  */
#define ADC_REGULAR_RANK_10   ((uint32_t)0x0000000A)       /*!< ADC regular conversion rank 10 */
#define ADC_REGULAR_RANK_11   ((uint32_t)0x0000000B)       /*!< ADC regular conversion rank 11 */
#define ADC_REGULAR_RANK_12   ((uint32_t)0x0000000C)       /*!< ADC regular conversion rank 12 */
#define ADC_REGULAR_RANK_13   ((uint32_t)0x0000000D)       /*!< ADC regular conversion rank 13 */
#define ADC_REGULAR_RANK_14   ((uint32_t)0x0000000E)       /*!< ADC regular conversion rank 14 */
#define ADC_REGULAR_RANK_15   ((uint32_t)0x0000000F)       /*!< ADC regular conversion rank 15 */
#define ADC_REGULAR_RANK_16   ((uint32_t)0x00000010)       /*!< ADC regular conversion rank 16 */

static TS_HandleTypeDef hts = {0};
static adcDevice_t adcDevice[ADCDEV_COUNT];

const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_APB2(ADC1),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = ADC1_DMA_CHANNEL
#endif
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_APB2(ADC2),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = ADC2_DMA_CHANNEL
#endif
    }
};

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
    { DEFIO_TAG_E__PA0, ADC_DEVICES_12,   ADC_CHANNEL_0,  0},
    { DEFIO_TAG_E__PA1, ADC_DEVICES_12,   ADC_CHANNEL_1,  1},
    { DEFIO_TAG_E__PA2, ADC_DEVICES_12,   ADC_CHANNEL_2,  2},
    { DEFIO_TAG_E__PA3, ADC_DEVICES_12,   ADC_CHANNEL_3,  3},
    { DEFIO_TAG_E__PA6, ADC_DEVICES_12,   ADC_CHANNEL_6,  6},
    { DEFIO_TAG_E__PA7, ADC_DEVICES_12,   ADC_CHANNEL_7,  7},
    { DEFIO_TAG_E__PB0, ADC_DEVICES_12,   ADC_CHANNEL_8,  8},
    { DEFIO_TAG_E__PB1, ADC_DEVICES_12,   ADC_CHANNEL_9,  9},
    { DEFIO_TAG_E__PC0, ADC_DEVICES_12,   ADC_CHANNEL_10, 10},
    { DEFIO_TAG_E__PC1, ADC_DEVICES_12,   ADC_CHANNEL_11, 11},
    { DEFIO_TAG_E__PC2, ADC_DEVICES_12,   ADC_CHANNEL_12, 12},
    { DEFIO_TAG_E__PC3, ADC_DEVICES_12,   ADC_CHANNEL_13, 13},
    { DEFIO_TAG_E__PC4, ADC_DEVICES_12,   ADC_CHANNEL_14, 14},
    { DEFIO_TAG_E__PC5, ADC_DEVICES_12,   ADC_CHANNEL_15, 15},
};

// An array to convert rank number to encoded rank code for HAL.
// Note that the table only list possible values, as rank for any single conversion
// will not exceed maximum number of input sources (ADC_CHANNEL_COUNT).
static uint32_t adcRegularRank[] = {
    0,                     // ranks is counted by 1-origin; dodge zero.
    ADC_REGULAR_RANK_1,
    ADC_REGULAR_RANK_2,
    ADC_REGULAR_RANK_3,
    ADC_REGULAR_RANK_4,
};

// Note on sampling time.
// Temperature sensor has minimum sample time of 9us.
// With prescaler = 4 at 200MHz (AHB1), fADC = 50MHz (tcycle = 0.02us), 9us = 450cycles < 810
static void adcInitDevice(adcDevice_t *adcdev, int channelCount)
{
    ADC_HandleTypeDef *hadc = &adcdev->ADCHandle; 

    hadc->Instance = adcdev->ADCx;

    hadc->Init.Opamp_en                 = ENABLE;
    hadc->Init.ContinuousConvMode       = ENABLE;
    hadc->Init.DiscontinuousConvMode    = DISABLE;
    hadc->Init.NbrOfDiscConversion      = 0;
    hadc->Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc->Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc->Init.ClockPrescaler           = 10;
    hadc->Init.NbrOfConversion          = channelCount;

	if(HAL_ADC_Init(hadc) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
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
void adcInitCalibrationValues(void)
{
    adcVREFINTCAL = VREFINT_CAL_VREF;
    adcTSCAL1 = *TEMPSENSOR_CAL1_ADDR;
    adcTSCAL2 = *TEMPSENSOR_CAL2_ADDR;
    adcTSSlopeK = (adcTSCAL2 - adcTSCAL1)/(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP);
}
#endif

// ADC conversion result DMA buffer
// Need this separate from the main adcValue[] array, because channels are numbered
// by ADC instance order that is different from ADC_xxx numbering.

volatile DMA_RAM_R uint16_t adcConversionBuffer[ADC_SOURCE_COUNT] __attribute__((aligned(32)));

void adcInit(const adcConfig_t *config)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memcpy(adcDevice, adcHardware, sizeof(adcDevice));

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

#ifdef USE_ADC_INTERNAL
    adcInitCalibrationValues();

	hts.Instance = TS;
	hts.Init.Mode 		= TS_NORMAL_MODE;
	hts.Init.WorkMode 	= TS_LOW_SPEED_MODE;
	hts.Init.ChopEn 	= TS_CHOP_CLK_EN;
	hts.Init.ChopDiv 	= 0x3FF;
	hts.Init.irq_en    = DISABLE;
	
	if(HAL_TS_Init(&hts) != HAL_OK)
	{
	/* Initialization Error */
		Error_Handler();
	}
    
    adcOperatingConfig[ADC_VREFINT].enabled = true;
    adcOperatingConfig[ADC_TEMPSENSOR].enabled = true;
#endif

#ifdef USE_OPA
	OPA_HandleTypeDef hopa;
	
	/* Opa Base configuration */
    hopa.Opax = OPA0;     
    hopa.Opa_Mode = HAL_OPA_MODE_UNITBUFF;                
    hopa.Init.Capen = OPA_CAPEN_OPEN;    
    hopa.Init.Fbresen = OPA_FBRESEN_OPEN;
    hopa.Init.GainSel = OPA_GAINSEL_1X;
    hopa.Init.Otpen = OPA_OTPEN_CLOSE;
    hopa.Init.Seln = OPA_SELN_RESERVE;
    hopa.Init.Selp = OPA_SELP_ADCMUX;
    
    if(HAL_OPA_Init(&hopa) != HAL_OK)
    {
        /* OPA Initialization Error */
        Error_Handler();
    }
#endif

    for (int i = 0; i < ADC_EXTERNAL_COUNT; i++) {
        int map;
        int dev;

        {
            if (!adcOperatingConfig[i].tag) {
                continue;
            }

            map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
            if (map < 0) {
                continue;
            }

            // Found a tag map entry for this input pin
            // Find an ADC device that can handle this input pin

            for (dev = 0; dev < ADCDEV_COUNT; dev++) {
                if (!adcDevice[dev].ADCx
#ifndef USE_DMA_SPEC
                     || !adcDevice[dev].dmaResource
#endif
                   ) {
                    // Instance not activated
                    continue;
                }
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

        // At this point, map is an entry for the input pin and dev is a valid ADCx for the pin for input i
        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].sampleTime = 0;
        adcOperatingConfig[i].enabled = true;

        adcDevice[dev].channelBits |= (1 << adcTagMap[map].channelOrdinal);

        // Configure a pin for ADC
        if (adcOperatingConfig[i].tag) {
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL));
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
            Error_Handler();
        }
    }

    // Configure ADCx with inputs

    int dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        RCC_ClockCmd(adc->rccADC, ENABLE);

        int configuredAdcChannels = BITCOUNT(adc->channelBits);

        adcInitDevice(adc, configuredAdcChannels);

        // Configure channels

        int rank = 1;

        for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {

            if (!adcOperatingConfig[adcChan].enabled) {
                continue;
            }

            if (adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }

            adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;

            ADC_ChannelConfTypeDef sConfig = {0};

            sConfig.Channel      = adcOperatingConfig[adcChan].adcChannel; /* Sampled channel number */
            sConfig.Rank         = adcRegularRank[rank++];      /* Rank of sampled channel number ADCx_CHANNEL */
            sConfig.AverageTimes = ADC_CHANNEL_AVERAGE_TIMES_1;            /* Sampling time (number of clock cycles unit) */

            if (HAL_ADC_ConfigChannel(&adc->ADCHandle, &sConfig) != HAL_OK) {
                Error_Handler();
            }
        }

#if(0) //Don't use DMA 
        if(HAL_ADC_Start(&adc->ADCHandle) != HAL_OK)
        {
            /* Start Conversation Error */
            Error_Handler();
        }

        uint32_t adc_val = 0;
        while(1){
            HAL_ADC_PollForConversion(&adc->ADCHandle, 10);

            if((HAL_ADC_GetState(&adc->ADCHandle) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
            {
                /*##-5- Get the converted value of regular channel  ######################*/
                adc_val = HAL_ADC_GetValue(&adc->ADCHandle, adcOperatingConfig[0].adcChannel) & 0xFFF;
                tfp_printf("%d\n", adc_val);
                delay(1);
            }
        }
#endif


        // Configure DMA for this ADC peripheral
#ifdef USE_DMA_SPEC
        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);

        dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);
        if (!dmaSpec || !dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
            return;
        }

        adc->DmaHandle.Instance                 = (DMA_TypeDef *)dmaGetInstance(dmaGetIdentifier(dmaSpec->ref));
        adc->DmaHandle.DmaChannelSel            = dmaSpec->channel;
#else
        //...
#endif
        adc->DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;

        adc->DmaHandle.Init.DstMSize            = DMA_BURST_DST_NUM_1;   
        adc->DmaHandle.Init.SrcMSize            = DMA_BURST_SRC_NUM_1;   
   
        adc->DmaHandle.Init.SrcInc              = DMA_SRCINC_NOC;    
        adc->DmaHandle.Init.DstInc              = DMA_DSTINC_INC;     
        adc->DmaHandle.Init.SrcDataAlignment    = DMA_SRCDATAALIGN_HALFWORD;  
        adc->DmaHandle.Init.DstDataAlignment    = DMA_DSTDATAALIGN_HALFWORD;  
	
        adc->DmaHandle.Init.SrcHsSel            = DMA_SRC_HS_HW;       
        adc->DmaHandle.Init.DstHsSel            = DMA_DST_HS_HW;        
    
        adc->DmaHandle.Init.SrcPer              = DMA_SRC_HANDSHAKING(dmaSpec->code);    
        adc->DmaHandle.Init.DstPer              = DMA_DST_HANDSHAKING(0);     
    
        adc->DmaHandle.Init.SrcReload           = DMA_SRC_RELOAD_ENABLE;  
        adc->DmaHandle.Init.DstReload           = DMA_DST_RELOAD_ENABLE; 
    
        adc->DmaHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;    
        adc->DmaHandle.Init.FCMode              = DMA_FCMODE_ENABLE;   

        adc->DmaHandle.Init.Priority            = LL_DMA_PRIORITY_1;
        // Deinitialize  & Initialize the DMA for new transfer

        // dmaEnable must be called before calling HAL_DMA_Init,
        // to enable clock for associated DMA if not already done so.
        dmaEnable(dmaIdentifier);

        HAL_DMA_DeInit(&adc->DmaHandle);
        if (HAL_DMA_Init(&adc->DmaHandle) != HAL_OK)
        {
            /* Initialization Error */
            Error_Handler();
        }

        // Associate the DMA handle

        __HAL_LINKDMA(&adc->ADCHandle, DMA_Handle, adc->DmaHandle);

#ifdef USE_ADC_INTERRUPT
        // XXX No interrupt used, so we can skip this.
        // If interrupt is needed in any case, use dmaXXX facility instead,
        // using dmaIdentifier obtained above.

        // NVIC configuration for DMA Input data interrupt

        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
#endif
    }

    // Start channels.
    // This must be done after channel configuration is complete, as HAL_ADC_ConfigChannel
    // throws an error when configuring internal channels if ADC1 or ADC2 are already enabled.
    dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {

        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        // Start conversion in DMA mode

        if (HAL_ADC_Start_DMA(&adc->ADCHandle, (uint32_t *)&adcConversionBuffer[dmaBufferIndex], BITCOUNT(adc->channelBits)) != HAL_OK) {
            Error_Handler();
        }

        dmaBufferIndex += BITCOUNT(adc->channelBits);
    }
}

void adcGetChannelValues(void)
{
    // Transfer values in conversion buffer into adcValues[]
    // Cache coherency should be maintained by MPU facility
    for (int i = 0; i < ADC_EXTERNAL_COUNT; i++) {
        if (adcOperatingConfig[i].enabled) {
            adcValues[adcOperatingConfig[i].dmaIndex] = adcConversionBuffer[adcOperatingConfig[i].dmaIndex] & 0xFFF;
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


uint16_t adcInternalReadVrefint(void)
{
    return adcVREFINTCAL;
}

uint16_t adcInternalReadTempsensor(void)
{
    uint16_t value = HAL_TS_get_data(&hts);
    if(value == 0){
        value = adcTSCAL1 + adcTSCAL1 / 4;
    }
    return value;
}

uint16_t adcInternalRead(adcSource_e source)
{
    switch (source) {
    case ADC_VREFINT:
        return adcInternalReadVrefint();
    case ADC_TEMPSENSOR:
        return adcInternalReadTempsensor();
    default:
        return 0;
    }
}

#endif // USE_ADC_INTERNAL
#endif // USE_ADC


