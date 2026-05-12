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
#include "drivers/resource.h"
#include "drivers/dma.h"
#include "platform/dma.h"
#include "drivers/sensor.h"

#include "drivers/adc.h"
#include "platform/adc_impl.h"

#include "pg/adc.h"

const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_AHB1_1(ADC1PLL),
#if !defined(USE_DMA_SPEC)
        .dmaResource = NULL
#endif
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_AHB1_4(ADC2PLL),
#if !defined(USE_DMA_SPEC)
        .dmaResource = NULL
#endif

    },
    {
        .ADCx = ADC3,
        .rccADC = RCC_AHB1_4(ADC3PLL),
#if !defined(USE_DMA_SPEC)
        .dmaResource = NULL
#endif
    },
};


adcDevice_t adcDevice[ADCDEV_COUNT];

#define ADC_DEVICE_FOR_INTERNAL ADC_DEVICES_3

const adcTagMap_t adcTagMap[] = {
#ifdef USE_ADC_INTERNAL
#define ADC_TAG_MAP_VREFINT    0
#define ADC_TAG_MAP_TEMPSENSOR 1
#define ADC_TAG_MAP_VBAT4      2
    { DEFIO_TAG_E__NONE, ADC_DEVICES_3,   ADC3_Channel_19_VREFINT    },
    { DEFIO_TAG_E__NONE, ADC_DEVICES_3,   ADC3_Channel_18_Temperture_Sensor},
    { DEFIO_TAG_E__NONE, ADC_DEVICES_3,   ADC3_Channel_17_Battery_DIV4 }, //
#endif
    { DEFIO_TAG_E__PA0,  ADC_DEVICES_1,    ADC_CH_16  },
    { DEFIO_TAG_E__PA1,  ADC_DEVICES_1,    ADC_CH_17  },
    { DEFIO_TAG_E__PA2,  ADC_DEVICES_12,   ADC_CH_14  },
    { DEFIO_TAG_E__PA3,  ADC_DEVICES_12,   ADC_CH_15  },
    { DEFIO_TAG_E__PA4,  ADC_DEVICES_12,   ADC_CH_18  },
    { DEFIO_TAG_E__PA5,  ADC_DEVICES_12,   ADC_CH_19  },
    { DEFIO_TAG_E__PA6,  ADC_DEVICES_12,   ADC_CH_3   },
    { DEFIO_TAG_E__PA7,  ADC_DEVICES_12,   ADC_CH_7   },
    { DEFIO_TAG_E__PB0,  ADC_DEVICES_12,   ADC_CH_9   },
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_12,   ADC_CH_5   },
    { DEFIO_TAG_E__PC0,  ADC_DEVICES_123,  ADC_CH_10  },
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_123,  ADC_CH_11  },
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_123,  ADC_CH_12  },
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_12,   ADC_CH_13  },
    { DEFIO_TAG_E__PC4,  ADC_DEVICES_12,   ADC_CH_4   },
    { DEFIO_TAG_E__PC5,  ADC_DEVICES_12,   ADC_CH_8   },
    { DEFIO_TAG_E__PH2,  ADC_DEVICES_3,    ADC_CH_13  },
    { DEFIO_TAG_E__PH3,  ADC_DEVICES_3,    ADC_CH_14  },
    { DEFIO_TAG_E__PH4,  ADC_DEVICES_3,    ADC_CH_15  },
    { DEFIO_TAG_E__PH5,  ADC_DEVICES_3,    ADC_CH_16  },
    { DEFIO_TAG_E__PF3,  ADC_DEVICES_3,    ADC_CH_5   },
    { DEFIO_TAG_E__PF4,  ADC_DEVICES_3,    ADC_CH_9   },
    { DEFIO_TAG_E__PF5,  ADC_DEVICES_3,    ADC_CH_4   },
    { DEFIO_TAG_E__PF6,  ADC_DEVICES_3,    ADC_CH_8   },
    { DEFIO_TAG_E__PF7,  ADC_DEVICES_3,    ADC_CH_3   },
    { DEFIO_TAG_E__PF8,  ADC_DEVICES_3,    ADC_CH_7   },
    { DEFIO_TAG_E__PF9,  ADC_DEVICES_3,    ADC_CH_2   },
    { DEFIO_TAG_E__PF10, ADC_DEVICES_3,    ADC_CH_6   }, 
    { DEFIO_TAG_E__PF11, ADC_DEVICES_1,    ADC_CH_2   },
    { DEFIO_TAG_E__PF12, ADC_DEVICES_1,    ADC_CH_6   },
    { DEFIO_TAG_E__PJ0,  ADC_DEVICES_2,    ADC_CH_16  },
};

static volatile uint16_t adcConversionBuffer[ADC_SOURCE_COUNT];

/**
 * Initialise the specified ADC to read multiple channels in repeat mode
 *
 * Sets 12 bit resolution, right aligned
 *
 * @param dev Specifies the ADC device to use
 * @param channelCount how many channels to repeat over
 *
*/
static void adcInitDevice(const adcDevice_t *adcdev, const int channelCount)
{
    ADC_InitType ADC_InitStructure;
    if(adcdev->ADCx == ADC1)
    {
        RCC_EnableAHB1PeriphClk1(RCC_AHB1_PERIPHEN_M7_ADC1SYS | RCC_AHB1_PERIPHEN_M7_ADC1BUS, ENABLE);
        ADC_ConfigClk(ADC1, ADC_CTRL3_CKMOD_AHB, RCC_ADCPLLCLK_SRC_PLL1B, RCC_ADCSYSCLK_DIV20);
    }
    else if(adcdev->ADCx == ADC2)
    {
        RCC_EnableAHB1PeriphClk4(RCC_AHB1_PERIPHEN_M7_ADC2SYS | RCC_AHB1_PERIPHEN_M7_ADC2BUS, ENABLE);
        ADC_ConfigClk(ADC2, ADC_CTRL3_CKMOD_AHB, RCC_ADCPLLCLK_SRC_PLL1B, RCC_ADCSYSCLK_DIV20);
    }
    else if(adcdev->ADCx == ADC3)
    {
        RCC_EnableAHB1PeriphClk4(RCC_AHB1_PERIPHEN_M7_ADC3SYS | RCC_AHB1_PERIPHEN_M7_ADC3BUS, ENABLE);
        ADC_ConfigClk(ADC3, ADC_CTRL3_CKMOD_AHB, RCC_ADCPLLCLK_SRC_PLL1B, RCC_ADCSYSCLK_DIV20);
    }
    

    ADC_InitStruct(&ADC_InitStructure);   // not currently needed, but insurance against the struct being changed in the future

    ADC_InitStructure.WorkMode         = ADC_WORKMODE_INDEPENDENT;
    ADC_InitStructure.ContinueConvEn   = ENABLE;
    ADC_InitStructure.ExtTrigSelect    = ADC_EXT_TRIG_REG_CONV_SOFTWARE;
    ADC_InitStructure.DataTransferMode = ADC_REG_DMA_TRANSFER;
    ADC_InitStructure.DatAlign         = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber        = channelCount;
    ADC_InitStructure.Resolution       = ADC_DATA_RES_12BIT;

    // Multiple injected channel seems to require scan conversion mode to be
    // enabled even if main (non-injected) channel count is 1.
    if(adcdev->ADCx == ADC3)
    {
        #ifdef USE_ADC_INTERNAL
            ADC_InitStructure.MultiChEn             = ENABLE;
        #else
            ADC_InitStructure.MultiChEn            = channelCount > 1 ? ENABLE : DISABLE; 
        #endif
    }
    else
    {
        ADC_InitStructure.MultiChEn            = channelCount > 1 ? ENABLE : DISABLE; 
    }

    ADC_Init(adcdev->ADCx, &ADC_InitStructure);
}

/**
 * Find a given pin (defined by ioTag) in the map
 *
 * @param tag the ioTag to search for
 * @return the index in adcTagMap corresponding to the given ioTag or -1 if not found
*/
static int adcFindTagMapEntry(const ioTag_t tag)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].tag == tag) {
            return i;
        }
    }
    return -1;
}

/**
 * Setup the scaling offsets and factors used in adc.c
 * @see src/main/drivers/adc.c
 * @see src/platform/common/stm32/platform/adc_impl.h
 *
 * There are a number of global calibration/scaling factors used in src/main/drivers/adc.c that need to
 * be set to appropriate values if we want to re-use existing code, e.g. adcInternalComputeTemperature
 * (the alternative would be to duplicate the code into ST and AT specific versions).
 * This is made a little confusing since the implementation based on ST datasheets approaches the calculation with
 * different formula and express the scaling factors in different units compared to the AT datasheets.
 * The constants are defined in src/platform/common/stm32/platform/adc_impl.h. It seems clearest to use the units from
 * the datasheet when defining those values, so here we have to convert to what's expected in
 * adcInternalComputeTemperature.
*/
#ifdef USE_ADC_INTERNAL
static void setScalingFactors(void)
{
    // The expected reading for 1.2V internal reference if external vref+ was 3.3V
    adcVREFINTCAL = VREFINT_EXPECTED;

    // adcTSCAL1 needs to be the raw ADC value at 25C, but we can't adjust it for VREF+ because we haven't calculated it yet
    // So this will have to be aproximate (and the use should be fixed at some point)
    adcTSCAL1 = (TEMPSENSOR_CAL1_V * 4095.0f) / 3.3f;

    // TEMPSENSOR_SLOPE is given in mv/C.
    // adcTSSlopeK has the opposite sign compared to the AT32 standard because adcInternalComputeTemperature subtracts
    // the calibration value from the reading where as the AT32 docs subtract the reading from the calibration.
    // 3300/4095 converts the reading to mV and the factor of 1000 is needed to prevent the slope from rounding to 0.
    // The intermediate result when this is used is in mC, and adcInternalComputeTemperature then divides by 1000
    // to get an answer in C
    adcTSSlopeK = lrintf(-3300.0f*1000.0f/4095.0f/TEMPSENSOR_SLOPE);
}
#endif

/**
 * Setup the ADC so that it's running in the background and ready to
 * provide channel data
 *
 * Notes:
 *  This code only uses ADC1 despite appearances to the contrary, and has not been tested with the other ADCs
 *
 * From the RM:
 *   ADCCLK must be less than 80 MHz, while the ADCCLK frequency must be lower than PCLK2
 *
 * PCLK2 looks like it's running at 144Mhz, but should be confirmed
 *
 * With HCLK of 288, a divider of 4 gives an ADCCLK of 72MHz
 *
 * sample time is
 *   ADC_SAMPLE + nbits + 0.5 ADCCLK periods
 *
 * So with 12bit samples and ADC_SAMPLE_92_5 that's 105 clks.
 *
 * We're using HCLK/4, so 288/4 = 72MHz, each tick is 14ns. Add 5 clks for the interval between conversions,
 * 110 clks total is 1.54us per channel.
 *
 * Max 6 channels is a total of 9.24us, or a basic sample rate of 108kHz per channel
 *
 * If we use 64x oversampling we'll get an effective rate of 1.7kHz per channel which should still be plenty.
 *
 * (RM and DS mention fast and slow channels, but don't give details on how this affects the above.
 *  It's not relevant to our use case, so ignore the difference for now)
 *
 * Called from fc/init.c
 *
 * @param config - defines the channels to use for each external input (vbat, rssi, current, external) and also has calibration values for the temperature sensor
 *
*/

extern void AFIO_ConfigPinAnalogSignalChannel(IO_t io, FunctionalState cmd);
void adcInit(const adcConfig_t *config)
{
    uint32_t nChannelsUsed[ADCDEV_COUNT] = {0};

    for (int i = 0; i < ADCDEV_COUNT; i++) {
        adcDevice[i] = adcHardware[i];
    }

#ifdef USE_ADC_INTERNAL
    setScalingFactors();
#endif

    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));  // adcOperatingConfig defined in generic adc.c

    // Set channels (ioTags) for enabled ADC inputs
    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;
    }

    // loop over all possible channels and build the adcOperatingConfig to represent
    // the set of enabled channels
    for (int i = 0; i < ADC_SOURCE_COUNT; i++) {
        int map;
        int dev;

        switch(i) {
#ifdef USE_ADC_INTERNAL
        case ADC_TEMPSENSOR:
            map = ADC_TAG_MAP_TEMPSENSOR;
            dev = ADCDEV_3;
            adcOperatingConfig[i].sampleTime = ADC_SAMP_TIME_CYCLES_397;
            break;
        case ADC_VREFINT:
            map = ADC_TAG_MAP_VREFINT;
            dev = ADCDEV_3;
            adcOperatingConfig[i].sampleTime = ADC_SAMP_TIME_CYCLES_397;
            break;
#if ADC_INTERNAL_VBAT4_ENABLED
        case ADC_VBAT4:
            map = ADC_TAG_MAP_VBAT4;
            dev = ADCDEV_3;
            break;
#endif

#endif
        default:
            if (!adcOperatingConfig[i].tag) {
                continue;
            }

            map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
            if (map < 0) {
                continue;
            }

            // Since ADC1 can do all channels this will only ever return adc1 and is unnecessary

            // Find an ADC instance that can be used for the given TagMap index.
            for (dev = 0; dev < ADCDEV_COUNT; dev++) {
                #ifndef USE_DMA_SPEC
                if (!adcDevice[dev].ADCx || !adcDevice[dev].dmaResource) {
                    continue;
                }
                #else
                if (!adcDevice[dev].ADCx) {
                    continue;
                }
                #endif

                if (adcTagMap[map].devices & (1 << dev)) {
                    break;
                }
            }
        }

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].sampleTime = ADC_SAMP_TIME_CYCLES_24;
        adcOperatingConfig[i].enabled = true;

        nChannelsUsed[dev] += 1;    // increase the active channel count for this device

        // Enable the gpio for analog input
        if (adcOperatingConfig[i].tag) {
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG,GPIO_SLEW_RATE_SLOW, GPIO_NO_PULL,0x00 ));
            AFIO_ConfigPinAnalogSignalChannel(IOGetByTag(adcOperatingConfig[i].tag), ENABLE);
            if(adcOperatingConfig[i].tag == 0xa0)
            {
                AFIO_ADCSWPJ0Enable(ENABLE);
            }
        }
    }   // for each channel

    ADC_DeInit(ADC1);    // reset all ADC instances
    ADC_DeInit(ADC2); 
    ADC_DeInit(ADC3); 
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_AFIO, ENABLE);
    // Enable the clock (power up the circuit)
    // This has to be done before calling adc_common_config for reasons that are unclear
    // NB only ADC1 will have any channels, so this loop for pedantry only
    // for (int i = 0; i < ADCDEV_COUNT; i++) {
    //     if (nChannelsUsed[i] > 0) {
    //         RCC_ClockCmd(adcHardware[i].rccADC, ENABLE);
    //     }
    // }


    int  dmaBufferIndex = 0;
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        const adcDevice_t *adc = &adcHardware[dev];

        // skip this device if it doesn't have any active channels
        if (nChannelsUsed[dev] == 0) {
            continue;
        }

        adcInitDevice(adc, nChannelsUsed[dev]);

        // Set the oversampling ratio and matching shift
        ADC_ConfigOverSamplingRatioAndShift(adc->ADCx, ADC_OVERSAMPE_RATE_TIMES_64, ADC_OVERSAMPE_DATA_SHIFT_6);

#ifdef USE_DMA_SPEC

        // Setup the DMA channel so that data is automatically and continuously transferred from the ADC output register
        // to the results buffer

        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
        if (dmaSpec == NULL) {
            return;
        }

        dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);
        if (!dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
            return;
        }

        dmaEnable(dmaIdentifier);   // enables clock/power
        dmaMuxEnable(dmaIdentifier, dmaSpec->dmaMuxId);

        DMA_ChInitType DMA_InitStructure;
        DMA_ControllerCmd(dmaX32ControllerFromChannel((DMA_ARCH_TYPE *)dmaSpec->ref), ENABLE);

        DMA_ChannelStructInit(&DMA_InitStructure);
        DMA_InitStructure.IntEn              = 0x1U;
        DMA_InitStructure.DstTfrWidth        = DMA_CH_TRANSFER_WIDTH_16;
        DMA_InitStructure.SrcTfrWidth        = DMA_CH_TRANSFER_WIDTH_16;
        DMA_InitStructure.DstAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
        DMA_InitStructure.SrcAddrCountMode   = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
        DMA_InitStructure.DstBurstLen        = DMA_CH_BURST_LENGTH_1;
        DMA_InitStructure.SrcBurstLen        = DMA_CH_BURST_LENGTH_1;
        DMA_InitStructure.TfrTypeFlowCtrl    = DMA_CH_TRANSFER_FLOW_P2M_DMA;
        DMA_InitStructure.BlkTfrSize         = nChannelsUsed[dev];
        DMA_InitStructure.SrcAddr            = (uint32_t)&(adc->ADCx->DAT);
        DMA_InitStructure.DstAddr            = (uint32_t)&(adcConversionBuffer[dmaBufferIndex]);;
        DMA_InitStructure.TfrType            = DMA_CH_TRANSFER_TYPE_MULTI_BLOCK_SRCADR_RELOAD_DSTADR_RELOAD;
        DMA_InitStructure.ChannelPriority    = DMA_CH_PRIORITY_0;
        DMA_InitStructure.SrcHandshaking     = DMA_CH_SRC_HANDSHAKING_HARDWARE;
        DMA_InitStructure.SrcHsInterface     = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)dmaSpec->ref);
        DMA_InitStructure.SrcHsInterfacePol  = DMA_CH_HANDSHAKING_IF_POL_H;
        xDMA_Init(dmaSpec->ref, &DMA_InitStructure);

        // enable the DMA channel
        xDMA_Cmd(dmaSpec->ref, ENABLE);
#endif //end of USE_DMA_SPEC

        uint8_t rank = 1;
        // set each channel into the auto sequence for this ADC device
        for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++)
        {
            // only add enabled channels for the current dev (can be simplified if we drop the pretense at handling adc2 and 3)
            if (adcOperatingConfig[adcChan].enabled && adcOperatingConfig[adcChan].adcDevice == dev)
            {
                if(adc->ADCx ==ADC3)
                {
                    if(adcOperatingConfig[adcChan].adcChannel == ADC3_Channel_19_VREFINT)
                    {
                        ADC_EnableVrefint(ENABLE);
                    }
                    else if(adcOperatingConfig[adcChan].adcChannel == ADC3_Channel_18_Temperture_Sensor)
                    {
                        ADC_EnableTempSensor(ENABLE);
                    }
                    else if(adcOperatingConfig[adcChan].adcChannel == ADC3_Channel_17_Battery_DIV4)
                    {
                        ADC_EnableBatteryVoltageMonitor(ENABLE);
                    }
                    else
                    {
                        /**nop */
                    }
                }
                
                adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;
                ADC_ConfigRegularChannel(adc->ADCx,
                    adcOperatingConfig[adcChan].adcChannel, rank++,
                    adcOperatingConfig[adcChan].sampleTime );
            }
        }

        ADC_Enable(adc->ADCx, ENABLE);
        while(ADC_GetFlagStatus(adc->ADCx,ADC_FLAG_RDY) == RESET)
        {
        } // wait for ready

        ADC_CalibrationOperation(adc->ADCx);
        while (ADC_GetCalibrationStatus(adc->ADCx) == SET) {
            // wait for calibration
        }

        ADC_StartRegularConv(adc->ADCx);  // start sampling
        /* Start ADC Software Conversion */
        ADC_EnableSoftwareStartConv(adc->ADCx, ENABLE);
    }
}

/**
 * Copies the latest ADC external channel data into adcValues defined in adc.c
*/
void adcGetChannelValues(void)
{
    for (unsigned i = 0; i < ADC_EXTERNAL_COUNT; i++) {
        if (adcOperatingConfig[i].enabled) {
            adcValues[adcOperatingConfig[i].dmaIndex] = adcConversionBuffer[adcOperatingConfig[i].dmaIndex];
        }
    }
}

#ifdef USE_ADC_INTERNAL

/**
 * This impl is never busy in the sense of being unable to supply data
*/
bool adcInternalIsBusy(void)
{
    return false;
}

/**
 * Nop, since the ADC is free running
*/
void adcInternalStartConversion(void)
{
    return;
}

/**
 * Reads a given internal channel from the DMA buffer
*/
uint16_t adcInternalRead(adcSource_e source)
{
    switch (source) {
    case ADC_VREFINT:
    case ADC_TEMPSENSOR: 
#if ADC_INTERNAL_VBAT4_ENABLED
    case ADC_VBAT4:
#endif
    {
        const unsigned dmaIndex = adcOperatingConfig[source].dmaIndex;
        return dmaIndex < ARRAYLEN(adcConversionBuffer) ? adcConversionBuffer[dmaIndex] : 0;
    }
    default:
        return 0;
    }
}

#endif // USE_ADC_INTERNAL
#endif // USE_ADC
