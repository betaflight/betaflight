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

/**
 * Read internal and external analog channels
 * 
 * Internal channels provide temperature and the internal voltage reference
 * External channels are for vbat, rssi, current and a generic 'external' inputs
 * 
 * The ADC is free running and so doesn't require a timer. Samples are moved from 
 * the ADC output register to a buffer by DMA
 * 
 * The sample rate is kept low to reduce impact on the DMA controller, and the lowest
 * priority is set for the DMA transfer. It's also recommended to use the highest numbered
 * DMA channel on the dma controller for ADC, since that is the lowest priority channel
 * for transfers at the same DMA priority.
 * 
 * Sample rate is set between 1 and 2kHz by using a long input sampling time and reasonably
 * high hardware oversampling.
 * 
 * Note that only ADC1 is used, although the code contains remnants of support for all
 * three ADC.
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
#include "drivers/rcc.h"
#include "drivers/resource.h"
#include "drivers/dma.h"

#include "drivers/sensor.h"

#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "pg/adc.h"


const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_APB2(ADC1),
#if !defined(USE_DMA_SPEC)
        .dmaResource = NULL
#endif
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_APB2(ADC2),
#if !defined(USE_DMA_SPEC)
        .dmaResource = NULL
#endif

    },
    {
        .ADCx = ADC3,
        .rccADC = RCC_APB2(ADC3),
#if !defined(USE_DMA_SPEC)
        .dmaResource = NULL
#endif
    },
};

#define ADC_CHANNEL_VREFINT         ADC_CHANNEL_17
#define ADC_CHANNEL_TEMPSENSOR_ADC1 ADC_CHANNEL_16

const adcTagMap_t adcTagMap[] = {
#ifdef USE_ADC_INTERNAL
#define ADC_TAG_MAP_VREFINT    0
#define ADC_TAG_MAP_TEMPSENSOR 1
    { DEFIO_TAG_E__NONE, ADC_DEVICES_1,   ADC_CHANNEL_VREFINT,         17 },
    { DEFIO_TAG_E__NONE, ADC_DEVICES_1,   ADC_CHANNEL_TEMPSENSOR_ADC1, 16 },
#endif

    { DEFIO_TAG_E__PA0,  ADC_DEVICES_123,  ADC_CHANNEL_0,   0  },
    { DEFIO_TAG_E__PA1,  ADC_DEVICES_123,  ADC_CHANNEL_1,   1  },
    { DEFIO_TAG_E__PA2,  ADC_DEVICES_123,  ADC_CHANNEL_2,   2  },
    { DEFIO_TAG_E__PA3,  ADC_DEVICES_123,  ADC_CHANNEL_3,   3  },
    { DEFIO_TAG_E__PA4,  ADC_DEVICES_12,   ADC_CHANNEL_4,   4  },
    { DEFIO_TAG_E__PA5,  ADC_DEVICES_12,   ADC_CHANNEL_5,   5  },
    { DEFIO_TAG_E__PA6,  ADC_DEVICES_12,   ADC_CHANNEL_6,   6  },
    { DEFIO_TAG_E__PA7,  ADC_DEVICES_12,   ADC_CHANNEL_7,   7  },
    { DEFIO_TAG_E__PB0,  ADC_DEVICES_12,   ADC_CHANNEL_8,   8  },
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_12,   ADC_CHANNEL_9,   9  },
    { DEFIO_TAG_E__PC0,  ADC_DEVICES_123,  ADC_CHANNEL_10,  10 },
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_123,  ADC_CHANNEL_11,  11 },
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_123,  ADC_CHANNEL_12,  12 },
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_123,  ADC_CHANNEL_13,  13 },
    { DEFIO_TAG_E__PC4,  ADC_DEVICES_12,   ADC_CHANNEL_14,  14 },
    { DEFIO_TAG_E__PC5,  ADC_DEVICES_12,   ADC_CHANNEL_15,  15 },
};

static volatile DMA_DATA uint32_t adcConversionBuffer[ADC_CHANNEL_COUNT];

/**
 * Initialise the specified ADC to read multiple channels in repeat mode
 * 
 * Sets 12 bit resolution, right aligned
 * 
 * @param dev Specifies the ADC device to use
 * @param channelCount how many channels to repeat over
 * 
*/
void adcInitDevice(const adc_type *dev, const int channelCount)
{
    adc_base_config_type adc_base_struct;

    adc_base_default_para_init(&adc_base_struct);   // not currently needed, but insurance against the struct being changed in the future
    adc_base_struct.sequence_mode = TRUE;           // reading multiple channels
    adc_base_struct.repeat_mode = TRUE;             // free running, so no need to retrigger
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
    adc_base_struct.ordinary_channel_length = channelCount;

    adc_base_config((adc_type *)dev, &adc_base_struct);
    adc_resolution_set((adc_type *)dev, ADC_RESOLUTION_12B);
}

/**
 * Find a given pin (defined by ioTag) in the map
 * 
 * @param tag the ioTag to search for
 * @return the index in adcTagMap corresponding to the given ioTag or -1 if not found
*/
int adcFindTagMapEntry(const ioTag_t tag)
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
 * @see src/main/drivers/adc_impl.h
 * 
 * There are a number of global calibration/scaling factors used in src/main/drivers/adc.c that need to
 * be set to appropriate values if we want to re-use existing code, e.g. adcInternalComputeTemperature
 * (the alternative would be to duplicate the code into ST and AT specific versions).
 * This is made a little confusing since the implementation based on ST datasheets approaches the calculation with
 * different formula and express the scaling factors in different units compared to the AT datasheets. 
 * The constants are defined in src/main/drivers/adc_impl.h. It seems clearest to use the units from
 * the datasheet when defining those values, so here we have to convert to what's expected in
 * adcInternalComputeTemperature.
*/
void setScalingFactors(void)
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
void adcInit(const adcConfig_t *config)
{
    uint32_t nChannelsUsed[ADCDEV_COUNT] = {0};

    setScalingFactors();

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
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        int map;
        int dev;

        if (i == ADC_TEMPSENSOR) {
            map = ADC_TAG_MAP_TEMPSENSOR;
            dev = ADCDEV_1;
        } else if (i == ADC_VREFINT) {
            map = ADC_TAG_MAP_VREFINT;
            dev = ADCDEV_1;
        } else {
            if (!adcOperatingConfig[i].tag) {
                continue;
            }

            map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
            if (map < 0) {
                continue;
            }

            // Since ADC1 can do all channels this will only ever return adc1 and is unnecessary

            // Find an ADC instance that can be used for the given TagMap index. 
            // for (dev = 0; dev < ADCDEV_COUNT; dev++) {
            //     #ifndef USE_DMA_SPEC
            //     if (!adcDevice[dev].ADCx || !adcDevice[dev].dmaResource) {
            //         continue;
            //     }
            //     #else
            //     if (!adcDevice[dev].ADCx) {
            //         continue;
            //     }
            //     #endif

            //     if (adcTagMap[map].devices & (1 << dev)) {
            //         break;
            //     }
            // }
            dev = ADCDEV_1;
        }

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_92_5;
        adcOperatingConfig[i].enabled = true;

        nChannelsUsed[dev] += 1;    // increase the active channel count for this device

        // Enable the gpio for analog input
        if (adcOperatingConfig[i].tag) {
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG,GPIO_DRIVE_STRENGTH_MODERATE, 0, GPIO_PULL_NONE));
        }
    }   // for each channel

    adc_reset();    // reset all ADC instances

    // Enable the clock (power up the circuit)
    // This has to be done before calling adc_common_config for reasons that are unclear
    // NB only ADC1 will have any channels, so this loop for pedantry only
    for (int i = 0; i < ADCDEV_COUNT; i++) {
        if (nChannelsUsed[i] > 0) {
            RCC_ClockCmd(adcHardware[0].rccADC, ENABLE);
        }
    }

    // Common config applies equally to all three ADCs, so only apply it once
    adc_common_config_type adc_common_struct;
    adc_common_default_para_init(&adc_common_struct);
    adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
    adc_common_struct.div = ADC_HCLK_DIV_4;
    adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
    adc_common_struct.common_dma_request_repeat_state = FALSE;
    adc_common_struct.sampling_interval = ADC_SAMPLING_INTERVAL_5CYCLES;
    adc_common_struct.tempervintrv_state = TRUE;    // internal channels need to be enabled before use

    adc_common_config(&adc_common_struct);


    // Only adc1 will have any channels assigned to it after the code above, so this can be simplified

    int  dmaBufferIndex = 0;
    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        const adcDevice_t *adc = &adcHardware[dev];

        // skip this device if it doesn't have any active channels
        if (nChannelsUsed[dev] == 0) {
            continue;
        }

        adcInitDevice(adc->ADCx, nChannelsUsed[dev]);

        // Set the oversampling ratio and matching shift
        adc_oversample_ratio_shift_set(adc->ADCx, ADC_OVERSAMPLE_RATIO_64, ADC_OVERSAMPLE_SHIFT_6);


        #ifdef USE_DMA_SPEC

        // Setup the DMA channel so that data is automatically and continuously transferred from the ADC output register
        // to the results buffer

        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
        if (dmaSpec == NULL) {
            return;
        }

        dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);
        if ( ! dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev)) ) {
            return;
        }

        dmaEnable(dmaIdentifier);   // enables clock/power
        xDMA_DeInit(dmaSpec->ref);

        dma_init_type dma_init_struct;
        dma_default_para_init(&dma_init_struct);
        dma_init_struct.buffer_size = nChannelsUsed[dev];
        dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
        dma_init_struct.memory_inc_enable = TRUE;
        dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
        dma_init_struct.peripheral_inc_enable = FALSE;
        dma_init_struct.priority = DMA_PRIORITY_LOW;
        dma_init_struct.loop_mode_enable = TRUE;    // operate as a circular buffer, no interrupt handling required

        dma_init_struct.memory_base_addr = (uint32_t)&(adcConversionBuffer[dmaBufferIndex]);
        dma_init_struct.peripheral_base_addr = (uint32_t)&(adc->ADCx->odt);

        xDMA_Init(dmaSpec->ref, &dma_init_struct);
        dmaMuxEnable(dmaIdentifier, dmaSpec->dmaMuxId);

        xDMA_Cmd(dmaSpec->ref,TRUE);    // means dma_channel_enable

        adc_dma_mode_enable(adc->ADCx, TRUE);
        adc_dma_request_repeat_enable(adc->ADCx, TRUE);

        #endif //end of USE_DMA_SPEC

        // set each channel into the auto sequence for this ADC device
        for (int adcChan = 0; adcChan < ADC_CHANNEL_COUNT; adcChan++)
        {
            // only add enabled channels for the current dev (can be simplified if we drop the pretense at handling adc2 and 3)
            if (adcOperatingConfig[adcChan].enabled && adcOperatingConfig[adcChan].adcDevice == dev)
            {
                adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;
                adc_ordinary_channel_set(adc->ADCx,
                    adcOperatingConfig[adcChan].adcChannel,
                    adcOperatingConfig[adcChan].dmaIndex+1, // This is the sequence number for the adc conversion
                    adcOperatingConfig[adcChan].sampleTime );
            }
        }

        adc_enable(adc->ADCx, TRUE);
        while (adc_flag_get(adc->ADCx, ADC_RDY_FLAG) == RESET); // wait for ready

        adc_calibration_init(adc->ADCx);
        while (adc_calibration_init_status_get(adc->ADCx));
        adc_calibration_start(adc->ADCx);
        while (adc_calibration_status_get(adc->ADCx));

        adc_ordinary_software_trigger_enable(adc->ADCx, TRUE);  // start sampling
    }
}

/**
 * Copies the latest ADC external channel data into adcValues defined in adc.c
*/
void adcGetChannelValues(void)
{
    for (int i = 0; i < ADC_CHANNEL_INTERNAL_FIRST_ID; i++) {
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
 * Reads a given channel from the DMA buffer
*/
uint16_t adcInternalRead(int channel)
{
    const int dmaIndex = adcOperatingConfig[channel].dmaIndex;
    return adcConversionBuffer[dmaIndex];
}

/**
 * Read the internal Vref and return raw value
 * 
 * The internal Vref is 1.2V and can be used to calculate the external Vref+
 * External Vref+ determines the scale for the raw ADC readings but since it
 * is often directly connected to Vdd (approx 3.3V) it isn't accurately controlled.
 * Calculating the actual value of Vref+ by using measurements of the known 1.2V
 * internal reference can improve overall accuracy.
 * 
 * @return the raw ADC reading for the internal voltage reference
 * @see adcInternalCompensateVref in src/main/drivers/adc.c
*/
uint16_t adcInternalReadVrefint(void)
{
    const uint16_t value = adcInternalRead(ADC_VREFINT);

    return value;
}

/**
 * Read the internal temperature sensor
 * 
 * @return the raw ADC reading
*/
uint16_t adcInternalReadTempsensor(void)
{
    const uint16_t value = adcInternalRead(ADC_TEMPSENSOR);
    return value;
}

#endif // USE_ADC_INTERNAL
#endif // USE_ADC
