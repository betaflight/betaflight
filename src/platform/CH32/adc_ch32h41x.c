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
 *  porting for ch32h41x by Temperslee
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/adc.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/sensor.h"
#include "platform/rcc.h"

#include "pg/adc.h"
#include "platform/adc_impl.h"
<<<<<<< HEAD
<<<<<<< HEAD
#include "platform/dma.h"

#include <math.h>
<<<<<<< HEAD

=======
>>>>>>> fd9c9cdce (target based cross-compiler)
=======
=======
#include "platform/dma.h"
>>>>>>> cdcdf52d6 (update CH32 drivers for rebase)

#include <math.h>

>>>>>>> ab93a2999 (WIP CH32 platform updates)
const adcDevice_t adcHardware[ADCDEV_COUNT] = {{.ADCx = ADC1,
                                                .rccADC = RCC_HB2(ADC1),
#if !defined(USE_DMA_SPEC)
                                                .dmaResource = NULL
#endif
                                               },
                                               {.ADCx = ADC2,
                                                .rccADC = RCC_HB2(ADC2),
#if !defined(USE_DMA_SPEC)
                                                .dmaResource = NULL
#endif
                                               }};

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
#ifdef USE_ADC_INTERNAL
#define ADC_TAG_MAP_VREFINT 0
#define ADC_TAG_MAP_TEMPSENSOR 1
    {DEFIO_TAG_E__NONE, ADC_DEVICES_1, ADC_Channel_Vrefint},
    {DEFIO_TAG_E__NONE, ADC_DEVICES_1, ADC_Channel_TempSensor},
#endif
    {DEFIO_TAG_E__PC0, ADC_DEVICES_12, ADC_Channel_10},
    {DEFIO_TAG_E__PC1, ADC_DEVICES_12, ADC_Channel_11},
    {DEFIO_TAG_E__PC2, ADC_DEVICES_12, ADC_Channel_12},
    {DEFIO_TAG_E__PC3, ADC_DEVICES_12, ADC_Channel_13},
    {DEFIO_TAG_E__PC4, ADC_DEVICES_12, ADC_Channel_14},
    {DEFIO_TAG_E__PA0, ADC_DEVICES_12, ADC_Channel_0},
    {DEFIO_TAG_E__PA1, ADC_DEVICES_12, ADC_Channel_1},
    {DEFIO_TAG_E__PA2, ADC_DEVICES_12, ADC_Channel_2},
    {DEFIO_TAG_E__PA3, ADC_DEVICES_12, ADC_Channel_3},
    {DEFIO_TAG_E__PA4, ADC_DEVICES_12, ADC_Channel_4},
    {DEFIO_TAG_E__PA5, ADC_DEVICES_12, ADC_Channel_5},
    {DEFIO_TAG_E__PA6, ADC_DEVICES_12, ADC_Channel_6},
    {DEFIO_TAG_E__PA7, ADC_DEVICES_12, ADC_Channel_7},
    {DEFIO_TAG_E__PB0, ADC_DEVICES_12, ADC_Channel_8},
    {DEFIO_TAG_E__PB1, ADC_DEVICES_12, ADC_Channel_9},
};

<<<<<<< HEAD
<<<<<<< HEAD
static volatile DMA_DATA uint32_t adcConversionBuffer[ADC_SOURCE_COUNT]; //ADC_SOURCE_COUNT ADC_CHANNEL_COUNT

static void adcInitDevice(const adcDevice_t *adcdev, int channelCount) {
  ADC_TypeDef *Instance = adcdev->ADCx;
  ADC_InitTypeDef ADC_InitStructure = {0};

  ADC_DeInit(Instance);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = channelCount;

=======
static volatile DMA_DATA uint32_t adcConversionBuffer[ADC_CHANNEL_COUNT];
=======
static volatile DMA_DATA uint32_t adcConversionBuffer[ADC_SOURCE_COUNT]; //ADC_SOURCE_COUNT ADC_CHANNEL_COUNT
>>>>>>> ab93a2999 (WIP CH32 platform updates)

static void adcInitDevice(const adcDevice_t *adcdev, int channelCount) {
  ADC_TypeDef *Instance = adcdev->ADCx;
  ADC_InitTypeDef ADC_InitStructure = {0};

  ADC_DeInit(Instance);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = channelCount;

>>>>>>> fd9c9cdce (target based cross-compiler)
  // Multiple injected channel seems to require scan conversion mode to be
  // enabled even if main (non-injected) channel count is 1.
#ifdef USE_ADC_INTERNAL
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
#else
  ADC_InitStructure.ADC_ScanConvMode =
      channelCount > 1 ? ENABLE
                       : DISABLE; // 1=scan more that one channel in group
#endif
  ADC_Init(Instance, &ADC_InitStructure);
}

/**
 * Find a given pin (defined by ioTag) in the map
 *
 * @param tag the ioTag to search for
 * @return the index in adcTagMap corresponding to the given ioTag or -1 if not
 * found
 */
static int adcFindTagMapEntry(const ioTag_t tag) {
  for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
    if (adcTagMap[i].tag == tag) {
      return i;
    }
  }
  return -1;
}

static void setScalingFactors(void) {
  // The expected reading for 1.2V internal reference if external vref+ was 3.3V
  adcVREFINTCAL = VREFINT_EXPECTED;

  // adcTSCAL1 needs to be the raw ADC value at 25C, but we can't adjust it for
  // VREF+ because we haven't calculated it yet So this will have to be
  // aproximate (and the use should be fixed at some point)
  adcTSCAL1 = (TEMPSENSOR_CAL1_V * 4095.0f) / 3.3f;

  // TEMPSENSOR_SLOPE is given in mv/C.
  // adcTSSlopeK has the opposite sign compared to the CH32 standard because
  // adcInternalComputeTemperature subtracts the calibration value from the
  // reading where as the CH32 docs subtract the reading from the calibration.
  // 3300/4095 converts the reading to mV and the factor of 1000 is needed to
  // prevent the slope from rounding to 0. The intermediate result when this is
  // used is in mC, and adcInternalComputeTemperature then divides by 1000 to
  // get an answer in C
  adcTSSlopeK = lrintf(-3300.0f * 1000.0f / 4095.0f / TEMPSENSOR_SLOPE);
}

void adcInit(const adcConfig_t *config) {
  uint32_t nChannelsUsed[ADCDEV_COUNT] = {0};

  setScalingFactors();

  memset(
      adcOperatingConfig, 0,
      sizeof(
          adcOperatingConfig)); // adcOperatingConfig defined in generic adc.c

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

  // loop over all possible channels and build the adcOperatingConfig to
  // represent the set of enabled channels
  for (int i = 0; i < ADC_SOURCE_COUNT; i++) {
    int map;
    int dev;

    switch (i){
      #ifdef USE_ADC_INTERNAL
        case ADC_TEMPSENSOR:
          map = ADC_TAG_MAP_TEMPSENSOR;
          dev = ADCDEV_1;
          break;
        case ADC_VREFINT:
          map = ADC_TAG_MAP_VREFINT;
          dev = ADCDEV_1;
          break;
      #endif

      default:
        if (!adcOperatingConfig[i].tag) {
                continue;
          }

        map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
        if (map < 0) {
          continue;
        }
      // Since ADC1 can do all channels this will only ever return adc1 and is
      // unnecessary

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
    adcOperatingConfig[i].sampleTime = ADC_SampleTime_CyclesMode7;
    adcOperatingConfig[i].enabled = true;

    nChannelsUsed[dev] += 1; // increase the active channel count for this device

    // Enable the gpio for analog input
    if (adcOperatingConfig[i].tag) {
      IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
      IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag),
                   IO_CONFIG(DIR_IN, GPIO_MODE_IN_AN, GPIO_SPEED_VERY_HIGH,
                             GPIO_PULL_NONE));
    }
  } // for each channel

  // reset all ADC instances，before init we do this;
  // ADC_DeInit(ADC1);
  // ADC_DeInit(ADC2);

  // Common config applies equally to all two ADCs, so only apply it once
  RCC_ADCCLKConfig(RCC_ADCCLKSource_HCLK);
  RCC_ADCHCLKCLKAsSourceConfig(RCC_PPRE2_DIV0, RCC_HCLK_ADCPRE_DIV8);

  // Enable the clock (power up the circuit)
  // This has to be done before calling adc_common_config for reasons that are
  // unclear NB only ADC1 will have any channels, so this loop for pedantry only
  for (int i = 0; i < ADCDEV_COUNT; i++) {
    if (nChannelsUsed[i] > 0) {
      RCC_ClockCmd(adcHardware[0].rccADC, ENABLE);
    }
  }

  // Only adc1 will have any channels assigned to it after the code above, so
  // this can be simplified

  int dmaBufferIndex = 0;
  for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
    const adcDevice_t *adc = &adcHardware[dev];

    // skip this device if it doesn't have any active channels
    if (nChannelsUsed[dev] == 0) {
      continue;
    }

    adcInitDevice(adc, nChannelsUsed[dev]);

#ifdef USE_DMA_SPEC

    // Setup the DMA channel so that data is automatically and continuously
    // transferred from the ADC output register to the results buffer

    const dmaChannelSpec_t *dmaSpec =
        dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
    if (dmaSpec == NULL) {
      return;
    }

    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);
    if (!dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
      return;
    }

    dmaEnable(dmaIdentifier); // enables clock/power
    xDMA_DeInit(dmaSpec->ref);

    DMA_InitTypeDef DMA_InitStructure = {0};

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(adc->ADCx->RDATAR);
    DMA_InitStructure.DMA_Memory0BaseAddr =
        (uint32_t)&(adcConversionBuffer[dmaBufferIndex]);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = nChannelsUsed[dev];
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    xDMA_Init(dmaSpec->ref, &DMA_InitStructure);
    dmaMuxEnable(dmaIdentifier, dmaSpec->dmaMuxId);

    xDMA_Cmd(dmaSpec->ref, ENABLE); // means dma_channel_enable
    ADC_DMACmd(adc->ADCx, ENABLE);

#endif // end of USE_DMA_SPEC

    // set each channel into the auto sequence for this ADC device
    for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {
      // only add enabled channels for the current dev (can be simplified if we
      // drop the pretense at handling adc2 and 3)
      if (adcOperatingConfig[adcChan].enabled &&
          adcOperatingConfig[adcChan].adcDevice == dev) {
        adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;
        ADC_RegularChannelConfig(
            adc->ADCx, adcOperatingConfig[adcChan].adcChannel,
            adcOperatingConfig[adcChan].dmaIndex +
                1, // This is the sequence number for the adc conversion
            adcOperatingConfig[adcChan].sampleTime);
      }
    }

    adc->ADCx->CTLR1 |= (1 << 31);
    ADC_Cmd(adc->ADCx, ENABLE);
    ADC_BufferCmd(adc->ADCx, ENABLE);

    // ADC_ResetCalibration(adc->ADCx);
    // while(ADC_GetResetCalibrationStatus(adc->ADCx));
    // ADC_StartCalibration(adc->ADCx);
    // while(ADC_GetCalibrationStatus(adc->ADCx));

    // RCC_ADCHCLKCLKAsSourceConfig(RCC_PPRE2_DIV0,RCC_HCLK_ADCPRE_DIV8);
    ADC_LowPowerModeCmd(adc->ADCx, DISABLE);
#ifdef USE_ADC_INTERNAL
    ADC_TempSensorVrefintCmd(ENABLE); // only ADC1
#endif
<<<<<<< HEAD
        ADC_SoftwareStartConvCmd(adc->ADCx, ENABLE);  // start sampling
    }
  } // for each channel

  // reset all ADC instances，before init we do this;
  // ADC_DeInit(ADC1);
  // ADC_DeInit(ADC2);

  // Common config applies equally to all two ADCs, so only apply it once
  RCC_ADCCLKConfig(RCC_ADCCLKSource_HCLK);
  RCC_ADCHCLKCLKAsSourceConfig(RCC_PPRE2_DIV0, RCC_HCLK_ADCPRE_DIV8);

  // Enable the clock (power up the circuit)
  // This has to be done before calling adc_common_config for reasons that are
  // unclear NB only ADC1 will have any channels, so this loop for pedantry only
  for (int i = 0; i < ADCDEV_COUNT; i++) {
    if (nChannelsUsed[i] > 0) {
      RCC_ClockCmd(adcHardware[0].rccADC, ENABLE);
    }
  }

  // Only adc1 will have any channels assigned to it after the code above, so
  // this can be simplified

  int dmaBufferIndex = 0;
  for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
    const adcDevice_t *adc = &adcHardware[dev];

    // skip this device if it doesn't have any active channels
    if (nChannelsUsed[dev] == 0) {
      continue;
    }

    adcInitDevice(adc, nChannelsUsed[dev]);

#ifdef USE_DMA_SPEC

    // Setup the DMA channel so that data is automatically and continuously
    // transferred from the ADC output register to the results buffer

    const dmaChannelSpec_t *dmaSpec =
        dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);
    if (dmaSpec == NULL) {
      return;
    }

    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);
    if (!dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
      return;
    }

    dmaEnable(dmaIdentifier); // enables clock/power
    xDMA_DeInit(dmaSpec->ref);

    DMA_InitTypeDef DMA_InitStructure = {0};

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(adc->ADCx->RDATAR);
    DMA_InitStructure.DMA_Memory0BaseAddr =
        (uint32_t)&(adcConversionBuffer[dmaBufferIndex]);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = nChannelsUsed[dev];
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    xDMA_Init(dmaSpec->ref, &DMA_InitStructure);
    dmaMuxEnable(dmaIdentifier, dmaSpec->dmaMuxId);

    xDMA_Cmd(dmaSpec->ref, ENABLE); // means dma_channel_enable
    ADC_DMACmd(adc->ADCx, ENABLE);

#endif // end of USE_DMA_SPEC

    // set each channel into the auto sequence for this ADC device
    for (int adcChan = 0; adcChan < ADC_SOURCE_COUNT; adcChan++) {
      // only add enabled channels for the current dev (can be simplified if we
      // drop the pretense at handling adc2 and 3)
      if (adcOperatingConfig[adcChan].enabled &&
          adcOperatingConfig[adcChan].adcDevice == dev) {
        adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;
        ADC_RegularChannelConfig(
            adc->ADCx, adcOperatingConfig[adcChan].adcChannel,
            adcOperatingConfig[adcChan].dmaIndex +
                1, // This is the sequence number for the adc conversion
            adcOperatingConfig[adcChan].sampleTime);
      }
    }

    adc->ADCx->CTLR1 |= (1 << 31);
    ADC_Cmd(adc->ADCx, ENABLE);
    ADC_BufferCmd(adc->ADCx, ENABLE);

    // ADC_ResetCalibration(adc->ADCx);
    // while(ADC_GetResetCalibrationStatus(adc->ADCx));
    // ADC_StartCalibration(adc->ADCx);
    // while(ADC_GetCalibrationStatus(adc->ADCx));

    // RCC_ADCHCLKCLKAsSourceConfig(RCC_PPRE2_DIV0,RCC_HCLK_ADCPRE_DIV8);
    ADC_LowPowerModeCmd(adc->ADCx, DISABLE);
#ifdef USE_ADC_INTERNAL
    ADC_TempSensorVrefintCmd(ENABLE); // only ADC1
#endif
=======
>>>>>>> fd9c9cdce (target based cross-compiler)
    ADC_SoftwareStartConvCmd(adc->ADCx, ENABLE); // start sampling
  }
}

/**
 * Copies the latest ADC external channel data into adcValues defined in adc.c
 */
void adcGetChannelValues(void) {
<<<<<<< HEAD
<<<<<<< HEAD
  for (unsigned i = 0; i < ADC_EXTERNAL_COUNT; i++) {
=======
  for (int i = 0; i < ADC_CHANNEL_INTERNAL_FIRST_ID; i++) {
>>>>>>> fd9c9cdce (target based cross-compiler)
=======
  for (unsigned i = 0; i < ADC_EXTERNAL_COUNT; i++) {
>>>>>>> ab93a2999 (WIP CH32 platform updates)
    if (adcOperatingConfig[i].enabled) {
      adcValues[adcOperatingConfig[i].dmaIndex] =
          adcConversionBuffer[adcOperatingConfig[i].dmaIndex];
    }
  }
}

#ifdef USE_ADC_INTERNAL

/**
 * This impl is never busy in the sense of being unable to supply data
 */
bool adcInternalIsBusy(void) { return false; }

/**
 * Nop, since the ADC is free running
 */
void adcInternalStartConversion(void) { return; }

/**
 * Reads a given channel from the DMA buffer
 */
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ab93a2999 (WIP CH32 platform updates)
uint16_t adcInternalRead(adcSource_e source) {
  // const int dmaIndex = adcOperatingConfig[channel].dmaIndex;
  // return adcConversionBuffer[dmaIndex];
    switch (source) {
    case ADC_VREFINT:
    case ADC_TEMPSENSOR:
        const unsigned dmaIndex = adcOperatingConfig[source].dmaIndex;
        return dmaIndex < ARRAYLEN(adcConversionBuffer) ? adcConversionBuffer[dmaIndex] : 0;
    default:
        return 0;
    }
<<<<<<< HEAD
=======
static uint16_t adcInternalRead(int channel) {
  const int dmaIndex = adcOperatingConfig[channel].dmaIndex;
  return adcConversionBuffer[dmaIndex];
>>>>>>> fd9c9cdce (target based cross-compiler)
=======
>>>>>>> ab93a2999 (WIP CH32 platform updates)
}

/**
 * Read the internal Vref and return raw value
 *
 * The internal Vref is 1.2V and can be used to calculate the external Vref+
 * External Vref+ determines the scale for the raw ADC readings but since it
 * is often directly connected to Vdd (approx 3.3V) it isn't accurately
 * controlled. Calculating the actual value of Vref+ by using measurements of
 * the known 1.2V internal reference can improve overall accuracy.
 *
 * @return the raw ADC reading for the internal voltage reference
 * @see adcInternalCompensateVref in src/main/drivers/adc.c
 */
<<<<<<< HEAD
<<<<<<< HEAD
// uint16_t adcInternalReadVrefint(void) {
//   const uint16_t value = adcInternalRead(ADC_VREFINT);

//   return value;
// }
=======
uint16_t adcInternalReadVrefint(void) {
  const uint16_t value = adcInternalRead(ADC_VREFINT);

  return value;
}
>>>>>>> fd9c9cdce (target based cross-compiler)
=======
// uint16_t adcInternalReadVrefint(void) {
//   const uint16_t value = adcInternalRead(ADC_VREFINT);

//   return value;
// }
>>>>>>> ab93a2999 (WIP CH32 platform updates)

/**
 * Read the internal temperature sensor
 *
 * @return the raw ADC reading
 */
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ab93a2999 (WIP CH32 platform updates)
// uint16_t adcInternalReadTempsensor(void) {
//   const uint16_t value = adcInternalRead(ADC_TEMPSENSOR);
//   return value;
// }
<<<<<<< HEAD
=======
uint16_t adcInternalReadTempsensor(void) {
  const uint16_t value = adcInternalRead(ADC_TEMPSENSOR);
  return value;
}
>>>>>>> fd9c9cdce (target based cross-compiler)
=======
>>>>>>> ab93a2999 (WIP CH32 platform updates)

#endif // USE_ADC_INTERNAL

#endif // USE_ADC
