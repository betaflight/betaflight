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

#pragma once

#include "platform.h"
#include "drivers/adc.h"
#include "drivers/dma.h"
#include "drivers/io_types.h"

#if PLATFORM_TRAIT_RCC
#include "platform/rcc_types.h"
#endif

#if defined(STM32F4) || defined(STM32F7)
#define ADC_TAG_MAP_COUNT 16
#elif defined(AT32F435)
#ifdef USE_ADC_INTERNAL
#define ADC_TAG_MAP_COUNT 18
#else
#define ADC_TAG_MAP_COUNT 16
#endif
#elif defined(STM32H7)
#ifdef USE_ADC_INTERNAL
#define ADC_TAG_MAP_COUNT 30
#else
#define ADC_TAG_MAP_COUNT 28
#endif
#elif defined(STM32G4)
#ifdef USE_ADC_INTERNAL
#define ADC_TAG_MAP_COUNT 49
#else
#define ADC_TAG_MAP_COUNT 47
#endif
#elif defined(APM32F4)
#define ADC_TAG_MAP_COUNT 16
#else
#define ADC_TAG_MAP_COUNT 10
#endif

typedef struct adcTagMap_s {
    ioTag_t tag;
    uint8_t devices;
    uint32_t channel;
#if defined(STM32H7) || defined(STM32G4) || defined(AT32F435)
    uint8_t channelOrdinal;
#endif
} adcTagMap_t;

// Encoding for adcTagMap_t.devices

#define ADC_DEVICES_1   (1 << ADCDEV_1)
#define ADC_DEVICES_2   (1 << ADCDEV_2)
#define ADC_DEVICES_3   (1 << ADCDEV_3)
#define ADC_DEVICES_4   (1 << ADCDEV_4)
#define ADC_DEVICES_5   (1 << ADCDEV_5)
#define ADC_DEVICES_12  ((1 << ADCDEV_1)|(1 << ADCDEV_2))
#define ADC_DEVICES_34  ((1 << ADCDEV_3)|(1 << ADCDEV_4))
#define ADC_DEVICES_123 ((1 << ADCDEV_1)|(1 << ADCDEV_2)|(1 << ADCDEV_3))
#define ADC_DEVICES_345 ((1 << ADCDEV_3)|(1 << ADCDEV_4)|(1 << ADCDEV_5))

typedef struct adcDevice_s {
    ADC_TypeDef* ADCx;
#if PLATFORM_TRAIT_RCC
    rccPeriphTag_t rccADC;
#endif
#if !defined(USE_DMA_SPEC)
    dmaResource_t* dmaResource;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(APM32F4)
    uint32_t channel;
#endif
#endif // !defined(USE_DMA_SPEC)
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(APM32F4)
    ADC_HandleTypeDef ADCHandle;
    DMA_HandleTypeDef DmaHandle;
#endif
#if defined(STM32H7) || defined(STM32G4)
    uint8_t irq;
    uint32_t channelBits;
#endif
} adcDevice_t;

#ifndef ADC_INSTANCE
#define ADC_INSTANCE                ADC1
#endif

typedef struct adcOperatingConfig_s {
    uint32_t adcChannel;        // Channel number for this input. Note that H7 and G4 HAL requires this to be 32-bit encoded number.
    ioTag_t tag;
    uint8_t dmaIndex;           // index into DMA buffer in case of sparse channels
    uint8_t sampleTime;
    bool enabled;
#if PLATFORM_TRAIT_ADC_DEVICE
    adcDevice_e adcDevice;      // ADCDEV_x for this input
#endif
} adcOperatingConfig_t;

extern adcOperatingConfig_t adcOperatingConfig[ADC_SOURCE_COUNT];
extern volatile DMA_DATA_ZERO_INIT uint16_t adcValues[ADC_SOURCE_COUNT];

#define ADC_CFG_TO_DEV(x) ((x) - 1)
#define ADC_DEV_TO_CFG(x) ((x) + 1)

extern const adcDevice_t adcHardware[];
extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];

#ifdef USE_ADC_INTERNAL
extern int32_t adcVREFINTCAL; // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
extern int32_t adcTSCAL1;
extern int32_t adcTSCAL2;
extern int32_t adcTSSlopeK;

uint16_t adcInternalRead(adcSource_e source);
#endif

uint32_t adcChannelByTag(ioTag_t ioTag);
#if PLATFORM_TRAIT_ADC_DEVICE
adcDevice_e adcDeviceByInstance(const ADC_TypeDef *instance);
bool adcVerifyPin(ioTag_t tag, adcDevice_e device);
#endif

// Marshall values in DMA instance/channel based order to adcChannel based order.
// Required for multi DMA instance implementation
void adcGetChannelValues(void);

//
// VREFINT and TEMPSENSOR related definitions
// These are shared among common adc.c and MCU dependent adc_stm32XXX.c
//
#ifdef STM32F7
// STM32F7 HAL library V1.12.0 defines VREFINT and TEMPSENSOR in stm32f7xx_ll_adc.h,
// which is not included from stm32f7xx_hal_adc.h
// We manually copy required lines here.
// XXX V1.14.0 may solve this problem

#define VREFINT_CAL_VREF                   ( 3300U)                    /* Analog voltage reference (Vref+) value with which temperature sensor has been calibrated in production (tolerance: +-10 mV) (unit: mV). */
#define TEMPSENSOR_CAL1_TEMP               (( int32_t)   30)           /* Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL1_ADDR (tolerance: +-5 DegC) (unit: DegC). */
#define TEMPSENSOR_CAL2_TEMP               (( int32_t)  110)           /* Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL2_ADDR (tolerance: +-5 DegC) (unit: DegC). */
#define TEMPSENSOR_CAL_VREFANALOG          ( 3300U)                    /* Analog voltage reference (Vref+) voltage with which temperature sensor has been calibrated in production (+-10 mV) (unit: mV). */

// These addresses are incorrectly defined in stm32f7xx_ll_adc.h
#if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx)
// F745xx_F746xx and  F765xx_F767xx_F769xx
#define VREFINT_CAL_ADDR                   ((uint16_t*) (0x1FF0F44A))
#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) (0x1FF0F44C))
#define TEMPSENSOR_CAL2_ADDR               ((uint16_t*) (0x1FF0F44E))
#elif defined(STM32F722xx)
// F72x_F73x
#define VREFINT_CAL_ADDR                   ((uint16_t*) (0x1FF07A2A))
#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) (0x1FF07A2C))
#define TEMPSENSOR_CAL2_ADDR               ((uint16_t*) (0x1FF07A2E))
#endif
#endif // STM32F7

#ifdef STM32F4
// STM32F4 stdlib does not define any of these
#define VREFINT_CAL_VREF                   (3300U)
#define TEMPSENSOR_CAL_VREFANALOG          (3300U)
#define TEMPSENSOR_CAL1_TEMP               ((int32_t)  30)
#define TEMPSENSOR_CAL2_TEMP               ((int32_t) 110)
#endif

#ifdef AT32F435
#define VREFINT_EXPECTED                   (1489U)  // The raw ADC reading at 12bit resolution expected for the 1V2 internal ref
#define VREFINT_CAL_VREF                   (3300U)  // The nominal external Vref+ for the above reading
#define TEMPSENSOR_CAL_VREFANALOG          (3300U)
#define TEMPSENSOR_CAL1_TEMP               (25U)
#define TEMPSENSOR_CAL1_V                  (1.27f)
#define TEMPSENSOR_SLOPE                   (-4.13f /* mV/C */)
#endif
