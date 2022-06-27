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

#include "drivers/adc.h"
#include "drivers/dma.h"
#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

#if defined(STM32F4) || defined(STM32F7)
#define ADC_TAG_MAP_COUNT 16
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
#else
#define ADC_TAG_MAP_COUNT 10
#endif

typedef struct adcTagMap_s {
    ioTag_t tag;
    uint8_t devices;
    uint32_t channel;
#if defined(STM32H7) || defined(STM32G4)
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
    rccPeriphTag_t rccADC;
#if !defined(USE_DMA_SPEC)
    dmaResource_t* dmaResource;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    uint32_t channel;
#endif
#endif // !defined(USE_DMA_SPEC)
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    ADC_HandleTypeDef ADCHandle;
    DMA_HandleTypeDef DmaHandle;
#endif
#if defined(STM32H7) || defined(STM32G4)
    uint8_t irq;
    uint32_t channelBits;
#endif
} adcDevice_t;

#ifdef USE_ADC_INTERNAL
extern int32_t adcVREFINTCAL;      // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
extern int32_t adcTSCAL1;
extern int32_t adcTSCAL2;
extern int32_t adcTSSlopeK;
#endif

extern const adcDevice_t adcHardware[];
extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];
extern adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

uint8_t adcChannelByTag(ioTag_t ioTag);
ADCDevice adcDeviceByInstance(ADC_TypeDef *instance);
bool adcVerifyPin(ioTag_t tag, ADCDevice device);

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
