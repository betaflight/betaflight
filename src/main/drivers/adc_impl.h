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
#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

#if defined(STM32F4) || defined(STM32F7)
#define ADC_TAG_MAP_COUNT 16
#elif defined(STM32F3)
#define ADC_TAG_MAP_COUNT 39
#else
#define ADC_TAG_MAP_COUNT 10
#endif

typedef struct adcTagMap_s {
    ioTag_t tag;
#if !defined(STM32F1) // F1 pins have uniform connection to ADC instances
    uint8_t devices;
#endif
    uint8_t channel;
} adcTagMap_t;

// Encoding for adcTagMap_t.devices

#define ADC_DEVICES_1   (1 << ADCDEV_1)
#define ADC_DEVICES_2   (1 << ADCDEV_2)
#define ADC_DEVICES_3   (1 << ADCDEV_3)
#define ADC_DEVICES_4   (1 << ADCDEV_4)
#define ADC_DEVICES_12  ((1 << ADCDEV_1)|(1 << ADCDEV_2))
#define ADC_DEVICES_34  ((1 << ADCDEV_3)|(1 << ADCDEV_4))
#define ADC_DEVICES_123 ((1 << ADCDEV_1)|(1 << ADCDEV_2)|(1 << ADCDEV_3))

typedef struct adcDevice_s {
    ADC_TypeDef* ADCx;
    rccPeriphTag_t rccADC;
#if !defined(USE_DMA_SPEC)
#if defined(STM32F4) || defined(STM32F7)
    DMA_Stream_TypeDef* DMAy_Streamx;
    uint32_t channel;
#else
    DMA_Channel_TypeDef* DMAy_Channelx;
#endif
#endif
#if defined(STM32F7)
    ADC_HandleTypeDef ADCHandle;
    DMA_HandleTypeDef DmaHandle;
#endif
} adcDevice_t;

extern const adcDevice_t adcHardware[];
extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];
extern adcOperatingConfig_t adcOperatingConfig[ADC_CHANNEL_COUNT];
extern volatile uint16_t adcValues[ADC_CHANNEL_COUNT];

uint8_t adcChannelByTag(ioTag_t ioTag);
ADCDevice adcDeviceByInstance(ADC_TypeDef *instance);
bool adcVerifyPin(ioTag_t tag, ADCDevice device);
