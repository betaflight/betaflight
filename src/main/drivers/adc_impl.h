/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/io_types.h"
#include "rcc_types.h"

#if defined(STM32F4) || defined(STM32F7)
#define ADC_TAG_MAP_COUNT 16
#elif defined(STM32F3)
#define ADC_TAG_MAP_COUNT 39
#else
#define ADC_TAG_MAP_COUNT 10
#endif

typedef enum ADCDevice {
    ADCINVALID = -1,
    ADCDEV_1   = 0,
#if defined(STM32F3)
    ADCDEV_2,
    ADCDEV_MAX = ADCDEV_2,
#elif defined(STM32F4) || defined(STM32F7)
    ADCDEV_2,
    ADCDEV_3,
    ADCDEV_MAX = ADCDEV_3,
#else
    ADCDEV_MAX = ADCDEV_1,
#endif
    ADCDEV_COUNT = ADCDEV_MAX + 1
} ADCDevice;

typedef struct adcTagMap_s {
    ioTag_t tag;
    uint8_t channel;
} adcTagMap_t;

typedef struct adcDevice_s {
    ADC_TypeDef* ADCx;
    rccPeriphTag_t rccADC;
    rccPeriphTag_t rccDMA;
#if defined(STM32F4) || defined(STM32F7)
    DMA_Stream_TypeDef* DMAy_Streamx;
    uint32_t channel;
#else
    DMA_Channel_TypeDef* DMAy_Channelx;
#endif
#if defined(STM32F7)
    ADC_HandleTypeDef ADCHandle;
    DMA_HandleTypeDef DmaHandle;
#endif
    bool enabled;
    int usedChannelCount;
} adcDevice_t;

typedef struct adc_config_s {
    ioTag_t tag;
    ADCDevice adcDevice;
    uint8_t adcChannel;         // ADC1_INxx channel number
    uint8_t dmaIndex;           // index into DMA buffer in case of sparse channels
    bool enabled;
    uint8_t sampleTime;
} adc_config_t;

extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];
extern adc_config_t adcConfig[ADC_CHN_COUNT];
extern volatile uint16_t adcValues[ADCDEV_COUNT][ADC_CHN_COUNT];

void adcHardwareInit(drv_adc_config_t *init);
ADCDevice adcDeviceByInstance(ADC_TypeDef *instance);
uint8_t adcChannelByTag(ioTag_t ioTag);
