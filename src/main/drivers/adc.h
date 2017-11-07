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

#ifndef ADC_INSTANCE
#define ADC_INSTANCE                ADC1
#endif

#if defined(STM32F4) || defined(STM32F7)
#ifndef ADC1_DMA_STREAM
#define ADC1_DMA_STREAM DMA2_Stream4 // ST0 or ST4
#endif

#ifndef ADC2_DMA_STREAM
#define ADC2_DMA_STREAM DMA2_Stream3 // ST2 or ST3
#endif

#ifndef ADC3_DMA_STREAM
#define ADC3_DMA_STREAM DMA2_Stream0 // ST0 or ST1
#endif
#endif

typedef enum ADCDevice {
    ADCINVALID = -1,
    ADCDEV_1   = 0,
#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
    ADCDEV_2,
    ADCDEV_3,
#endif
#if defined(STM32F3)
    ADCDEV_4,
#endif
    ADCDEV_COUNT
} ADCDevice;

#define ADC_CFG_TO_DEV(x) ((x) - 1)
#define ADC_DEV_TO_CFG(x) ((x) + 1)

typedef enum {
    ADC_BATTERY = 0,
    ADC_CURRENT = 1,
    ADC_EXTERNAL1 = 2,
    ADC_RSSI = 3,
    ADC_CHANNEL_COUNT
} AdcChannel;

typedef struct adc_config_s {
    ioTag_t tag;
    uint8_t adcChannel;         // ADC1_INxx channel number
    uint8_t dmaIndex;           // index into DMA buffer in case of sparse channels
    bool enabled;
    uint8_t sampleTime;
} adcOperatingConfig_t;

typedef struct adcChannelConfig_t {
    bool enabled;
    ioTag_t ioTag;
} adcChannelConfig_t;

typedef struct adcConfig_s {
    adcChannelConfig_t vbat;
    adcChannelConfig_t rssi;
    adcChannelConfig_t current;
    adcChannelConfig_t external1;
    int8_t device; // ADCDevice
} adcConfig_t;

void adcInit(const adcConfig_t *config);
uint16_t adcGetChannel(uint8_t channel);

#ifndef SITL
ADCDevice adcDeviceByInstance(ADC_TypeDef *instance);
#endif
