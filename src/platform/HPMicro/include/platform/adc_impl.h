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

// HPMicro ADC implementation types: tag-to-channel map, ADC device list and
// operating configuration shared with the ADC driver.

#pragma once

#include "platform.h"
#include "drivers/adc.h"
#include "drivers/io_types.h"

#ifdef HPM6750
#include "hpm_adc12_drv.h"
#else
#include "hpm_adc16_drv.h"
#endif

#define ADC_TAG_MAP_COUNT 24

typedef struct adcTagMap_s {
    ioTag_t tag;
    uint8_t devices;
    uint32_t channel;
} adcTagMap_t;

#define ADC_DEVICES_1   (1 << ADCDEV_1)
#define ADC_DEVICES_2   (1 << ADCDEV_2)
#define ADC_DEVICES_3   (1 << ADCDEV_3)
#define ADC_DEVICES_4   (1 << ADCDEV_4)
#define ADC_DEVICES_5   (1 << ADCDEV_5)
#define ADC_DEVICES_12  ((1 << ADCDEV_1)|(1 << ADCDEV_2))
#define ADC_DEVICES_23  ((1 << ADCDEV_2)|(1 << ADCDEV_3))
#define ADC_DEVICES_34  ((1 << ADCDEV_3)|(1 << ADCDEV_4))
#define ADC_DEVICES_123 ((1 << ADCDEV_1)|(1 << ADCDEV_2)|(1 << ADCDEV_3))

typedef struct adcDevice_s {
    ADC_TypeDef *ADCx;
    uint32_t rccADC;
    uint32_t channelBits;
} adcDevice_t;

#ifndef ADC_INSTANCE
#define ADC_INSTANCE HPM_ADC0
#endif

typedef struct adcOperatingConfig_s {
    uint32_t adcChannel;
    ioTag_t tag;
    uint8_t dmaIndex;
    uint8_t sampleTime;
    bool enabled;
    adcDevice_e adcDevice;
} adcOperatingConfig_t;

extern adcOperatingConfig_t adcOperatingConfig[ADC_SOURCE_COUNT];
extern volatile DMA_DATA_ZERO_INIT uint16_t adcValues[ADC_SOURCE_COUNT];

#define ADC_CFG_TO_DEV(x) ((x) - 1)
#define ADC_DEV_TO_CFG(x) ((x) + 1)

extern const adcDevice_t adcHardware[];
extern const adcTagMap_t adcTagMap[ADC_TAG_MAP_COUNT];

void adcGetChannelValues(void);
