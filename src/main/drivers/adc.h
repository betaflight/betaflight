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

typedef enum {
    ADC_CHANNEL1_BIT = 0,
    ADC_CHANNEL2_BIT = 1,
    ADC_CHANNEL3_BIT = 2,
    ADC_CHANNEL4_BIT = 3,
} adcChannelBit_e;

typedef enum {
    ADC_CHANNEL1_ENABLE = (1 << ADC_CHANNEL1_BIT),
    ADC_CHANNEL2_ENABLE = (1 << ADC_CHANNEL2_BIT),
    ADC_CHANNEL3_ENABLE = (1 << ADC_CHANNEL3_BIT),
    ADC_CHANNEL4_ENABLE = (1 << ADC_CHANNEL4_BIT),
} adcChannelEnableMask_e;

typedef enum {
    ADC_CHANNEL_1 = 0,
    ADC_CHANNEL_2 = 1,
    ADC_CHANNEL_3 = 2,
    ADC_CHANNEL_4 = 3,

#ifdef OSD
    // OSD
    ADC_12V       = 0,
    ADC_5V        = 1,
    ADC_BATTERY   = 2,
    ADC_CURRENT   = 3,
#else
    // FC
    ADC_BATTERY   = 0,
    ADC_RSSI      = 1,
    ADC_EXTERNAL1 = 2,
    ADC_CURRENT   = 3,
#endif
    ADC_CHANNEL_MAX = 3
} adcChannelIndex_e;

#define ADC_CHANNEL_COUNT (ADC_CHANNEL_MAX + 1)

typedef struct adc_config_s {
    uint8_t adcChannel;         // ADC1_INxx channel number
    uint8_t dmaIndex;           // index into DMA buffer in case of sparse channels
    bool enabled;
    uint8_t sampleTime;
} adc_config_t;

typedef struct drv_adc_config_s {
    uint32_t channelMask;
} drv_adc_config_t;

void adcInit(drv_adc_config_t *init);
uint16_t adcGetChannel(uint8_t channel);
