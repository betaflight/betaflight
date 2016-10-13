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
    ADC_CHANNEL0_ENABLE = (1 << 0),
    ADC_CHANNEL1_ENABLE = (1 << 1),
    ADC_CHANNEL2_ENABLE = (1 << 2),
    ADC_CHANNEL3_ENABLE = (1 << 3),
    ADC_CHANNEL4_ENABLE = (1 << 4),
    ADC_CHANNEL5_ENABLE = (1 << 5),
} adcChannelEnableMask_e;

#ifndef ADC_CHANNEL_COUNT
#define ADC_CHANNEL_COUNT 4
#endif

typedef enum {
    ADC_CHANNEL0 = 0,
    ADC_CHANNEL1 = 1,
    ADC_CHANNEL2 = 2,
    ADC_CHANNEL3 = 3,
#if ADC_CHANNEL_COUNT > 4
    ADC_CHANNEL4 = 4,
#endif
#if ADC_CHANNEL_COUNT > 5
    ADC_CHANNEL5 = 5,
#endif
} adcChannelIndex_e;

#define ADC_CHANNEL_MASK(adcChannel) (1 << adcChannel)

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
