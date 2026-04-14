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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/rcc.h"
#include "drivers/resource.h"
#include "drivers/dma.h"

#include "drivers/sensor.h"

#include "drivers/adc.h"
#include "platform/adc_impl.h"

#include "pg/adc.h"

#ifndef ADC1_DMA_STREAM
#define ADC1_DMA_STREAM NULL
#endif
#ifndef ADC2_DMA_STREAM
#define ADC2_DMA_STREAM NULL
#endif

const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_AHB2(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = DMA_REQUEST_ADC1,
#endif
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_AHB2(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = DMA_REQUEST_ADC2,
#endif
    },
};

#endif
