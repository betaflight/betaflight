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

#include <stdbool.h>

#include "platform.h"
#include "drivers/io_types.h"
#include "drivers/time.h"

#if PLATFORM_TRAIT_ADC_DEVICE
typedef enum {
    ADCINVALID = -1,
#if defined(USE_ADC_DEVICE_0)
    ADCDEV_0   = 0,
#if defined(ADC1)
    ADCDEV_1,
#endif
#else
    ADCDEV_1   = 0,
#endif
#if defined(ADC2)
    ADCDEV_2,
#endif
#if defined(ADC3)
    ADCDEV_3,
#endif
#if defined(ADC4)
    ADCDEV_4,
#endif
#if defined(ADC5)
    ADCDEV_5,
#endif
    ADCDEV_COUNT
} adcDevice_e;
#endif

typedef enum {
    ADC_NONE = -1,
    ADC_BATTERY = 0,
    ADC_CURRENT,
    ADC_EXTERNAL1,
    ADC_RSSI,
#ifdef USE_ADC_INTERNAL
    // For certain processors internal sensors are treated in the similar fashion as regular ADC inputs
    ADC_SOURCE_INTERNAL_FIRST_ID,
    ADC_TEMPSENSOR = ADC_SOURCE_INTERNAL_FIRST_ID,
    ADC_VREFINT,
#if ADC_INTERNAL_VBAT4_ENABLED
    ADC_VBAT4,
#endif
#endif
    ADC_SOURCE_COUNT
} adcSource_e;

struct adcConfig_s;
void adcInit(const struct adcConfig_s *config);
uint16_t adcGetValue(adcSource_e source);

#ifdef USE_ADC_INTERNAL
bool adcInternalIsBusy(void);
void adcInternalStartConversion(void);
uint16_t adcInternalCompensateVref(uint16_t vrefAdcValue);
int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue);
#endif
