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

typedef enum {
    ADC_NONE = -1,
    ADC_BATTERY = 0,
    ADC_CURRENT = 1,
    ADC_EXTERNAL1 = 2,
    ADC_RSSI = 3,
#ifdef USE_ADC_INTERNAL
    // On H7 and G4, internal sensors are treated in the similar fashion as regular ADC inputs
    ADC_SOURCE_INTERNAL_FIRST_ID = 4,
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
extern int32_t adcVREFINTCAL; // ADC value (12-bit) of band gap with Vref = VREFINTCAL_VREF
extern int32_t adcTSCAL1;
extern int32_t adcTSCAL2;
extern int32_t adcTSSlopeK;

bool adcInternalIsBusy(void);
void adcInternalStartConversion(void);
uint16_t adcInternalCompensateVref(uint16_t vrefAdcValue);
int16_t adcInternalComputeTemperature(uint16_t tempAdcValue, uint16_t vrefValue);
#endif
