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

#pragma once

#include <stdint.h>

#if defined(IMXRT11XX)
// The LPADC reference is 1.825 V; with CMDL[CSCALE] clear the input is scaled by
// 30/64, giving 3.893 V full scale. A board with unscaled inputs sets
// IMXRT_ADC_INPUT_SCALED to 0.
#ifndef IMXRT_ADC_INPUT_SCALED
#define IMXRT_ADC_INPUT_SCALED  1
#endif

#if IMXRT_ADC_INPUT_SCALED
#define IMXRT_ADC_CMDL_SCALE    0
#define IMXRT_ADC_FULL_SCALE_MV 3893
#else
#define IMXRT_ADC_CMDL_SCALE    ADC_CMDL_CSCALE_MASK
#define IMXRT_ADC_FULL_SCALE_MV 1825
#endif
#endif // IMXRT11XX

#ifdef USE_ADC_INTERNAL

// The raw core temperature count adcGetValue() returns for ADC_TEMPSENSOR.
uint16_t adcInternalTemperatureRaw(void);

#endif
