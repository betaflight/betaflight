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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// -ffast-math lets the compiler drop isfinite(), so test the bits
static inline bool sitlIsFinite(double value)
{
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7ff0000000000000ULL) != 0x7ff0000000000000ULL;
}

// SITL's ADC (12 bits, 3.3 V) holds the samples that the ADC meters turn back into the simulator's values.
// The defaults read up to 36.3 V and 82.5 A.
#define SITL_ADC_VREF_MV            3300.0

static inline uint16_t sitlAdcSample(double sample)
{
    if (!(sample > 0.0)) {
        return 0;
    }
    return sample >= 4095.0 ? 4095 : (uint16_t)lrint(sample);
}

// voltage.c: cV = sample * scale * vref / 10 / (0xFFF * divider) / multiplier
static inline uint16_t sitlAdcVoltageSample(double volts, uint8_t scale, uint8_t divider, uint8_t multiplier)
{
    if (!sitlIsFinite(volts) || volts <= 0.0 || scale == 0) {
        return 0;
    }
    return sitlAdcSample(volts * 100.0 * multiplier * divider * 0xFFF * 10.0 / (scale * SITL_ADC_VREF_MV));
}

// current.c: mA = mV * 10000 / scale + offset, with mV = sample * vref / 4096
static inline uint16_t sitlAdcCurrentSample(double amps, int16_t scale, int16_t offsetMa)
{
    if (!sitlIsFinite(amps) || amps < 0.0 || scale == 0) {
        return 0;
    }
    return sitlAdcSample((amps * 1000.0 - offsetMa) * scale / 10000.0 / SITL_ADC_VREF_MV * 4096.0);
}

// The simulator's field (gauss, FRD body frame) as virtual mag samples (X forward, Y left, Z up); full scale 4 G
#define SITL_MAG_SAMPLES_PER_GAUSS  8192.0
#define SITL_PITOT_MAX_PA           1.0e6

static inline int16_t sitlMagSample(double gauss)
{
    const double sample = gauss * SITL_MAG_SAMPLES_PER_GAUSS;
    return sample >= 32767.0 ? 32767 : sample <= -32767.0 ? -32767 : (int16_t)lrint(sample);
}

// Differential pressure that is not a number or out of range reads 0
static inline float sitlPitotPressure(double pa)
{
    return sitlIsFinite(pa) && fabs(pa) < SITL_PITOT_MAX_PA ? (float)pa : 0.0f;
}

// Returns false for a field of 0, 0, 0 or a field that is not a number (not simulated)
static inline bool sitlMagSamples(const double fieldFrd[3], int16_t samples[3])
{
    if (fieldFrd[0] == 0.0 && fieldFrd[1] == 0.0 && fieldFrd[2] == 0.0) {
        return false;
    }
    if (!sitlIsFinite(fieldFrd[0]) || !sitlIsFinite(fieldFrd[1]) || !sitlIsFinite(fieldFrd[2])) {
        return false;
    }
    samples[0] = sitlMagSample(fieldFrd[0]);
    samples[1] = sitlMagSample(-fieldFrd[1]);
    samples[2] = sitlMagSample(-fieldFrd[2]);
    return true;
}
