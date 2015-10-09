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


#define LOWPASS_NUM_COEF 3
#define LPF_ROUND(x) (x < 0 ? (x - 0.5f) : (x + 0.5f))

typedef struct lowpass_s {
    bool init;
    int16_t freq;                           // Normalized freq in 1/1000ths
    float bf[LOWPASS_NUM_COEF];
    float af[LOWPASS_NUM_COEF];
    int64_t b[LOWPASS_NUM_COEF];
    int64_t a[LOWPASS_NUM_COEF];
    int16_t coeff_shift;
    int16_t input_shift;
    int32_t input_bias;
    float xf[LOWPASS_NUM_COEF];
    float yf[LOWPASS_NUM_COEF];
    int32_t x[LOWPASS_NUM_COEF];
    int32_t y[LOWPASS_NUM_COEF];
} lowpass_t;

void generateLowpassCoeffs2(int16_t freq, lowpass_t *filter);
int32_t lowpassFixed(lowpass_t *filter, int32_t in, int16_t freq);
