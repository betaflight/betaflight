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

 // Implementation of a Sliding Discrete Fourier Transform (SDFT).
 // Complexity for calculating frequency spectrum with N bins is O(N).

#pragma once

#include <stdint.h>
#include <complex.h>
#undef I  // avoid collision of imaginary unit I with variable I in pid.h
typedef float complex complex_t; // Better readability for type "float complex"

#define SDFT_SAMPLE_SIZE 72
#define SDFT_BIN_COUNT   (SDFT_SAMPLE_SIZE / 2)

typedef struct sdft_s {

    uint8_t idx;                       // circular buffer index
    uint8_t startBin;
    uint8_t endBin;
    uint8_t batchSize;
    uint8_t numBatches;
    float samples[SDFT_SAMPLE_SIZE];   // circular buffer
    complex_t data[SDFT_BIN_COUNT];    // complex frequency spectrum

} sdft_t;

STATIC_ASSERT(SDFT_SAMPLE_SIZE <= (uint8_t)-1, window_size_greater_than_underlying_type);

void sdftInit(sdft_t *sdft, const uint8_t startBin, const uint8_t endBin, const uint8_t numBatches);
void sdftPush(sdft_t *sdft, const float *sample);
void sdftPushBatch(sdft_t *sdft, const float *sample, const uint8_t *batchIdx);
void sdftMagSq(const sdft_t *sdft, float *output);
void sdftMagnitude(const sdft_t *sdft, float *output);
void sdftWinSq(const sdft_t *sdft, float *output);
void sdftWindow(const sdft_t *sdft, float *output);
