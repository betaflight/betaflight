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
#include <stdint.h>

#include "filter_magic_h.h"

#define DEFAULT_FILTER_DIMS (1)(2)(3)(4)(8)

#ifdef __cplusplus
// C++ does not like zzero-size arrays
DECLARE_FILTER(null, 1, 1, DEFAULT_FILTER_DIMS);
#else
DECLARE_FILTER(null, 0, 0, DEFAULT_FILTER_DIMS);
#endif

float rcFilterGain(float f_cut, float dT);
float rcFilterGainFromDelay(float delay, float dT);

DECLARE_FILTER(pt1, 1, 1, DEFAULT_FILTER_DIMS);
DECLARE_FILTER(pt2, 1, 2, DEFAULT_FILTER_DIMS);
DECLARE_FILTER(pt3, 1, 3, DEFAULT_FILTER_DIMS);

DEFINE_FILTER_INIT_KIND(pt1, pt1, DEFAULT_FILTER_DIMS, LPF, ((float, f_cut))((float, dT)));
DEFINE_FILTER_INIT_KIND(pt1, pt1, DEFAULT_FILTER_DIMS, Delay, ((float, f_cut))((float, dT)));
DEFINE_FILTER_INIT_KIND(pt1, pt1, DEFAULT_FILTER_DIMS, Alpha, ((float, alpha)));
DEFINE_FILTER_INIT_KIND(pt2, pt2, DEFAULT_FILTER_DIMS, LPF, ((float, f_cut))((float, dT)));
DEFINE_FILTER_INIT_KIND(pt2, pt2, DEFAULT_FILTER_DIMS, Delay, ((float, f_cut))((float, dT)));
DEFINE_FILTER_INIT_KIND(pt3, pt3, DEFAULT_FILTER_DIMS, LPF, ((float, f_cut))((float, dT)));
DEFINE_FILTER_INIT_KIND(pt3, pt3, DEFAULT_FILTER_DIMS, Delay, ((float, f_cut))((float, dT)));

enum biquad_coef_e {biquad_b0, biquad_b1, biquad_b2, biquad_a1, biquad_a2};
enum biquad_state_df1_e {biquad_x1, biquad_x2, biquad_y1, biquad_y2};
enum biquad_state_df2_e {biquad_s1, biquad_s2};
DECLARE_FILTER(biquad, 5, 2, DEFAULT_FILTER_DIMS);
DECLARE_FILTER_2(biquad, biquadDF1, 5, 4, DEFAULT_FILTER_DIMS);

float filterGetNotchQ(float centerFreq, float cutoffFreq);
void biquadFilterCoeffsApplyWeight(FILTER_COEFFS_TYPE(biquad) *dst, float weight);

DEFINE_FILTER_INIT_KIND(biquad, biquad, DEFAULT_FILTER_DIMS, LPF, ((float, filterFreq))((float, dT)));
DEFINE_FILTER_INIT_KIND(biquad, biquad, DEFAULT_FILTER_DIMS, Notch, ((float, filterFreq))((float, dT))((float, q)));
DEFINE_FILTER_INIT_KIND(biquad, biquadDF1, DEFAULT_FILTER_DIMS, LPF, ((float, filterFreq))((float, dT)));
DEFINE_FILTER_INIT_KIND(biquad, biquadDF1, DEFAULT_FILTER_DIMS, Notch, ((float, filterFreq))((float, dT))((float, q)));

enum phaseComp_coef_e {phaseComp_b0, phaseComp_b1, phaseComp_a1};
enum phaseComp_state_e {phaseComp_x1, phaseComp_y1};
DECLARE_FILTER(phaseComp, 3, 2, (1));
void phaseCompFilterCoeffsCenPh(FILTER_COEFFS_TYPE(phaseComp) *dst, const float centerFreqHz, const float centerPhaseDeg, const float dT);
DEFINE_FILTER_INIT_KIND(phaseComp, phaseComp, (1), CenPh, ((float, centerFreqHz))((float, centerPhaseDeg))((float, dT)));


typedef struct slewFilter_s {
    float state;
    float slewLimit;
    float threshold;
} slewFilter_t;

typedef struct laggedMovingAverage_s {
    uint16_t movingWindowIndex;
    uint16_t windowSize;
    float movingSum;
    float *buf;
    bool primed;
} laggedMovingAverage_t;

typedef struct simpleLowpassFilter_s {
    int32_t fp;
    int32_t beta;
    int32_t fpShift;
} simpleLowpassFilter_t;

typedef struct meanAccumulator_s {
    int32_t accumulator;
    int32_t count;
} meanAccumulator_t;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_PT2,
    FILTER_PT3,
} lowpassFilterType_e;

#define LOWPASS_FILTERS (null, pt1, biquad, pt2, pt3, biquadDF1)

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold);
float slewFilterApply(slewFilter_t *filter, float input);

void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf);
float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input);

void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift);
int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal);

void meanAccumulatorInit(meanAccumulator_t *filter);
void meanAccumulatorAdd(meanAccumulator_t *filter, const int8_t newVal);
int8_t meanAccumulatorCalc(meanAccumulator_t *filter, const int8_t defaultValue);
