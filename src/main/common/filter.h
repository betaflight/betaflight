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
#include "axis.h"

struct filter_s;
typedef struct filter_s filter_t;
typedef float (filterApplyFn)(filter_t *filter, float input);
typedef float (filterVec3ApplyFn)(filter_t *filter, float input, int axis);

typedef struct pt1Filter_s {
    float k;
    float state[1];
} pt1Filter_t;

typedef struct pt1FilterVec3_s {
    float k;
    float state[XYZ_AXIS_COUNT];
} pt1FilterVec3_t;

typedef struct pt1FilterMotorCount_s {
    float k;
    float state[MAX_SUPPORTED_MOTORS];
} pt1FilterMotorCount_t;

typedef struct pt2Filter_s {
    float k;
    float state[2][1];
} pt2Filter_t;

typedef struct pt2FilterVec3_s {
    float k;
    float state[2][XYZ_AXIS_COUNT];
} pt2FilterVec3_t;

typedef struct pt3Filter_s {
    float k;
    float state[3][1];
} pt3Filter_t;

typedef struct pt3FilterVec2_s {
    float k;
    float state[3][2];
} pt3FilterVec2_t;

typedef struct pt3FilterVec3_s {
    float k;
    float state[3][XYZ_AXIS_COUNT];
} pt3FilterVec3_t;

typedef struct biquadFilterCoeffs_s {
    float b0, b1, b2, a1, a2;
} biquadFilterCoeffs_t;

typedef struct biquadFilterState_s {
    float x1, x2, y1, y2;
} biquadFilterState_t;

typedef struct biquadFilter_s {
    biquadFilterCoeffs_t coeffs;
    biquadFilterState_t state;
} biquadFilter_t;

typedef struct biquadFilterVec3_s {
    biquadFilterCoeffs_t coeffs;
    biquadFilterState_t state[XYZ_AXIS_COUNT];
} biquadFilterVec3_t;

typedef struct biquadFilterServoCount_s {
    biquadFilterCoeffs_t coeffs;
    biquadFilterState_t state[MAX_SUPPORTED_SERVOS];
} biquadFilterServoCount_t;

typedef struct phaseComp_s {
    float b0, b1, a1;
    float x1, y1;
} phaseComp_t;

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

typedef union lowpassFilterVec3_u {
    pt1FilterVec3_t pt1Filter;
    biquadFilterVec3_t biquadFilter;
    pt2FilterVec3_t pt2Filter;
    pt3FilterVec3_t pt3Filter;
} lowpassFilterVec3_t;

float nullFilterApply(filter_t *filter, float input);
float nullFilterVec3Apply(filter_t *filter, float input, int axis);

float pt1FilterGain(float f_cut, float dT);
float pt1FilterGainFromDelay(float delay, float dT);
void pt1FilterInit(pt1Filter_t *filter, float k);
void pt1FilterInitArray(void *filter, float k_value, int count);
void pt1FilterUpdateCutoff(void *filter, float k);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApplyArray(void *filter, float input, int element);

float pt2FilterGain(float f_cut, float dT);
float pt2FilterGainFromDelay(float delay, float dT);
void pt2FilterInit(pt2Filter_t *filter, float k);
void pt2FilterInitArray(void *filter, float k_value, int count);
void pt2FilterUpdateCutoff(void *filter, float k);
float pt2FilterApply(pt2Filter_t *filter, float input);
float pt2FilterApplyArray(void *filter, float input, int element);

float pt3FilterGain(float f_cut, float dT);
float pt3FilterGainFromDelay(float delay, float dT);
void pt3FilterInit(pt3Filter_t *filter, float k);
void pt3FilterInitArray(void *filter, float k_value, int count);
void pt3FilterUpdateCutoff(void *filter, float k);
float pt3FilterApply(pt3Filter_t *filter, float input);
float pt3FilterApplyArray(void *filter, float input, int element);

float filterGetNotchQ(float centerFreq, float cutoffFreq);

void biquadFilterCoeffsLPF(biquadFilterCoeffs_t *filter, float filterFreq, float dt);
void biquadFilterCoeffsNotch(biquadFilterCoeffs_t *filter, float filterFreq, float dt, float q);
void biquadFilterCoeffsNotchWeighted(biquadFilterCoeffs_t *filter, float filterFreq, float dt, float q, float weight);
void biquadFilterInitState(biquadFilterState_t *state);
float biquadFilterApplyDF1(biquadFilterState_t *state, biquadFilterCoeffs_t coeffs, float input);
float biquadFilterApply(biquadFilterState_t *state, biquadFilterCoeffs_t coeffs, float input);

void phaseCompInit(phaseComp_t *filter, const float centerFreq, const float centerPhase, const uint32_t looptimeUs);
void phaseCompUpdate(phaseComp_t *filter, const float centerFreq, const float centerPhase, const uint32_t looptimeUs);
float phaseCompApply(phaseComp_t *filter, const float input);

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold);
float slewFilterApply(slewFilter_t *filter, float input);

void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf);
float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input);

void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift);
int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal);

void meanAccumulatorInit(meanAccumulator_t *filter);
void meanAccumulatorAdd(meanAccumulator_t *filter, const int8_t newVal);
int8_t meanAccumulatorCalc(meanAccumulator_t *filter, const int8_t defaultValue);
