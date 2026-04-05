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

struct filter_s;
typedef struct filter_s filter_t;
typedef float (*filterApplyFnPtr)(filter_t *filter, float input);

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BUTTERWORTH,
    FILTER_PT2,
    FILTER_PT3,
} lowpassFilterType_e;

typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

typedef struct pt2Filter_s {
    float state;
    float state1;
    float k;
} pt2Filter_t;

typedef struct pt3Filter_s {
    float state;
    float state1;
    float state2;
    float k;
} pt3Filter_t;

// SVF filter in Chamberlin form (less accurate cutoff but fine for lpf)
typedef struct butterworthFilter_s {
    float f;
    float low;
    float band;
} butterworthFilter_t;

// SVF filter in TPT form
// this version will have issues if you change Q on the fly
// heavily optimized to work as a notch filter
typedef struct notchFilter_s {
    float a1;
    float a2q;  // a2 * q
    float fq;   // f / q
    float ic1q; // State 1 scaled by q
    float ic2;  // State 2 (unscaled)
} notchFilter_t;

// SVF filter in TPT form
// heavily optimized to work as a notch filter with a weight
typedef struct rpmNotch_s {
    float f;
    float a1;
    float a2;
    float wq;          // q * weight — fully precomputed notch+weight term
    float ic1[3];
    float ic2[3];
} rpmNotch_t;

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

float nullFilterApply(filter_t *filter, float input);

float pt1FilterGain(float f_cut, float dT);
float pt1FilterGainFromDelay(float delay, float dT);
void pt1FilterInit(pt1Filter_t *filter, float k);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
float pt1FilterApply(pt1Filter_t *filter, float input);

float pt2FilterGain(float f_cut, float dT);
float pt2FilterGainFromDelay(float delay, float dT);
void pt2FilterInit(pt2Filter_t *filter, float k);
void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k);
float pt2FilterApply(pt2Filter_t *filter, float input);

float pt3FilterGain(float f_cut, float dT);
float pt3FilterGainFromDelay(float delay, float dT);
void pt3FilterInit(pt3Filter_t *filter, float k);
void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k);
float pt3FilterApply(pt3Filter_t *filter, float input);

float filterGetNotchQ(float centerFreq, float cutoffFreq);

void butterworthFilterInit(butterworthFilter_t *filter, float filterFreq, float dt);
void butterworthFilterUpdate(butterworthFilter_t *filter, float filterFreq, float dt);
float butterworthFilterApply(butterworthFilter_t *filter, float input);

void notchInit(notchFilter_t *filter, float filterFreq, float dt, float Q);
void notchUpdate(notchFilter_t *filter, float filterFreq, float dt, float Q);
float notchApply(notchFilter_t *filter, float input);

void rpmNotchInit(rpmNotch_t *filter, float filterFreq, float dt, float Q, float weight);
void rpmNotchUpdate(rpmNotch_t *filter, float filterFreq, float dt, float Q, float weight);
void rpmNotchApply(rpmNotch_t *filter, float input[3]);

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
