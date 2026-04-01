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
    FILTER_BIQUAD,
    FILTER_PT2,
    FILTER_PT3,
} lowpassFilterType_e;

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

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

//#define SVF_ALTERNATIVE

typedef struct butterworthFilter_s {
#ifndef SVF_ALTERNATIVE
    float b1, a1, a2; // b0 is half of b1, b2 == b0
    float x1, x2, y1, y2;
#else
    float f;
    float q;
    float low;
    float band;
#endif
} butterworthFilter_t;

typedef struct notchFilter_s {
#ifndef SVF_ALTERNATIVE
    float b0, a1, a2; // b1 == a1, b2 == b0
    float x1, x2, y1, y2;
#else
    float f;
    float q;
    float low;
    float band;
#endif
} notchFilter_t;

typedef struct rpmNotch_s {
#ifndef SVF_ALTERNATIVE
    float b0, b2, a1, a2; // b1 == a1
//    float weight; // TODO test the speed of the other weight version
    float x1[3], x2[3], y1[3], y2[3];
#else
    float f;
    float q;
    float low[3];
    float band[3];
    float weight;
#endif
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
float butterworthFilterApplyMoving(butterworthFilter_t *filter, float input);
float butterworthFilterApplyStatic(butterworthFilter_t *filter, float input);

void notchInit(notchFilter_t *filter, float filterFreq, float dt, float Q);
void notchUpdate(notchFilter_t *filter, float filterFreq, float dt, float Q);
float notchApplyMoving(notchFilter_t *filter, float input);
float notchApplyStatic(notchFilter_t *filter, float input);

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
