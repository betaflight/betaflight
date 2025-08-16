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

#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#define BUTTERWORTH_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/

// PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
#define CUTOFF_CORRECTION_PT2 1.553773974f
#define CUTOFF_CORRECTION_PT3 1.961459177f

// NULL filter

float nullFilterApply(filter_t *filter, float input)
{
    UNUSED(filter);
    return input;
}

float nullFilterVec3Apply(filter_t *filter, float input, int axis)
{
    UNUSED(filter);
    UNUSED(axis);
    return input;
}

// PT1 Low Pass filter

FAST_CODE_NOINLINE float pt1FilterGain(float f_cut, float dT)
{
    float omega = 2.0f * M_PIf * f_cut * dT;
    return omega / (omega + 1.0f);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
float pt1FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    // cutoffHz = 1.0f / (2.0f * M_PIf * delay)

    return dT / (dT + delay);
}

void pt1FilterInit(pt1Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->k = k;
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
{
    filter->k = k;
}

FAST_CODE float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

// PT1 Vec3 Low Pass filter

void pt1FilterVec3Init(pt1FilterVec3_t *filter, float k)
{
    memset(filter->state, 0.0f, sizeof(filter->state));
    filter->k = k;
}

void pt1FilterVec3UpdateCutoff(pt1FilterVec3_t *filter, float k)
{
    filter->k = k;
}

FAST_CODE float pt1FilterVec3Apply(pt1FilterVec3_t *filter, float input, int axis)
{
    filter->state[axis] = filter->state[axis] + filter->k * (input - filter->state[axis]);
    return filter->state[axis];
}

// PT2 Low Pass filter

FAST_CODE float pt2FilterGain(float f_cut, float dT)
{
    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT2, dT);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
float pt2FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    // cutoffHz = 1.0f / (2.0f * M_PIf * delay * CUTOFF_CORRECTION_PT2)

    return dT / (dT + delay * CUTOFF_CORRECTION_PT2);
}

void pt2FilterInit(pt2Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->k = k;
}

void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k)
{
    filter->k = k;
}

FAST_CODE float pt2FilterApply(pt2Filter_t *filter, float input)
{
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state = filter->state + filter->k * (filter->state1 - filter->state);
    return filter->state;
}

// PT2 Vec3 Low Pass filter

void pt2FilterVec3Init(pt2FilterVec3_t *filter, float k)
{
    memset(filter->state, 0.0f, sizeof(filter->state));
    memset(filter->state1, 0.0f, sizeof(filter->state1));
    filter->k = k;
}

void pt2FilterVec3UpdateCutoff(pt2FilterVec3_t *filter, float k)
{
    filter->k = k;
}

FAST_CODE float pt2FilterVec3Apply(pt2FilterVec3_t *filter, float input, int axis)
{
    filter->state1[axis] = filter->state1[axis] + filter->k * (input - filter->state1[axis]);
    filter->state[axis] = filter->state[axis] + filter->k * (filter->state1[axis] - filter->state[axis]);
    return filter->state[axis];
}

// PT3 Low Pass filter

FAST_CODE float pt3FilterGain(float f_cut, float dT)
{
    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT3, dT);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
float pt3FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    // cutoffHz = 1.0f / (2.0f * M_PIf * delay * CUTOFF_CORRECTION_PT3)

    return dT / (dT + delay * CUTOFF_CORRECTION_PT3);
}

void pt3FilterInit(pt3Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->state2 = 0.0f;
    filter->k = k;
}

void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k)
{
    filter->k = k;
}

FAST_CODE float pt3FilterApply(pt3Filter_t *filter, float input)
{
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state2 = filter->state2 + filter->k * (filter->state1 - filter->state2);
    filter->state = filter->state + filter->k * (filter->state2 - filter->state);
    return filter->state;
}

// PT3 Vec3 Low Pass filter

void pt3FilterVec3Init(pt3FilterVec3_t *filter, float k)
{
    memset(filter->state, 0.0f, sizeof(filter->state));
    memset(filter->state1, 0.0f, sizeof(filter->state1));
    memset(filter->state2, 0.0f, sizeof(filter->state2));
    filter->k = k;
}

void pt3FilterVec3UpdateCutoff(pt3FilterVec3_t *filter, float k)
{
    filter->k = k;
}

FAST_CODE float pt3FilterVec3Apply(pt3FilterVec3_t *filter, float input, int axis)
{
    filter->state1[axis] = filter->state1[axis] + filter->k * (input - filter->state1[axis]);
    filter->state2[axis] = filter->state2[axis] + filter->k * (filter->state1[axis] - filter->state2[axis]);
    filter->state[axis] = filter->state[axis] + filter->k * (filter->state2[axis] - filter->state[axis]);
    return filter->state[axis];
}

// Biquad filter

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

/* sets up a biquad filter as a 2nd order butterworth LPF */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, float dt)
{
    biquadFilterUpdateLPF(filter, filterFreq, dt);

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

void biquadFilterInitNotch(biquadFilter_t *filter, float filterFreq, float dt, float Q, float weight)
{
    biquadFilterUpdateNotch(filter, filterFreq, dt, Q, weight);

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

FAST_CODE void biquadFilterGainsLPF(biquadFilterGains_t *filter, float filterFreq, float dt)
{
    // setup variables
    const float omega = 2.0f * M_PIf * filterFreq * dt;
    const float sn = sin_approx_unchecked(omega);
    const float cs = cos_approx_unchecked(omega);
    const float alpha = sn / (2.0f * BUTTERWORTH_Q);
    const float invA0 = 1.0f / (1.0f + alpha);

     // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
     // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
     filter->b1 = (1 - cs) * invA0;
     filter->b0 = (filter->b1 * 0.5f) * invA0;
     filter->b2 = (filter->b0) * invA0;
     filter->a1 = (-2 * cs) * invA0;
     filter->a2 = (1 - alpha) * invA0;
}

FAST_CODE void biquadFilterGainsNotch(biquadFilterGains_t *filter, float filterFreq, float dt, float q, float weight)
{
    // setup variables
    const float omega = 2.0f * M_PIf * filterFreq * dt;
    const float sn = sin_approx_unchecked(omega);
    const float cs = cos_approx_unchecked(omega);
    const float alpha = sn / (2.0f * q);
    const float invA0 = 1.0f / (1.0f + alpha);

    filter->b0 = invA0;
    filter->b1 = (-2 * cs) * invA0;
    filter->b2 = invA0;
    filter->a1 = filter->b1;
    filter->a2 = (1 - alpha) * invA0;

    filter->weight = weight;
}

FAST_CODE void biquadFilterUpdateLPF(biquadFilter_t *filter, float filterFreq, float dt)
{
    biquadFilterGainsLPF(&filter->gains, filterFreq, dt);
}

FAST_CODE void biquadFilterUpdateNotch(biquadFilter_t *filter, float filterFreq, float dt, float q, float weight)
{
    biquadFilterGainsNotch(&filter->gains, filterFreq, dt, q, weight);
}

/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
FAST_CODE float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    /* compute result */
    float b0 = filter->gains.b0; float b1 = filter->gains.b1; float b2 = filter->gains.b2;
    float a1 = filter->gains.a1; float a2 = filter->gains.a2;

    const float result = b0 * input + b1 * filter->x1 + b2 * filter->x2 - a1 * filter->y1 - a2 * filter->y2;

    /* shift x1 to x2, input to x1 */
    filter->x2 = filter->x1;
    filter->x1 = input;

    /* shift y1 to y2, result to y1 */
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
}

/* Computes a biquadFilter_t filter in df1 and crossfades input with output */
FAST_CODE float biquadFilterApplyDF1Weighted(biquadFilter_t* filter, float input)
{
    // compute result
    const float result = biquadFilterApplyDF1(filter, input);

    // crossfading of input and output to turn filter on/off gradually
    return filter->gains.weight * result + (1 - filter->gains.weight) * input;
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
FAST_CODE float biquadFilterApply(biquadFilter_t *filter, float input)
{
    float b0 = filter->gains.b0; float b1 = filter->gains.b1; float b2 = filter->gains.b2;
    float a1 = filter->gains.a1; float a2 = filter->gains.a2;

    const float result = b0 * input + filter->x1;

    filter->x1 = b1 * input - a1 * result + filter->x2;
    filter->x2 = b2 * input - a2 * result;

    return result;
}

// Biquad Vec3 filter

void biquadFilterVec3InitLPF(biquadFilterVec3_t *filter, float filterFreq, float dt)
{
    biquadFilterVec3UpdateLPF(filter, filterFreq, dt);

    // zero initial samples
    memset(filter->x1, 0.0f, sizeof(filter->x1));
    memset(filter->x2, 0.0f, sizeof(filter->x2));
    memset(filter->y1, 0.0f, sizeof(filter->y1));
    memset(filter->y2, 0.0f, sizeof(filter->y2));
}

void biquadFilterVec3InitNotch(biquadFilterVec3_t *filter, float filterFreq, float dt, float Q, float weight)
{
    biquadFilterVec3UpdateNotch(filter, filterFreq, dt, Q, weight);

    // zero initial samples
    memset(filter->x1, 0.0f, sizeof(filter->x1));
    memset(filter->x2, 0.0f, sizeof(filter->x2));
    memset(filter->y1, 0.0f, sizeof(filter->y1));
    memset(filter->y2, 0.0f, sizeof(filter->y2));
}

FAST_CODE void biquadFilterVec3UpdateLPF(biquadFilterVec3_t *filter, float filterFreq, float dt)
{
    biquadFilterGainsLPF(&filter->gains, filterFreq, dt);
}

FAST_CODE void biquadFilterVec3UpdateNotch(biquadFilterVec3_t *filter, float filterFreq, float dt, float q, float weight)
{
    biquadFilterGainsNotch(&filter->gains, filterFreq, dt, q, weight);
}

/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
FAST_CODE float biquadFilterVec3ApplyDF1(biquadFilterVec3_t *filter, float input, int axis)
{
    /* compute result */
    float b0 = filter->gains.b0; float b1 = filter->gains.b1; float b2 = filter->gains.b2;
    float a1 = filter->gains.a1; float a2 = filter->gains.a2;

    const float result = b0 * input + b1 * filter->x1[axis] + b2 * filter->x2[axis] - a1 * filter->y1[axis] - a2 * filter->y2[axis];

    /* shift x1 to x2, input to x1 */
    filter->x2[axis] = filter->x1[axis];
    filter->x1[axis] = input;

    /* shift y1 to y2, result to y1 */
    filter->y2[axis] = filter->y1[axis];
    filter->y1[axis] = result;

    return result;
}

/* Computes a biquadFilter_t filter in df1 and crossfades input with output */
FAST_CODE float biquadFilterVec3ApplyDF1Weighted(biquadFilterVec3_t* filter, float input, int axis)
{
    // compute result
    const float result = biquadFilterVec3ApplyDF1(filter, input, axis);

    // crossfading of input and output to turn filter on/off gradually
    return filter->gains.weight * result + (1 - filter->gains.weight) * input;
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
FAST_CODE float biquadFilterVec3Apply(biquadFilterVec3_t *filter, float input, int axis)
{
    float b0 = filter->gains.b0; float b1 = filter->gains.b1; float b2 = filter->gains.b2;
    float a1 = filter->gains.a1; float a2 = filter->gains.a2;

    const float result = b0 * input + filter->x1[axis];

    filter->x1[axis] = b1 * input - a1 * result + filter->x2[axis];
    filter->x2[axis] = b2 * input - a2 * result;

    return result;
}

// Phase Compensator (Lead-Lag-Compensator)

void phaseCompInit(phaseComp_t *filter, const float centerFreqHz, const float centerPhaseDeg, const uint32_t looptimeUs)
{
    phaseCompUpdate(filter, centerFreqHz, centerPhaseDeg, looptimeUs);

    filter->x1 = 0.0f;
    filter->y1 = 0.0f;
}

FAST_CODE void phaseCompUpdate(phaseComp_t *filter, const float centerFreqHz, const float centerPhaseDeg, const uint32_t looptimeUs)
{
    const float omega = 2.0f * M_PIf * centerFreqHz * looptimeUs * 1e-6f;
    const float sn = sin_approx(centerPhaseDeg * RAD);
    const float gain = (1 + sn) / (1 - sn);
    const float alpha = (12 - sq(omega)) / (6 * omega * sqrtf(gain));  // approximate prewarping (series expansion)

    filter->b0 = 1 + alpha * gain;
    filter->b1 = 2 - filter->b0;
    filter->a1 = 1 - alpha;

    const float a0 = 1 / (1 + alpha);

    filter->b0 *= a0;
    filter->b1 *= a0;
    filter->a1 *= a0;
}

FAST_CODE float phaseCompApply(phaseComp_t *filter, const float input)
{
    // compute result
    const float result = filter->b0 * input + filter->b1 * filter->x1 - filter->a1 * filter->y1;

    // shift input to x1 and result to y1
    filter->x1 = input;
    filter->y1 = result;

    return result;
}

// Slew filter with limit

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold)
{
    filter->state = 0.0f;
    filter->slewLimit = slewLimit;
    filter->threshold = threshold;
}

FAST_CODE float slewFilterApply(slewFilter_t *filter, float input)
{
    if (filter->state >= filter->threshold) {
        if (input >= filter->state - filter->slewLimit) {
            filter->state = input;
        }
    } else if (filter->state <= -filter->threshold) {
        if (input <= filter->state + filter->slewLimit) {
            filter->state = input;
        }
    } else {
        filter->state = input;
    }
    return filter->state;
}

// Moving average

void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf)
{
    filter->movingWindowIndex = 0;
    filter->windowSize = windowSize;
    filter->buf = buf;
    filter->movingSum = 0;
    memset(filter->buf, 0, windowSize * sizeof(float));
    filter->primed = false;
}

FAST_CODE float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input)
{
    filter->movingSum -= filter->buf[filter->movingWindowIndex];
    filter->buf[filter->movingWindowIndex] = input;
    filter->movingSum += input;

    if (++filter->movingWindowIndex == filter->windowSize) {
        filter->movingWindowIndex = 0;
        filter->primed = true;
    }

    const uint16_t denom = filter->primed ? filter->windowSize : filter->movingWindowIndex;
    return filter->movingSum / denom;
}

// Simple fixed-point lowpass filter based on integer math

void simpleLPFilterInit(simpleLowpassFilter_t *filter, int32_t beta, int32_t fpShift)
{
    filter->fp = 0;
    filter->beta = beta;
    filter->fpShift = fpShift;
}

int32_t simpleLPFilterUpdate(simpleLowpassFilter_t *filter, int32_t newVal)
{
    filter->fp = (filter->fp << filter->beta) - filter->fp;
    filter->fp += newVal << filter->fpShift;
    filter->fp >>= filter->beta;
    int32_t result = filter->fp >> filter->fpShift;
    return result;
}

// Mean accumulator

void meanAccumulatorInit(meanAccumulator_t *filter)
{
    filter->accumulator = 0;
    filter->count = 0;
}

void meanAccumulatorAdd(meanAccumulator_t *filter, const int8_t newVal)
{
    filter->accumulator += newVal;
    filter->count++;
}

int8_t meanAccumulatorCalc(meanAccumulator_t *filter, const int8_t defaultValue)
{
    if (filter->count) {
        int8_t retVal = filter->accumulator / filter->count;
        meanAccumulatorInit(filter);
        return retVal;
    }
    return defaultValue;
}
