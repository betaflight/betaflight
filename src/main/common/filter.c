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
FAST_CODE float pt1FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    // cutoffHz = 1.0f / (2.0f * M_PIf * delay)

    return dT / (dT + delay);
}

void pt1FilterInitArray(void *filter, float k_value, int count)
{
    pt1Filter_t *f = (pt1Filter_t*)filter;
    f->k = k_value;
    for (int i = 0; i < count; i++) {
        f->state[i] = 0.0f;
    }
}

void pt1FilterInit(pt1Filter_t *filter, float k)
{
    pt1FilterInitArray(filter, k, 1);
}

void pt1FilterUpdateCutoff(void *filter, float k)
{
    pt1Filter_t *f = (pt1Filter_t*)filter;
    f->k = k;
}

FAST_CODE float pt1FilterApplyArray(void *filter, float input, int element)
{
    pt1Filter_t *f = (pt1Filter_t*)filter;
    f->state[element] = f->state[element] + f->k * (input - f->state[element]);
    return f->state[element];
}

FAST_CODE float pt1FilterApply(pt1Filter_t *filter, float input)
{
    return pt1FilterApplyArray(filter, input, 0);
}

// PT2 Low Pass filter

FAST_CODE float pt2FilterGain(float f_cut, float dT)
{
    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT2, dT);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
FAST_CODE float pt2FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    // cutoffHz = 1.0f / (2.0f * M_PIf * delay * CUTOFF_CORRECTION_PT2)

    return dT / (dT + delay * CUTOFF_CORRECTION_PT2);
}

void pt2FilterInitArray(void *filter, float k_value, int count)
{
    pt2Filter_t *f = (pt2Filter_t*)filter;
    f->k = k_value;

    // splitting the state as below suppresses compiler warnings
    float *state0 = &f->state[0][0];
    float *state1 = &f->state[1][0];
    for (int i = 0; i < count; i++) {
        state0[i] = 0.0f;
        state1[i] = 0.0f;
    }
}

void pt2FilterInit(pt2Filter_t *filter, float k)
{
    pt2FilterInitArray(filter, k, 1);
}

void pt2FilterUpdateCutoff(void *filter, float k)
{
    pt2Filter_t *f = (pt2Filter_t*)filter;
    f->k = k;
}

FAST_CODE float pt2FilterApplyArray(void *filter, float input, int element)
{
    pt2Filter_t *f = (pt2Filter_t*)filter;
    // splitting the state as below suppresses compiler warnings
    float *state0 = &f->state[0][0];
    float *state1 = &f->state[1][0];

    state0[element] += f->k * (input -state0[element]);            // stage 1
    state1[element] += f->k * (state0[element] - state1[element]); // stage 2
    return state1[element];
}

FAST_CODE float pt2FilterApply(pt2Filter_t *filter, float input)
{
    return pt2FilterApplyArray(filter, input, 0);
}

// PT3 Low Pass filter

FAST_CODE float pt3FilterGain(float f_cut, float dT)
{
    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT3, dT);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
FAST_CODE float pt3FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    // cutoffHz = 1.0f / (2.0f * M_PIf * delay * CUTOFF_CORRECTION_PT3)

    return dT / (dT + delay * CUTOFF_CORRECTION_PT3);
}

void pt3FilterInitArray(void *filter, float k_value, int count)
{
    pt2Filter_t *f = (pt2Filter_t*)filter;
    f->k = k_value;

    // splitting the state as below suppresses compiler warnings
    float *state0 = &f->state[0][0];
    float *state1 = &f->state[1][0];
    float *state2 = &f->state[2][0];
    for (int i = 0; i < count; i++) {
        state0[i] = 0.0f;
        state1[i] = 0.0f;
        state2[i] = 0.0f;
    }
}

void pt3FilterInit(pt3Filter_t *filter, float k)
{
    pt3FilterInitArray(filter, k, 1);
}

void pt3FilterUpdateCutoff(void *filter, float k)
{
    pt3Filter_t *f = (pt3Filter_t*)filter;
    f->k = k;
}

FAST_CODE float pt3FilterApplyArray(void *filter, float input, int element)
{
    pt3Filter_t *f = (pt3Filter_t*)filter;
    // splitting the state as below suppresses compiler warnings
    float *state0 = &f->state[0][0];
    float *state1 = &f->state[1][0];
    float *state2 = &f->state[2][0];

    state0[element] += f->k * (input - state0[element]);           // stage 1
    state1[element] += f->k * (state0[element] - state1[element]); // stage 2
    state2[element] += f->k * (state1[element] - state2[element]); // stage 3

    return state2[element];
}

FAST_CODE float pt3FilterApply(pt3Filter_t *filter, float input)
{
    return pt3FilterApplyArray(filter, input, 0);
}

// Biquad filter

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

FAST_CODE void biquadFilterGainsLPF(biquadFilterGains_t *filter, float filterFreq, float dt)
{
    // setup variables
    float omega = 2.0f * M_PIf * filterFreq * dt;
    const float cs = cos_approx_unchecked(-omega); // this is a hack since the valid range of cos unchecked is -pi to 0
    if (omega > (M_PIf * 0.5f)) {
        omega = M_PIf - omega;
    }
    const float sn = sin_approx_unchecked(omega);
    const float alpha = sn / (2.0f * BUTTERWORTH_Q);
    const float invA0 = 1.0f / (1.0f + alpha);

     // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
     // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
     filter->b1 = (1 - cs) * invA0;
     filter->b0 = (filter->b1 * 0.5f);
     filter->b2 = (filter->b0);
     filter->a1 = (-2 * cs) * invA0;
     filter->a2 = (1 - alpha) * invA0;
}

FAST_CODE void biquadFilterGainsNotch(biquadFilterGains_t *filter, float filterFreq, float dt, float q)
{
    // setup variables
    float omega = 2.0f * M_PIf * filterFreq * dt;
    const float cs = cos_approx_unchecked(-omega); // this is a hack since the valid range of cos unchecked is -pi to 0
    if (omega > (M_PIf * 0.5f)) {
        omega = M_PIf - omega;
    }
    const float sn = sin_approx_unchecked(omega);
    const float alpha = sn / (2.0f * q);
    const float invA0 = 1.0f / (1.0f + alpha);

    filter->b0 = invA0;
    filter->b1 = (-2 * cs) * invA0;
    filter->b2 = invA0;
    filter->a1 = filter->b1;
    filter->a2 = (1 - alpha) * invA0;
}

FAST_CODE void biquadFilterGainsNotchWeighted(biquadFilterGains_t *filter, float filterFreq, float dt, float q, float weight)
{
    // setup variables
    float omega = 2.0f * M_PIf * filterFreq * dt;
    const float cs = cos_approx_unchecked(-omega); // this is a hack since the valid range of cos unchecked is -pi to 0
    if (omega > (M_PIf * 0.5f)) {
        omega = M_PIf - omega;
    }
    const float sn = sin_approx_unchecked(omega);
    const float alpha = sn / (2.0f * q);
    const float invA0 = 1.0f / (1.0f + alpha);

    filter->b0 = invA0;
    filter->b1 = (-2 * cs) * invA0;
    filter->b2 = invA0;
    filter->a1 = filter->b1;
    filter->a2 = (1 - alpha) * invA0;

    // apply weight
    float weightRecip = 1.0f - weight;

    filter->b0 = weight * filter->b0 + weightRecip;
    filter->b1 = weight * filter->b1 + weightRecip * filter->a1;
    filter->b2 = weight * filter->b2 + weightRecip * filter->a2;
}

void biquadFilterInitArrayLPF(void *filter, float filterFreq, float dt, int count)
{
    biquadFilter_t *f = (biquadFilter_t*)filter;

    biquadFilterGainsLPF(&f->gains, filterFreq, dt);

    // Split state arrays to suppress compiler warnings
    float *x1_state = &f->x1[0];
    float *x2_state = &f->x2[0];
    float *y1_state = &f->y1[0];
    float *y2_state = &f->y2[0];

    // Zero initial samples
    for (int i = 0; i < count; i++) {
        x1_state[i] = 0.0f;
        x2_state[i] = 0.0f;
        y1_state[i] = 0.0f;
        y2_state[i] = 0.0f;
    }
}

void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, float dt)
{
    biquadFilterInitArrayLPF(filter, filterFreq, dt, 1);
}

void biquadFilterUpdateLPF(void *filter, float filterFreq, float dt)
{
    biquadFilter_t *f = (biquadFilter_t*)filter;
    biquadFilterGainsLPF(&f->gains, filterFreq, dt);
}

void biquadFilterInitArrayNotch(void *filter, float filterFreq, float dt, float Q, int count)
{
    biquadFilter_t *f = (biquadFilter_t*)filter;

    biquadFilterGainsNotch(&f->gains, filterFreq, dt, Q);

    // Split state arrays to suppress compiler warnings
    float *x1_state = &f->x1[0];
    float *x2_state = &f->x2[0];
    float *y1_state = &f->y1[0];
    float *y2_state = &f->y2[0];

    // Zero initial samples
    for (int i = 0; i < count; i++) {
        x1_state[i] = 0.0f;
        x2_state[i] = 0.0f;
        y1_state[i] = 0.0f;
        y2_state[i] = 0.0f;
    }
}

void biquadFilterInitNotch(biquadFilter_t *filter, float filterFreq, float Q, float dt)
{
    biquadFilterInitArrayNotch(filter, filterFreq, dt, Q, 1);
}

void biquadFilterUpdateNotch(void *filter, float filterFreq, float dt, float Q)
{
    biquadFilter_t *f = (biquadFilter_t*)filter;
    biquadFilterGainsNotch(&f->gains, filterFreq, dt, Q);
}

void biquadFilterInitArrayNotchWeighted(void *filter, float filterFreq, float dt, float Q, float weight, int count)
{
    biquadFilter_t *f = (biquadFilter_t*)filter;

    biquadFilterGainsNotchWeighted(&f->gains, filterFreq, dt, Q, weight);

    // Split state arrays to suppress compiler warnings
    float *x1_state = &f->x1[0];
    float *x2_state = &f->x2[0];
    float *y1_state = &f->y1[0];
    float *y2_state = &f->y2[0];

    // Zero initial samples
    for (int i = 0; i < count; i++) {
        x1_state[i] = 0.0f;
        x2_state[i] = 0.0f;
        y1_state[i] = 0.0f;
        y2_state[i] = 0.0f;
    }
}

void biquadFilterInitNotchWeighted(biquadFilter_t *filter, float filterFreq, float dt, float Q, float weight)
{
    biquadFilterInitArrayNotchWeighted(filter, filterFreq, dt, Q, weight, 1);
}

void biquadFilterUpdateNotchWeighted(void *filter, float filterFreq, float dt, float Q, float weight)
{
    biquadFilter_t *f = (biquadFilter_t*)filter;
    biquadFilterGainsNotchWeighted(&f->gains, filterFreq, dt, Q, weight);
}

/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
FAST_CODE float biquadFilterApplyArrayDF1(void *filter, float input, int element)
{
    biquadFilter_t *f = (biquadFilter_t*)filter;

    float b0 = f->gains.b0; float b1 = f->gains.b1; float b2 = f->gains.b2;
    float a1 = f->gains.a1; float a2 = f->gains.a2;

    const float result = b0 * input + b1 * f->x1[element] + b2 * f->x2[element]
                        - a1 * f->y1[element] - a2 * f->y2[element];

    /* shift x1 to x2, input to x1 */
    f->x2[element] = f->x1[element];
    f->x1[element] = input;

    /* shift y1 to y2, result to y1 */
    f->y2[element] = f->y1[element];
    f->y1[element] = result;

    return result;
}

FAST_CODE float biquadFilterApplyDF1(biquadFilter_t *filter, float input)
{
    return biquadFilterApplyArrayDF1(filter, input, 0);
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
FAST_CODE float biquadFilterApplyArray(void *filter, float input, int element)
{
    biquadFilter_t *f = (biquadFilter_t*)filter;

    float b0 = f->gains.b0; float b1 = f->gains.b1; float b2 = f->gains.b2;
    float a1 = f->gains.a1; float a2 = f->gains.a2;

    // suppress compiler warnings
    float *x1_state = &f->x1[0];
    float *x2_state = &f->x2[0];

    const float result = b0 * input + x1_state[element];

    x1_state[element] = b1 * input - a1 * result + x2_state[element];
    x2_state[element] = b2 * input - a2 * result;

    return result;
}

FAST_CODE float biquadFilterApply(biquadFilter_t *filter, float input)
{
    return biquadFilterApplyArray(filter, input, 0);
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
