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

// PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
#define CUTOFF_CORRECTION_PT2 1.553773974f
#define CUTOFF_CORRECTION_PT3 1.961459177f

// NULL filter

float nullFilterApply(filter_t *filter, float input)
{
    UNUSED(filter);
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

// SVF filters

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

void butterworthFilterInit(butterworthFilter_t *filter, float filterFreq, float dt)
{
    butterworthFilterUpdate(filter, filterFreq, dt);

    // zero initial samples
    filter->ic2 = 0.0f;
    filter->ic1 = 0.0f;
}

FAST_CODE void butterworthFilterUpdate(butterworthFilter_t *filter, float filterFreq, float dt)
{
    #define BUTTERWORTH_Q 1.41421356237f // Q is always set to this value, we can remove Q
    float sn, cs;
    sincosf_approx(M_PIf * filterFreq * dt, &sn, &cs);

    float f = sn / cs;
    float inv_denom = 1.0f / (1.0f + f * (f + BUTTERWORTH_Q));
    filter->a1 = inv_denom;
    filter->a2 = f * inv_denom;
    filter->f = f;
}

// Computes a SVF filter in TPT form on a sample
FAST_CODE float butterworthFilterApply(butterworthFilter_t *filter, float input)
{
    float a1 = filter->a1;
    float a2 = filter->a2;
    float f = filter->f;
    float ic1 = filter->ic1;
    float ic2 = filter->ic2;

    float v3 = input - ic2;
    float v1 = a1 * ic1 + a2 * v3;
    float v2 = ic2 + f * v1;

    filter->ic1 = 2.0f * v1 - ic1;
    filter->ic2 = 2.0f * v2 - ic2;

    return v2;
}

void notchInit(notchFilter_t *filter, float filterFreq, float dt, float Q)
{
    notchUpdate(filter, filterFreq, dt, Q);

    filter->ic1q = 0.0f; // State 1 scaled by q
    filter->ic2 = 0.0f;  // State 2 (unscaled)
}

FAST_CODE void notchUpdate(notchFilter_t *filter, float filterFreq, float dt, float Q)
{
    float sn, cs;
    sincosf_approx(M_PIf * filterFreq * dt, &sn, &cs);

    const float f = sn / cs;
    const float q = 1.0f / Q; // Still need this for a1/a2
    const float invDenom = 1.0f / (1.0f + f * (f + q));

    filter->a1  = invDenom;
    filter->a2q = (f * invDenom) * q;
    filter->fq  = f * Q; // Division replaced by multiplication
}

FAST_CODE float notchApply(notchFilter_t *filter, float input)
{
    const float a1  = filter->a1;
    const float a2q = filter->a2q;
    const float fq  = filter->fq;

    const float v3 = input - filter->ic2;
    const float v1q = a1 * filter->ic1q + a2q * v3; // This is now (q * BP)
    const float v2 = filter->ic2 + fq * v1q;       // Equivalent to ic2 + f * BP
    // Update scaled states
    filter->ic1q = 2.0f * v1q - filter->ic1q;
    filter->ic2 = 2.0f * v2 - filter->ic2;

    return input - v1q;
}

void rpmNotchInit(rpmNotch_t *filter, float filterFreq, float dt, float Q, float weight)
{
    rpmNotchUpdate(filter, filterFreq, dt, Q, weight);
    // zero initial samples
    for (int i = 0; i < 3; i++) {
        filter->ic1[i] = 0.0f;
        filter->ic2[i] = 0.0f;
    }
}

FAST_CODE void rpmNotchUpdate(rpmNotch_t *filter, float filterFreq, float dt, float Q, float weight)
{
    float sn, cs;
    sincosf_approx(M_PIf * filterFreq * dt, &sn, &cs);

    const float f  = sn / cs;
    const float q  = 1.0f / Q;
    const float a1 = 1.0f / (1.0f + f * (f + q));

    filter->f  = f;
    filter->a1 = a1;
    filter->a2 = f * a1;
    filter->wq = q * weight;
}

FAST_CODE void rpmNotchApply(rpmNotch_t *filter, float input[3])
{
    const float a1 = filter->a1;
    const float a2 = filter->a2;
    const float f  = filter->f;
    const float wq = filter->wq;

    for (int i = 0; i < 3; i++) {
        const float v3 = input[i] - filter->ic2[i];
        const float v1 = a1 * filter->ic1[i] + a2 * v3;
        const float v2 = filter->ic2[i] + f * v1;
        filter->ic1[i] = 2.0f * v1 - filter->ic1[i];
        filter->ic2[i] = 2.0f * v2 - filter->ic2[i];
        input[i] -= wq * v1;
    }
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
