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

// Biquad filter

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

void butterworthFilterInit(butterworthFilter_t *filter, float filterFreq, float dt)
{
    butterworthFilterUpdate(filter, filterFreq, dt);

#ifndef SVF_ALTERNATIVE
    // zero initial samples
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
#else
    filter->low = 0.0f;
    filter->band = 0.0f;
#endif
}

#define BUTTERWORTH_Q (1.0f / sqrtf(2.0f))     /* quality factor - 2nd order butterworth*/
#define BUTTERWORTH_ALPHA_MULTIPLIER (1.0f / (2.0f * BUTTERWORTH_Q))

FAST_CODE void butterworthFilterUpdate(butterworthFilter_t *filter, float filterFreq, float dt)
{
#ifndef SVF_ALTERNATIVE
    // setup variables
    const float omega = 2.0f * M_PIf * filterFreq * dt;
    float sn, cs;
    sincosf_approx(omega, &sn, &cs);
    const float alpha = sn * BUTTERWORTH_ALPHA_MULTIPLIER;
    const float a0inv = 1.0f / (1.0f + alpha);

    // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
    // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
    filter->b1 = (1.0f - cs) * a0inv;
//    filter->b0 = filter->b1 * 0.5f;
//    filter->b2 = filter->b0;
    filter->a1 = -2.0f * cs * a0inv;
    filter->a2 = (1.0f - alpha) * a0inv;
#else
    filter->f = 2.0f * sin_approx(M_PIf * filterFreq * dt);
    filter->q = 1.0f / BUTTERWORTH_Q;
#endif
}

// Computes a biquad filter in direct form 1 on a sample (slower but can handle changes in coefficients)
FAST_CODE float butterworthFilterApplyMoving(butterworthFilter_t *filter, float input)
{
#ifndef SVF_ALTERNATIVE
    // compute result
    const float result = filter->b1 * (filter->x1 + 0.5f * (input + filter->x2)) - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    // shift x1 to x2, input to x1
    filter->x2 = filter->x1;
    filter->x1 = input;

    // shift y1 to y2, result to y1
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
#else
    const float low  = filter->low + filter->f * filter->band;
    const float high = input - low - filter->q * filter->band;
    filter->band = filter->f * high + filter->band;
    filter->low  = low;
    return low;
#endif
}

// Computes a biquad filter in direct form 2 on a sample (faster but can't handle changes in coefficients)
FAST_CODE float butterworthFilterApplyStatic(butterworthFilter_t *filter, float input)
{
#ifndef SVF_ALTERNATIVE
    // compute result
    const float b1half_input = filter->b1 * 0.5f * input;  // b0*x[n], computed once
    const float result = b1half_input + filter->x1;

    filter->x1 = 2.0f * b1half_input - filter->a1 * result + filter->x2;  // b1*x[n] = 2 * b1half_input
    filter->x2 = b1half_input - filter->a2 * result;

    return result;
#else
    const float low  = filter->low + filter->f * filter->band;
    const float high = input - low - filter->q * filter->band;
    filter->band = filter->f * high + filter->band;
    filter->low  = low;
    return low;
#endif
}

void notchInit(notchFilter_t *filter, float filterFreq, float dt, float Q)
{
    notchUpdate(filter, filterFreq, dt, Q);

#ifndef SVF_ALTERNATIVE
    // zero initial samples
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
#else
    filter->low = 0.0f;
    filter->band = 0.0f;
#endif
}

FAST_CODE void notchUpdate(notchFilter_t *filter, float filterFreq, float dt, float Q)
{
#ifndef SVF_ALTERNATIVE
    const float omega = 2.0f * M_PIf * filterFreq * dt;
    float sn, cs;
    sincosf_approx(omega, &sn, &cs);
    const float alpha = sn / (2.0f * Q);
    const float a0inv = 1.0f / (1.0f + alpha);
    filter->b0 = a0inv;                        // b0 == b2, drop b2
    filter->a1 = -2.0f * cs * a0inv;           // b1 == a1, drop b1
    filter->a2 = (1.0f - alpha) * a0inv;
#else
    float sn, cs;
    sincosf_approx(M_PIf * filterFreq * dt, &sn, &cs);
    filter->f = sn / cs;
    filter->q = 1.0f / Q;
#endif
}

FAST_CODE float notchApplyMoving(notchFilter_t *filter, float input)
{
#ifndef SVF_ALTERNATIVE
    const float b0 = filter->b0;
    const float a1 = filter->a1;
    const float a2 = filter->a2;

    const float output = b0 * (input + filter->x2)   // b0 == b2
                       + a1 * (filter->x1 - filter->y1)  // b1 == a1
                       - a2 * filter->y2;
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
#else
    const float low  = filter->low + filter->f * filter->band;
    const float high = input - low - filter->q * filter->band;
    filter->band = filter->f * high + filter->band;
    filter->low  = low;
    return high + low;
#endif
}

FAST_CODE float notchApplyStatic(notchFilter_t *filter, float input)
{
#ifndef SVF_ALTERNATIVE
    const float b0input = filter->b0 * input;      // reused in w2
    const float output = b0input + filter->x1;
    const float a1term = filter->a1 * (input - output);  // a1*(x[n] - y[n]), b1==a1
    filter->x1 = a1term + filter->x2;
    filter->x2 = b0input - filter->a2 * output;
    return output;
#else
    const float low  = filter->low + filter->f * filter->band;
    const float high = input - low - filter->q * filter->band;
    filter->band = filter->f * high + filter->band;
    filter->low  = low;
    return high + low;
#endif
}

void rpmNotchInit(rpmNotch_t *filter, float filterFreq, float dt, float Q, float weight)
{
    rpmNotchUpdate(filter, filterFreq, dt, Q, weight);
    // zero initial samples
    for (int i = 0; i < 3; i++) {
#ifndef SVF_ALTERNATIVE
        filter->x1[i] = 0.0f;
        filter->x2[i] = 0.0f;
        filter->y1[i] = 0.0f;
        filter->y2[i] = 0.0f;
#else
        filter->low[i] = 0.0f;
        filter->band[i] = 0.0f;
#endif
    }
}

FAST_CODE void rpmNotchUpdate(rpmNotch_t *filter, float filterFreq, float dt, float Q, float weight)
{
#ifndef SVF_ALTERNATIVE
    const float omega = 2.0f * M_PIf * filterFreq * dt;
    float sn, cs;
    sincosf_approx(omega, &sn, &cs);
    const float alpha = sn / (2.0f * Q);
    const float a0inv = 1.0f / (1.0f + alpha);
    const float weightRecip = 1.0f - weight;

    const float b0raw = a0inv;
    const float a2raw = (1.0f - alpha) * a0inv;

    filter->a1 = -2.0f * cs * a0inv;           // b1 == a1, always, drop b1
    filter->b0 = weight * b0raw + weightRecip;
    filter->b2 = weight * b0raw + weightRecip * a2raw;  // b2_raw == b0_raw
    filter->a2 = a2raw;                         // unweighted, no change needed
#else
    float sn, cs;
    sincosf_approx(M_PIf * filterFreq * dt, &sn, &cs);
    filter->f = sn / cs;
    filter->q = 1.0f / Q;
    filter->weight = weight;
#endif
}

FAST_CODE void rpmNotchApply(rpmNotch_t *filter, float input[3])
{
#ifndef SVF_ALTERNATIVE
    const float b0 = filter->b0;
    const float b2 = filter->b2;
    const float a1 = filter->a1;   // also serves as b1
    const float a2 = filter->a2;

    for (int i = 0; i < 3; i++) {
        const float output = b0 * input[i] + a1 * (filter->x1[i] - filter->y1[i])
                           + b2 * filter->x2[i] - a2 * filter->y2[i];
        filter->x2[i] = filter->x1[i];
        filter->x1[i] = input[i];
        filter->y2[i] = filter->y1[i];
        filter->y1[i] = output;
        input[i] = output;
    }
#else
    const float f = filter->f;
    const float q = filter->q;
    const float weight = filter->weight;
    const float weightRecip = 1.0f - weight;

    for (int i = 0; i < 3; i++) {
        const float low  = filter->low[i] + f * filter->band[i];
        const float high = input[i] - low - q * filter->band[i];
        filter->band[i] = f * high + filter->band[i];
        filter->low[i]  = low;
        input[i] = (high + low) * weight + input[i] * weightRecip;
    }
#endif
}

//FAST_CODE void rpmNotchUpdate(rpmNotch_t *filter, float filterFreq, float dt, float Q)
//{
//    const float omega = 2.0f * M_PIf * filterFreq * dt;
//    float sn, cs;
//    sincosf_approx(omega, &sn, &cs);
//    const float alpha = sn / (2.0f * Q);
//    const float a0inv = 1.0f / (1.0f + alpha);
//    filter->b0 = a0inv;
//    filter->a1 = -2.0f * cs * a0inv;
//    filter->a2 = (1.0f - alpha) * a0inv;
//    // b1 == a1, b2 == b0, no longer need to store them
//}

// TODO test this for speed
//FAST_CODE void rpmNotchApply(rpmNotch_t* filter, float input[3])
//{
//    const float b0 = filter->b0;
//    const float a1 = filter->a1;   // == b1
//    const float a2 = filter->a2;
//    const float weight = filter->weight;
//    const float weightRecip = 1.0f - weight;
//
//    for (int i = 0; i < 3; i++) {
//        const float output = b0 * (input[i] + filter->x2[i])   // b0==b2, same trick as butterworth
//                           + a1 * (filter->x1[i] - filter->y1[i])  // b1==a1
//                           - a2 * filter->y2[i];
//        filter->x2[i] = filter->x1[i];
//        filter->x1[i] = input[i];
//        filter->y2[i] = filter->y1[i];
//        filter->y1[i] = output;
//        input[i] = output * weight + input[i] * weightRecip;
//    }
//}

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
