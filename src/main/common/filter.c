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

#include "filter_magic_c.h"

#define BUTTERWORTH_Q (1.0f / sqrtf(2.0f))     /* quality factor - 2nd order butterworth*/

// PTn cutoff correction = 1 / sqrt(2^(1/n) - 1)
#define CUTOFF_CORRECTION_PT2 1.553773974f
#define CUTOFF_CORRECTION_PT3 1.961459177f



float nullFilterApply_cs(const FILTER_COEFFS_TYPE(null) *c, FILTER_STATE_TYPE(null) *s, float input)
{
    UNUSED(s); UNUSED(c);
    return input;
}

DEFINE_FILTER_FN(null, DEFAULT_FILTER_DIMS);

// prototype filters used for PTx
float rcFilterGain(float f_cut, float dT)
{
    float omega = 2.0f * M_PIf * f_cut * dT;
    return omega / (omega + 1.0f);
}

float rcFilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }
    // cutoffHz = 1.0f / (2.0f * M_PIf * delay)
    return dT / (dT + delay);
}

// PT1 Low Pass filter
FAST_CODE_NOINLINE void pt1FilterCoeffsLPF(FILTER_COEFFS_TYPE(pt1) *dst, float f_cut, float dT)
{
    dst->c[0] = rcFilterGain(f_cut, dT);
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
FAST_CODE void pt1FilterCoeffsDelay(FILTER_COEFFS_TYPE(pt1) *dst, float delay, float dT)
{
    dst->c[0] = rcFilterGainFromDelay(delay, dT);
}

FAST_CODE_NOINLINE void pt1FilterCoeffsAlpha(FILTER_COEFFS_TYPE(pt1) *dst, float alpha)
{
    dst->c[0] = alpha;
}

float pt1FilterApply_cs(const FILTER_COEFFS_TYPE(pt1) *c, FILTER_STATE_TYPE(pt1) *s, float input)
{
    s->s[0] += c->c[0] * (input - s->s[0]);
    return s->s[0];
}

DEFINE_FILTER_FN(pt1, DEFAULT_FILTER_DIMS);

// PT2 Low Pass filter

FAST_CODE_NOINLINE void pt2FilterCoeffsLPF(FILTER_COEFFS_TYPE(pt2) *dst, float f_cut, float dT)
{
    dst->c[0] = rcFilterGain(f_cut * CUTOFF_CORRECTION_PT2, dT);
}


// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
FAST_CODE void pt2FilterCoeffsDelay(FILTER_COEFFS_TYPE(pt2) *dst, float delay, float dT)
{
    // cutoffHz = 1.0f / (2.0f * M_PIf * delay * CUTOFF_CORRECTION_PT2)
     dst->c[0] = rcFilterGainFromDelay(delay * CUTOFF_CORRECTION_PT2, dT);
}

float pt2FilterApply_cs(const FILTER_COEFFS_TYPE(pt2) *c, FILTER_STATE_TYPE(pt2) *s, float input)
{
    s->s[0] += c->c[0] * (input - s->s[0]);             // stage 1
    s->s[1] += c->c[0] * (s->s[0] - s->s[1]);           // stage 2
    return s->s[1];
}

DEFINE_FILTER_FN(pt2, DEFAULT_FILTER_DIMS);

FAST_CODE_NOINLINE void pt3FilterCoeffsLPF(FILTER_COEFFS_TYPE(pt3) *dst, float f_cut, float dT)
{
    dst->c[0] = rcFilterGain(f_cut * CUTOFF_CORRECTION_PT3, dT);
}

FAST_CODE void pt3FilterCoeffsDelay(FILTER_COEFFS_TYPE(pt3) *dst, float delay, float dT)
{
    // cutoffHz = 1.0f / (2.0f * M_PIf * delay * CUTOFF_CORRECTION_PT3)
    dst->c[0] = rcFilterGainFromDelay(delay * CUTOFF_CORRECTION_PT3, dT);
}

float pt3FilterApply_cs(const FILTER_COEFFS_TYPE(pt3) *c, FILTER_STATE_TYPE(pt3) *s, float input)
{
    s->s[0] += c->c[0] * (input - s->s[0]);            // stage 1
    s->s[1] += c->c[0] * (s->s[0] - s->s[1]);           // stage 2
    s->s[2] += c->c[0] * (s->s[1] - s->s[2]);           // stage 3
    return s->s[2];
}

DEFINE_FILTER_FN(pt3, DEFAULT_FILTER_DIMS);


// Biquad filter

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

FAST_CODE void biquadFilterCoeffsLPF(FILTER_COEFFS_TYPE(biquad) *c, float filterFreq, float dt)
{
    // setup variables
    float omega = 2.0f * M_PIf * filterFreq * dt;
    float sn, cs;
    sincosf_approx(omega, &sn, &cs);
    const float alpha = sn / (2.0f * BUTTERWORTH_Q);
    const float invA0 = 1.0f / (1.0f + alpha);

     // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
     // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
     c->c[biquad_b1] = (1 - cs) * invA0;
     c->c[biquad_b0] = c->c[biquad_b1] * 0.5f;
     c->c[biquad_b2] = c->c[biquad_b0];
     c->c[biquad_a1] = (-2 * cs) * invA0;
     c->c[biquad_a2] = (1 - alpha) * invA0;
}

FAST_CODE void biquadFilterCoeffsNotch(FILTER_COEFFS_TYPE(biquad) *c, float filterFreq, float dt, float q)
{
    // setup variables
    float omega = 2.0f * M_PIf * filterFreq * dt;
    float sn, cs;
    sincosf_approx(omega, &sn, &cs);
    const float alpha = sn / (2.0f * q);
    const float invA0 = 1.0f / (1.0f + alpha);

    c->c[biquad_b0] = invA0;
    c->c[biquad_b1] = (-2 * cs) * invA0;
    c->c[biquad_b2] = invA0;
    c->c[biquad_a1] = c->c[biquad_b1];
    c->c[biquad_a2] = (1 - alpha) * invA0;
}

// update filter coeffs to apply weiht to filter ( y = weight * filter(x) + (1 - weight) * x )
FAST_CODE void biquadFilterCoeffsApplyWeight(FILTER_COEFFS_TYPE(biquad) *c, float weight)
{
    // apply weight
    float weightRecip = 1.0f - weight;

    c->c[biquad_b0] = weight * c->c[biquad_b0] + weightRecip;
    c->c[biquad_b1] = weight * c->c[biquad_b1] + weightRecip * c->c[biquad_a1];
    c->c[biquad_b2] = weight * c->c[biquad_b2] + weightRecip * c->c[biquad_a2];
}

/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
FAST_CODE float biquadDF1FilterApply_cs(FILTER_COEFFS_TYPE(biquad) *c, FILTER_STATE_TYPE(biquadDF1) *s, float input)
{
    const float result = c->c[biquad_b0] * input
                        + c->c[biquad_b1] * s->s[biquad_x1]
                        + c->c[biquad_b2] * s->s[biquad_x2]
                        - c->c[biquad_a1] * s->s[biquad_y1]
                        - c->c[biquad_a2] * s->s[biquad_y2];

    /* shift x1 to x2, input to x1 */
    s->s[biquad_x2] = s->s[biquad_x1];
    s->s[biquad_x1] = input;

    /* shift y1 to y2, result to y1 */
    s->s[biquad_y2] = s->s[biquad_y1];
    s->s[biquad_y1] = result;

    return result;
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
FAST_CODE float biquadFilterApply_cs(FILTER_COEFFS_TYPE(biquad) *c, FILTER_STATE_TYPE(biquad) *s, float input)
{
    const float result = c->c[biquad_b0] * input + s->s[biquad_s1];

    s->s[biquad_s1] = c->c[biquad_b1] * input - c->c[biquad_a1] * result + s->s[biquad_s2];
    s->s[biquad_s2] = c->c[biquad_b2] * input - c->c[biquad_a2] * result;

    return result;
}


DEFINE_FILTER_FN(biquad, DEFAULT_FILTER_DIMS);
DEFINE_FILTER_FN2(biquad, biquadDF1, DEFAULT_FILTER_DIMS);

// Phase Compensator (Lead-Lag-Compensator)

FAST_CODE void phaseCompFilterCoeffsCenPh(FILTER_COEFFS_TYPE(phaseComp) *c, const float centerFreqHz, const float centerPhaseDeg, const float dT)
{
    const float omega = 2.0f * M_PIf * centerFreqHz * dT;
    const float sn = sin_approx(centerPhaseDeg * RAD);
    const float gain = (1 + sn) / (1 - sn);
    const float alpha = (12 - sq(omega)) / (6 * omega * sqrtf(gain));  // approximate prewarping (series expansion)

    const float a0 = 1 / (1 + alpha);

    c->c[phaseComp_b0] = a0 * (1 + alpha * gain);
    c->c[phaseComp_b1] = a0 * (2 - c->c[phaseComp_b0]);
    c->c[phaseComp_a1] = a0 * (1 - alpha);
}

FAST_CODE float phaseCompFilterApply_cs(const FILTER_COEFFS_TYPE(phaseComp) *c, FILTER_STATE_TYPE(phaseComp) *s, const float input)
{
    // compute result
    const float result = c->c[phaseComp_b0] * input
                        + c->c[phaseComp_b1] * s->s[phaseComp_x1]
                        - c->c[phaseComp_a1] * s->s[phaseComp_y1];

    // shift input to x1 and result to y1
    s->s[phaseComp_x1] = input;
    s->s[phaseComp_y1] = result;

    return result;
}

DEFINE_FILTER_FN(phaseComp, (1));

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
