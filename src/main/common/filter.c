/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/filter.h"
#include "common/maths.h"

#define M_LN2_FLOAT 0.69314718055994530942f
#define M_PI_FLOAT  3.14159265358979323846f

#define BIQUAD_BANDWIDTH 1.9f     /* bandwidth in octaves */
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/

// PT1 Low Pass filter

void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT)
{
    filter->RC = 1.0f / ( 2.0f * M_PI_FLOAT * f_cut );
    filter->dT = dT;
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);
    return filter->state;
}

float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT)
{
    // Pre calculate and store RC
    if (!filter->RC) {
        filter->RC = 1.0f / ( 2.0f * M_PI_FLOAT * f_cut );
        filter->dT = dT;
    }

    filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);

    return filter->state;
}

float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff) {
    float octaves = log2f((float) centerFreq  / (float) cutoff) * 2;
    return sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1);
}

/* sets up a biquad Filter */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF);
}
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType)
{
    // setup variables
    const float sampleRate = 1 / ((float)refreshRate * 0.000001f);
    const float omega = 2 * M_PI_FLOAT * filterFreq / sampleRate;
    const float sn = sinf(omega);
    const float cs = cosf(omega);
    const float alpha = sn / (2 * Q);

    float b0, b1, b2, a0, a1, a2;

    switch (filterType) {
        case FILTER_LPF:
            b0 = (1 - cs) / 2;
            b1 = 1 - cs;
            b2 = (1 - cs) / 2;
            a0 = 1 + alpha;
            a1 = -2 * cs;
            a2 = 1 - alpha;
            break;
        case FILTER_NOTCH:
            b0 =  1;
            b1 = -2 * cs;
            b2 =  1;
            a0 =  1 + alpha;
            a1 = -2 * cs;
            a2 =  1 - alpha;
            break;
    }

    // precompute the coefficients
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // zero initial samples
    filter->d1 = filter->d2 = 0;
}

/* Computes a biquad_t filter on a sample */
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->d1;
    filter->d1 = filter->b1 * input - filter->a1 * result + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * result;
    return result;
}

/*
 * FIR filter
 */
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength)
{
    filter->buf = buf;
    filter->bufLength = bufLength;
    filter->coeffs = coeffs;
    filter->coeffsLength = coeffsLength;
    memset(filter->buf, 0, sizeof(float) * filter->bufLength);
}

/*
 * FIR filter initialisation
 * If FIR filter is just used for averaging, coeffs can be set to NULL
 */
void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs)
{
    firFilterInit2(filter, buf, bufLength, coeffs, bufLength);
}

void firFilterUpdate(firFilter_t *filter, float input)
{
    memmove(&filter->buf[1], &filter->buf[0], (filter->bufLength-1) * sizeof(input));
    filter->buf[0] = input;
}

float firFilterApply(const firFilter_t *filter)
{
    float ret = 0.0f;
    for (int ii = 0; ii < filter->coeffsLength; ++ii) {
        ret += filter->coeffs[ii] * filter->buf[ii];
    }
    return ret;
}

float firFilterCalcPartialAverage(const firFilter_t *filter, uint8_t count)
{
    float ret = 0.0f;
    for (int ii = 0; ii < count; ++ii) {
        ret += filter->buf[ii];
    }
    return ret / count;
}

float firFilterCalcAverage(const firFilter_t *filter)
{
    return firFilterCalcPartialAverage(filter, filter->coeffsLength);
}

float firFilterLastInput(const firFilter_t *filter)
{
    return filter->buf[0];
}

float firFilterGet(const firFilter_t *filter, int index)
{
    return filter->buf[index];
}

/*
 *  int16_t based FIR filter
 *  Can be directly updated from devices that produce 16-bit data, eg gyros and accelerometers
 */
void firFilterInt16Init2(firFilterInt16_t *filter, int16_t *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength)
{
    filter->buf = buf;
    filter->bufLength = bufLength;
    filter->coeffs = coeffs;
    filter->coeffsLength = coeffsLength;
    memset(filter->buf, 0, sizeof(int16_t) * filter->bufLength);
}

/*
 * FIR filter initialisation
 * If FIR filter is just used for averaging, coeffs can be set to NULL
 */
void firFilterInt16Init(firFilterInt16_t *filter, int16_t *buf, uint8_t bufLength, const float *coeffs)
{
    firFilterInt16Init2(filter, buf, bufLength, coeffs, bufLength);
}

void firFilterInt16Update(firFilterInt16_t *filter, int16_t input)
{
    memmove(&filter->buf[1], &filter->buf[0], (filter->bufLength-1) * sizeof(input));
    filter->buf[0] = input;
}

float firFilterInt16Apply(const firFilterInt16_t *filter)
{
    float ret = 0.0f;
    for (int ii = 0; ii < filter->coeffsLength; ++ii) {
        ret += filter->coeffs[ii] * filter->buf[ii];
    }
    return ret;
}

float firFilterInt16CalcPartialAverage(const firFilterInt16_t *filter, uint8_t count)
{
    float ret = 0;
    for (int ii = 0; ii < count; ++ii) {
        ret += filter->buf[ii];
    }
    return ret / count;
}

float firFilterInt16CalcAverage(const firFilterInt16_t *filter)
{
    return firFilterInt16CalcPartialAverage(filter, filter->coeffsLength);
}

int16_t firFilterInt16LastInput(const firFilterInt16_t *filter)
{
    return filter->buf[0];
}

int16_t firFilterInt16Get(const firFilter_t *filter, int index)
{
    return filter->buf[index];
}

