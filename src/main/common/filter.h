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

#define MAX_DENOISE_WINDOW_SIZE 120

typedef struct pt1Filter_s {
    float state;
    float RC;
    float dT;
} pt1Filter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquadFilter_t;

typedef struct dennoisingState_s {
    int filledCount;
    int targetCount;
    int index;
    float movingSum;
    float state[MAX_DENOISE_WINDOW_SIZE];
} denoisingState_t;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_DENOISE
} filterType_e;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

typedef struct firFilter_s {
    float *buf;
    const float *coeffs;
    float movingSum;
    uint8_t index;
    uint8_t count;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilter_t;

typedef struct firFilterInt16_s {
    int16_t *buf;
    const float *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilterInt16_t;


void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float input);
float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff);

void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT);

void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs);
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength);
void firFilterUpdate(firFilter_t *filter, float input);
void firFilterUpdateAverage(firFilter_t *filter, float input);
float firFilterApply(const firFilter_t *filter);
float firFilterCalcPartialAverage(const firFilter_t *filter, uint8_t count);
float firFilterCalcMovingAverage(const firFilter_t *filter);
float firFilterLastInput(const firFilter_t *filter);

void firFilterInt16Init(firFilterInt16_t *filter, int16_t *buf, uint8_t bufLength, const float *coeffs);
void firFilterInt16Init2(firFilterInt16_t *filter, int16_t *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength);
void firFilterInt16Update(firFilterInt16_t *filter, int16_t input);
float firFilterInt16Apply(const firFilterInt16_t *filter);
float firFilterInt16CalcPartialAverage(const firFilterInt16_t *filter, uint8_t count);
float firFilterInt16CalcAverage(const firFilterInt16_t *filter);
int16_t firFilterInt16LastInput(const firFilterInt16_t *filter);
int16_t firFilterInt16Get(const firFilter_t *filter, int index);
void initDenoisingFilter(denoisingState_t *filter, uint8_t gyroSoftLpfHz, uint16_t targetLooptime);
float denoisingFilterUpdate(denoisingState_t *filter, float input);

