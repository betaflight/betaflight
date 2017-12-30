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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"

#ifdef USE_GYRO_DATA_ANALYSE
#include "arm_math.h"

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/time.h"

#include "sensors/gyro.h"
#include "sensors/gyroanalyse.h"

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency of 500 at a window size of 32 gives 16 frequency bins each with a width 31.25Hz
// Eg [0,31), [31,62), [62, 93) etc

#define FFT_WINDOW_SIZE                32  // max for f3 targets
#define FFT_MIN_FREQ                  100  // not interested in filtering frequencies below 100Hz
#define FFT_SAMPLING_RATE            1000  // allows analysis up to 500Hz which is more than motors create
#define FFT_BPF_HZ                    200  // use a bandpass on gyro data to ignore extreme low and extreme high frequencies
#define DYN_NOTCH_WIDTH               100  // just an orientation and start value
#define DYN_NOTCH_CHANGERATE           60  // lower cut does not improve the performance much, higher cut makes it worse...
#define DYN_NOTCH_MIN_CUTOFF          120  // don't cut too deep into low frequencies
#define DYN_NOTCH_MAX_CUTOFF          200  // don't go above this cutoff (better filtering with "constant" delay at higher center frequencies)

#define BIQUAD_Q 1.0f / sqrtf(2.0f)         // quality factor - butterworth

static uint16_t samplingFrequency;          // gyro rate
static uint8_t fftBinCount;
static float fftResolution;                 // hz per bin
static float gyroData[3][FFT_WINDOW_SIZE];  // gyro data used for frequency analysis

static arm_rfft_fast_instance_f32 fftInstance;
static float fftData[FFT_WINDOW_SIZE];
static float rfftData[FFT_WINDOW_SIZE];
static gyroFftData_t fftResult[3];
static uint16_t fftMaxFreq = 0;             // nyquist rate
static uint16_t fftIdx = 0;                 // use a circular buffer for the last FFT_WINDOW_SIZE samples


// accumulator for oversampled data => no aliasing and less noise
static float fftAcc[3] = {0, 0, 0};
static int fftAccCount = 0;
static int fftSamplingScale;

// bandpass filter gyro data
static biquadFilter_t fftGyroFilter[3];

// filter for smoothing frequency estimation
static biquadFilter_t fftFreqFilter[3];

// Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
static float hanningWindow[FFT_WINDOW_SIZE];

void initHanning(void)
{
    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        hanningWindow[i] = (0.5 - 0.5 * cos_approx(2 * M_PIf * i / (FFT_WINDOW_SIZE - 1)));
    }
}

void initGyroData(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
            gyroData[axis][i] = 0;
        }
    }
}

static inline int fftFreqToBin(int freq)
{
    return ((FFT_WINDOW_SIZE / 2 - 1) * freq) / (fftMaxFreq);
}

void gyroDataAnalyseInit(uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    samplingFrequency = 1000000 / targetLooptimeUs;
    fftSamplingScale = samplingFrequency / FFT_SAMPLING_RATE;
    fftMaxFreq = FFT_SAMPLING_RATE / 2;
    fftBinCount = fftFreqToBin(fftMaxFreq) + 1;
    fftResolution = FFT_SAMPLING_RATE / FFT_WINDOW_SIZE;
    arm_rfft_fast_init_f32(&fftInstance, FFT_WINDOW_SIZE);

    initGyroData();
    initHanning();

    // recalculation of filters takes 4 calls per axis => each filter gets updated every 3 * 4 = 12 calls
    // at 4khz gyro loop rate this means 4khz / 4 / 3 = 333Hz => update every 3ms
    float looptime = targetLooptimeUs * 4 * 3;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        fftResult[axis].centerFreq = 200; // any init value
        biquadFilterInitLPF(&fftFreqFilter[axis], DYN_NOTCH_CHANGERATE, looptime);
        biquadFilterInit(&fftGyroFilter[axis], FFT_BPF_HZ, 1000000 / FFT_SAMPLING_RATE, BIQUAD_Q, FILTER_BPF);
    }
}

// used in OSD
const gyroFftData_t *gyroFftData(int axis)
{
    return &fftResult[axis];
}

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
void gyroDataAnalyse(const gyroDev_t *gyroDev, biquadFilter_t *notchFilterDyn)
{
    // if gyro sampling is > 1kHz, accumulate multiple samples
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        fftAcc[axis] += gyroDev->gyroADC[axis];
    }
    fftAccCount++;

    // this runs at 1kHz
    if (fftAccCount == fftSamplingScale) {
        fftAccCount = 0;

        //calculate mean value of accumulated samples
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            float sample = fftAcc[axis] / fftSamplingScale;
            sample = biquadFilterApply(&fftGyroFilter[axis], sample);
            gyroData[axis][fftIdx] = sample;
            if (axis == 0)
                DEBUG_SET(DEBUG_FFT, 2, lrintf(sample * gyroDev->scale));
            fftAcc[axis] = 0;
        }

        fftIdx = (fftIdx + 1) % FFT_WINDOW_SIZE;
    }

    // calculate FFT and update filters
    gyroDataAnalyseUpdate(notchFilterDyn);
}

void stage_rfft_f32(arm_rfft_fast_instance_f32 * S, float32_t * p, float32_t * pOut);
void arm_cfft_radix8by2_f32( arm_cfft_instance_f32 * S, float32_t * p1);
void arm_cfft_radix8by4_f32( arm_cfft_instance_f32 * S, float32_t * p1);
void arm_radix8_butterfly_f32(float32_t * pSrc, uint16_t fftLen, const float32_t * pCoef, uint16_t twidCoefModifier);
void arm_bitreversal_32(uint32_t * pSrc, const uint16_t bitRevLen, const uint16_t * pBitRevTable);

typedef enum {
    STEP_ARM_CFFT_F32,
    STEP_BITREVERSAL,
    STEP_STAGE_RFFT_F32,
    STEP_ARM_CMPLX_MAG_F32,
    STEP_CALC_FREQUENCIES,
    STEP_UPDATE_FILTERS,
    STEP_HANNING,
    STEP_COUNT
} UpdateStep_e;

/*
 * Analyse last gyro data from the last FFT_WINDOW_SIZE milliseconds
 */
void gyroDataAnalyseUpdate(biquadFilter_t *notchFilterDyn)
{
    static int axis = 0;
    static int step = 0;
    arm_cfft_instance_f32 * Sint = &(fftInstance.Sint);

    uint32_t startTime = 0;
    if (debugMode == (DEBUG_FFT_TIME))
        startTime = micros();

    DEBUG_SET(DEBUG_FFT_TIME, 0, step);
    switch (step) {
        case STEP_ARM_CFFT_F32:
        {
            switch (FFT_WINDOW_SIZE / 2) {
            case 16:
                // 16us
                arm_cfft_radix8by2_f32(Sint, fftData);
                break;
            case 32:
                // 35us
                arm_cfft_radix8by4_f32(Sint, fftData);
                break;
            case 64:
                // 70us
                arm_radix8_butterfly_f32(fftData, FFT_WINDOW_SIZE / 2, Sint->pTwiddle, 1);
                break;
            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_BITREVERSAL:
        {
            // 6us
            arm_bitreversal_32((uint32_t*) fftData, Sint->bitRevLength, Sint->pBitRevTable);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            step++;
            FALLTHROUGH;
        }
        case STEP_STAGE_RFFT_F32:
        {
            // 14us
            // this does not work in place => fftData AND rfftData needed
            stage_rfft_f32(&fftInstance, fftData, rfftData);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_ARM_CMPLX_MAG_F32:
        {
            // 8us
            arm_cmplx_mag_f32(rfftData, fftData, fftBinCount);
            DEBUG_SET(DEBUG_FFT_TIME, 2, micros() - startTime);
            step++;
            FALLTHROUGH;
        }
        case STEP_CALC_FREQUENCIES:
        {
            // 13us
            float fftSum = 0;
            float fftWeightedSum = 0;

            fftResult[axis].maxVal = 0;
            // iterate over fft data and calculate weighted indexes
            float squaredData;
            for (int i = 0; i < fftBinCount; i++) {
                squaredData = fftData[i] * fftData[i];  //more weight on higher peaks
                fftResult[axis].maxVal = MAX(fftResult[axis].maxVal, squaredData);
                fftSum += squaredData;
                fftWeightedSum += squaredData * (i + 1); // calculate weighted index starting at 1, not 0
            }

            // get weighted center of relevant frequency range (this way we have a better resolution than 31.25Hz)
            if (fftSum > 0) {
                // idx was shifted by 1 to start at 1, not 0
                float fftMeanIndex = (fftWeightedSum / fftSum) - 1;
                // the index points at the center frequency of each bin so index 0 is actually 16.125Hz
                // fftMeanIndex += 0.5;

                // don't go below the minimal cutoff frequency + 10 and don't jump around too much
                float centerFreq;
                centerFreq = constrain(fftMeanIndex * fftResolution, DYN_NOTCH_MIN_CUTOFF + 10, fftMaxFreq);
                centerFreq = biquadFilterApply(&fftFreqFilter[axis], centerFreq);
                centerFreq = constrain(centerFreq, DYN_NOTCH_MIN_CUTOFF + 10, fftMaxFreq);
                fftResult[axis].centerFreq = centerFreq;
                if (axis == 0) {
                    DEBUG_SET(DEBUG_FFT, 3, lrintf(fftMeanIndex * 100));
                }
            }

            DEBUG_SET(DEBUG_FFT_FREQ, axis, fftResult[axis].centerFreq);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_UPDATE_FILTERS:
        {
            // 7us
            // calculate new filter coefficients
            float cutoffFreq = constrain(fftResult[axis].centerFreq - DYN_NOTCH_WIDTH, DYN_NOTCH_MIN_CUTOFF, DYN_NOTCH_MAX_CUTOFF);
            float notchQ = filterGetNotchQApprox(fftResult[axis].centerFreq, cutoffFreq);
            biquadFilterUpdate(&notchFilterDyn[axis], fftResult[axis].centerFreq, gyro.targetLooptime, notchQ, FILTER_NOTCH);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            axis = (axis + 1) % 3;
            step++;
            FALLTHROUGH;
        }
        case STEP_HANNING:
        {
            // 5us
            // apply hanning window to gyro samples and store result in fftData
            // hanning starts and ends with 0, could be skipped for minor speed improvement
            uint8_t ringBufIdx = FFT_WINDOW_SIZE - fftIdx;
            arm_mult_f32(&gyroData[axis][fftIdx], &hanningWindow[0], &fftData[0], ringBufIdx);
            if (fftIdx > 0)
                arm_mult_f32(&gyroData[axis][0], &hanningWindow[ringBufIdx], &fftData[ringBufIdx], fftIdx);

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
        }
    }

    step = (step + 1) % STEP_COUNT;
}

#endif // USE_GYRO_DATA_ANALYSE
