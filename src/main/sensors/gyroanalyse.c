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

#define FFT_WINDOW_SIZE       32  // max for f3 targets
#define FFT_BIN_COUNT         (FFT_WINDOW_SIZE / 2)
#define FFT_MIN_FREQ          100  // not interested in filtering frequencies below 100Hz
#define FFT_SAMPLING_RATE     1000  // allows analysis up to 500Hz which is more than motors create
#define FFT_MAX_FREQUENCY     (FFT_SAMPLING_RATE / 2) // nyquist rate
#define FFT_BPF_HZ            200  // use a bandpass on gyro data to ignore extreme low and extreme high frequencies
#define FFT_RESOLUTION        ((float)FFT_SAMPLING_RATE / FFT_WINDOW_SIZE) // hz per bin
#define DYN_NOTCH_WIDTH       100  // just an orientation and start value
#define DYN_NOTCH_CHANGERATE  60  // lower cut does not improve the performance much, higher cut makes it worse...
#define DYN_NOTCH_MIN_CUTOFF  120  // don't cut too deep into low frequencies
#define DYN_NOTCH_MAX_CUTOFF  200  // don't go above this cutoff (better filtering with "constant" delay at higher center frequencies)
#define DYN_NOTCH_CALC_TICKS  (XYZ_AXIS_COUNT * 4) // we need 4 steps for each axis

#define BIQUAD_Q 1.0f / sqrtf(2.0f)         // quality factor - butterworth

static FAST_RAM_ZERO_INIT uint16_t fftSamplingScale;

// gyro data used for frequency analysis
static float FAST_RAM_ZERO_INIT gyroData[XYZ_AXIS_COUNT][FFT_WINDOW_SIZE];

static FAST_RAM_ZERO_INIT arm_rfft_fast_instance_f32 fftInstance;
static FAST_RAM_ZERO_INIT float fftData[FFT_WINDOW_SIZE];
static FAST_RAM_ZERO_INIT float rfftData[FFT_WINDOW_SIZE];
static FAST_RAM_ZERO_INIT gyroFftData_t fftResult[XYZ_AXIS_COUNT];

// use a circular buffer for the last FFT_WINDOW_SIZE samples
static FAST_RAM_ZERO_INIT uint16_t fftIdx;

// bandpass filter gyro data
static FAST_RAM_ZERO_INIT biquadFilter_t fftGyroFilter[XYZ_AXIS_COUNT];

// filter for smoothing frequency estimation
static FAST_RAM_ZERO_INIT biquadFilter_t fftFreqFilter[XYZ_AXIS_COUNT];

// Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
static FAST_RAM_ZERO_INIT float hanningWindow[FFT_WINDOW_SIZE];

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

void gyroDataAnalyseInit(uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    const uint16_t samplingFrequency = 1000000 / targetLooptimeUs;
    fftSamplingScale = samplingFrequency / FFT_SAMPLING_RATE;
    arm_rfft_fast_init_f32(&fftInstance, FFT_WINDOW_SIZE);

    initGyroData();
    initHanning();

    // recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
    // at 4khz gyro loop rate this means 4khz / 4 / 3 = 333Hz => update every 3ms
    // for gyro rate > 16kHz, we have update frequency of 1kHz => 1ms
    const float looptime = MAX(1000000u / FFT_SAMPLING_RATE, targetLooptimeUs * DYN_NOTCH_CALC_TICKS);
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
    // accumulator for oversampled data => no aliasing and less noise
    static FAST_RAM_ZERO_INIT float fftAcc[XYZ_AXIS_COUNT];
    static FAST_RAM_ZERO_INIT uint32_t fftAccCount;

    static FAST_RAM_ZERO_INIT uint32_t gyroDataAnalyseUpdateTicks;

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

        // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
        gyroDataAnalyseUpdateTicks = DYN_NOTCH_CALC_TICKS;
    }

    // calculate FFT and update filters
    if (gyroDataAnalyseUpdateTicks > 0) {
        gyroDataAnalyseUpdate(notchFilterDyn);
        --gyroDataAnalyseUpdateTicks;
    }
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
    static int axis;
    static int step;
    arm_cfft_instance_f32 * Sint = &(fftInstance.Sint);

    uint32_t startTime = 0;
    if (debugMode == (DEBUG_FFT_TIME))
        startTime = micros();

    DEBUG_SET(DEBUG_FFT_TIME, 0, step);
    switch (step) {
        case STEP_ARM_CFFT_F32:
        {
            switch (FFT_BIN_COUNT) {
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
                arm_radix8_butterfly_f32(fftData, FFT_BIN_COUNT, Sint->pTwiddle, 1);
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
            arm_cmplx_mag_f32(rfftData, fftData, FFT_BIN_COUNT);
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
            for (int i = 0; i < FFT_BIN_COUNT; i++) {
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
                centerFreq = constrain(fftMeanIndex * FFT_RESOLUTION, DYN_NOTCH_MIN_CUTOFF + 10, FFT_MAX_FREQUENCY);
                centerFreq = biquadFilterApply(&fftFreqFilter[axis], centerFreq);
                centerFreq = constrain(centerFreq, DYN_NOTCH_MIN_CUTOFF + 10, FFT_MAX_FREQUENCY);
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
            float notchQ = filterGetNotchQ(fftResult[axis].centerFreq, cutoffFreq);
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
