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

/* original work by Rav
 * 2018_07 updated by ctzsnooze to post filter, wider Q, different peak detection
 * coding assistance and advice from DieHertz, Rav, eTracer
 * test pilots icr4sh, UAV Tech, Flint723
 */
#include <stdint.h>

#include "platform.h"

#ifdef USE_GYRO_DATA_ANALYSE
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

#define FFT_BIN_COUNT             (FFT_WINDOW_SIZE / 2)
// for gyro loop >= 4KHz, analyse up to 666Hz, 16 bins each 41.625 Hz wide
#define FFT_SAMPLING_RATE_HZ      1333
// following bin must be at least 2 times previous to indicate start of peak
#define FFT_MIN_BIN_RISE          2
// the desired approimate lower frequency when calculating bin offset
#define FFT_BIN_OFFSET_DESIRED_HZ 90
// lowpass frequency for smoothing notch centre point
#define DYN_NOTCH_SMOOTH_FREQ_HZ  60
// notch centre point will not go below this, must be greater than cutoff, mid of bottom bin
#define DYN_NOTCH_MIN_CENTRE_HZ   125
// lowest allowed notch cutoff frequency
#define DYN_NOTCH_MIN_CUTOFF_HZ   105
// we need 4 steps for each axis
#define DYN_NOTCH_CALC_TICKS      (XYZ_AXIS_COUNT * 4)

static uint16_t FAST_RAM_ZERO_INIT fftSamplingRateHz;
// centre frequency of bandpass that constrains input to FFT
static uint16_t FAST_RAM_ZERO_INIT fftBpfHz;
// Hz per bin
static float FAST_RAM_ZERO_INIT    fftResolution;
// maximum notch centre frequency limited by Nyquist
static uint16_t FAST_RAM_ZERO_INIT dynNotchMaxCentreHz;
static uint8_t  FAST_RAM_ZERO_INIT fftBinOffset;

// Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
static FAST_RAM_ZERO_INIT float hanningWindow[FFT_WINDOW_SIZE];
static FAST_RAM_ZERO_INIT float dynamicNotchCutoff;

void gyroDataAnalyseInit(uint32_t targetLooptimeUs)
{
#ifdef USE_DUAL_GYRO
    static bool gyroAnalyseInitialized;
    if (gyroAnalyseInitialized) {
        return;
    }
    gyroAnalyseInitialized = true;
#endif

    const int gyroLoopRateHz = lrintf((1.0f / targetLooptimeUs) * 1e6f);

    // If we get at least 3 samples then use the default FFT sample frequency
    // otherwise we need to calculate a FFT sample frequency to ensure we get 3 samples (gyro loops < 4K)
    fftSamplingRateHz = MIN((gyroLoopRateHz / 3), FFT_SAMPLING_RATE_HZ);

    fftBpfHz = fftSamplingRateHz / 4;
    fftResolution = (float)fftSamplingRateHz / FFT_WINDOW_SIZE;
    dynNotchMaxCentreHz = fftSamplingRateHz / 2;

    // Calculate the FFT bin offset to try and get the lowest bin used
    // in the center calc close to 90hz
    // > 1333hz = 1, 889hz (2.67K) = 2, 666hz (2K) = 3
    fftBinOffset = MAX(1, lrintf(FFT_BIN_OFFSET_DESIRED_HZ / fftResolution - 1.5f));

    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        hanningWindow[i] = (0.5f - 0.5f * cos_approx(2 * M_PIf * i / (FFT_WINDOW_SIZE - 1)));
    }

    dynamicNotchCutoff = (100.0f - gyroConfig()->dyn_notch_width_percent) / 100;
}

void gyroDataAnalyseStateInit(gyroAnalyseState_t *state, uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    gyroDataAnalyseInit(targetLooptimeUs);

    const uint16_t samplingFrequency = 1000000 / targetLooptimeUs;
    state->maxSampleCount = samplingFrequency / fftSamplingRateHz;
    state->maxSampleCountRcp = 1.f / state->maxSampleCount;

    arm_rfft_fast_init_f32(&state->fftInstance, FFT_WINDOW_SIZE);

    // recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
    // at 4khz gyro loop rate this means 4khz / 4 / 3 = 333Hz => update every 3ms
    // for gyro rate > 16kHz, we have update frequency of 1kHz => 1ms
    const float looptime = MAX(1000000u / fftSamplingRateHz, targetLooptimeUs * DYN_NOTCH_CALC_TICKS);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // any init value
        state->centerFreq[axis] = 200;
        biquadFilterInit(&state->gyroBandpassFilter[axis], fftBpfHz, 1000000 / fftSamplingRateHz, 0.01f * gyroConfig()->dyn_notch_quality, FILTER_BPF);
        biquadFilterInitLPF(&state->detectedFrequencyFilter[axis], DYN_NOTCH_SMOOTH_FREQ_HZ, looptime);
    }
}

void gyroDataAnalysePush(gyroAnalyseState_t *state, const int axis, const float sample)
{
    state->oversampledGyroAccumulator[axis] += sample;
}

static void gyroDataAnalyseUpdate(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn);

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
void gyroDataAnalyse(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn)
{
    // samples should have been pushed by `gyroDataAnalysePush`
    // if gyro sampling is > 1kHz, accumulate multiple samples
    state->sampleCount++;

    // this runs at 1kHz
    if (state->sampleCount == state->maxSampleCount) {
        state->sampleCount = 0;

        // calculate mean value of accumulated samples
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            float sample = state->oversampledGyroAccumulator[axis] * state->maxSampleCountRcp;
            sample = biquadFilterApply(&state->gyroBandpassFilter[axis], sample);

            state->downsampledGyroData[axis][state->circularBufferIdx] = sample;
            if (axis == 0) {
                DEBUG_SET(DEBUG_FFT, 2, lrintf(sample));
            }

            state->oversampledGyroAccumulator[axis] = 0;
        }

        state->circularBufferIdx = (state->circularBufferIdx + 1) % FFT_WINDOW_SIZE;

        // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
        state->updateTicks = DYN_NOTCH_CALC_TICKS;
    }

    // calculate FFT and update filters
    if (state->updateTicks > 0) {
        gyroDataAnalyseUpdate(state, notchFilterDyn);
        --state->updateTicks;
    }
}

void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen, const float32_t *pCoef, uint16_t twidCoefModifier);
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable);

/*
 * Analyse last gyro data from the last FFT_WINDOW_SIZE milliseconds
 */
static FAST_CODE_NOINLINE void gyroDataAnalyseUpdate(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn)
{
    enum {
        STEP_ARM_CFFT_F32,
        STEP_BITREVERSAL,
        STEP_STAGE_RFFT_F32,
        STEP_ARM_CMPLX_MAG_F32,
        STEP_CALC_FREQUENCIES,
        STEP_UPDATE_FILTERS,
        STEP_HANNING,
        STEP_COUNT
    };

    arm_cfft_instance_f32 *Sint = &(state->fftInstance.Sint);

    uint32_t startTime = 0;
    if (debugMode == (DEBUG_FFT_TIME)) {
        startTime = micros();
    }

    DEBUG_SET(DEBUG_FFT_TIME, 0, state->updateStep);
    switch (state->updateStep) {
        case STEP_ARM_CFFT_F32:
        {
            switch (FFT_BIN_COUNT) {
            case 16:
                // 16us
                arm_cfft_radix8by2_f32(Sint, state->fftData);
                break;
            case 32:
                // 35us
                arm_cfft_radix8by4_f32(Sint, state->fftData);
                break;
            case 64:
                // 70us
                arm_radix8_butterfly_f32(state->fftData, FFT_BIN_COUNT, Sint->pTwiddle, 1);
                break;
            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_BITREVERSAL:
        {
            // 6us
            arm_bitreversal_32((uint32_t*) state->fftData, Sint->bitRevLength, Sint->pBitRevTable);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_STAGE_RFFT_F32:
        {
            // 14us
            // this does not work in place => fftData AND rfftData needed
            stage_rfft_f32(&state->fftInstance, state->fftData, state->rfftData);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_ARM_CMPLX_MAG_F32:
        {
            // 8us
            arm_cmplx_mag_f32(state->rfftData, state->fftData, FFT_BIN_COUNT);
            DEBUG_SET(DEBUG_FFT_TIME, 2, micros() - startTime);
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_CALC_FREQUENCIES:
        {
            // 13us
            // calculate FFT centreFreq
            float fftSum = 0;
            float fftWeightedSum = 0;
            bool fftIncreasing = false;

            // iterate over fft data and calculate weighted indices
            for (int i = 1 + fftBinOffset; i < FFT_BIN_COUNT; i++) {
                const float data = state->fftData[i];
                const float prevData = state->fftData[i - 1];

                if (fftIncreasing || data > prevData * FFT_MIN_BIN_RISE) {
                    float cubedData = data * data * data;

                    // add previous bin before first rise
                    if (!fftIncreasing) {
                        cubedData += prevData * prevData * prevData;

                        fftIncreasing = true;
                    }

                    fftSum += cubedData;
                    // calculate weighted index starting at 1, not 0
                    fftWeightedSum += cubedData * (i + 1);
                }
            }

            // get weighted center of relevant frequency range (this way we have a better resolution than 31.25Hz)
            // if no peak, go to highest point to minimise delay
            float centerFreq = dynNotchMaxCentreHz;
            float fftMeanIndex = 0;

            if (fftSum > 0) {
                // idx was shifted by 1 to start at 1, not 0
                fftMeanIndex = (fftWeightedSum / fftSum) - 1;
                // the index points at the center frequency of each bin so index 0 is actually 16.125Hz
                centerFreq = constrain(fftMeanIndex * fftResolution, DYN_NOTCH_MIN_CENTRE_HZ, dynNotchMaxCentreHz);
            }

            centerFreq = biquadFilterApply(&state->detectedFrequencyFilter[state->updateAxis], centerFreq);
            centerFreq = constrain(centerFreq, DYN_NOTCH_MIN_CENTRE_HZ, dynNotchMaxCentreHz);
            state->centerFreq[state->updateAxis] = centerFreq;

            if (state->updateAxis == 0) {
               DEBUG_SET(DEBUG_FFT, 3, lrintf(fftMeanIndex * 100));
            }
            DEBUG_SET(DEBUG_FFT_FREQ, state->updateAxis, state->centerFreq[state->updateAxis]);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_UPDATE_FILTERS:
        {
            // 7us
            // calculate cutoffFreq and notch Q, update notch filter
            const float cutoffFreq = fmax(state->centerFreq[state->updateAxis] * dynamicNotchCutoff, DYN_NOTCH_MIN_CUTOFF_HZ);
            const float notchQ = filterGetNotchQ(state->centerFreq[state->updateAxis], cutoffFreq);
            biquadFilterUpdate(&notchFilterDyn[state->updateAxis], state->centerFreq[state->updateAxis], gyro.targetLooptime, notchQ, FILTER_NOTCH);
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            state->updateAxis = (state->updateAxis + 1) % XYZ_AXIS_COUNT;
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_HANNING:
        {
            // 5us
            // apply hanning window to gyro samples and store result in fftData
            // hanning starts and ends with 0, could be skipped for minor speed improvement
            const uint8_t ringBufIdx = FFT_WINDOW_SIZE - state->circularBufferIdx;
            arm_mult_f32(&state->downsampledGyroData[state->updateAxis][state->circularBufferIdx], &hanningWindow[0], &state->fftData[0], ringBufIdx);
            if (state->circularBufferIdx > 0) {
                arm_mult_f32(&state->downsampledGyroData[state->updateAxis][0], &hanningWindow[ringBufIdx], &state->fftData[ringBufIdx], state->circularBufferIdx);
            }

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
        }
    }

    state->updateStep = (state->updateStep + 1) % STEP_COUNT;
}
#endif // USE_GYRO_DATA_ANALYSE
