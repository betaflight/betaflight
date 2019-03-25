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

#include "fc/core.h"

// The FFT splits the frequency domain into an number of bins
// A sampling frequency of 1000 and max frequency of 500 at a window size of 32 gives 16 frequency bins each 31.25Hz wide
// Eg [0,31), [31,62), [62, 93) etc
// for gyro loop >= 4KHz, sample rate 2000 defines FFT range to 1000Hz, 16 bins each 62.5 Hz wide
// NB  FFT_WINDOW_SIZE is set to 32 in gyroanalyse.h
#define FFT_BIN_COUNT             (FFT_WINDOW_SIZE / 2)
// smoothing frequency for FFT centre frequency
#define DYN_NOTCH_SMOOTH_FREQ_HZ  50
// we need 4 steps for each axis
#define DYN_NOTCH_CALC_TICKS      (XYZ_AXIS_COUNT * 4)

#define DYN_NOTCH_OSD_MIN_THROTTLE 20

static uint16_t FAST_RAM_ZERO_INIT   fftSamplingRateHz;
static float FAST_RAM_ZERO_INIT      fftResolution;
static uint8_t FAST_RAM_ZERO_INIT    fftStartBin;
static uint16_t FAST_RAM_ZERO_INIT   dynNotchMaxCtrHz;
static uint8_t dynamicFilterRange;
static float FAST_RAM_ZERO_INIT      dynNotchQ;
static float FAST_RAM_ZERO_INIT      dynNotch1Ctr;
static float FAST_RAM_ZERO_INIT      dynNotch2Ctr;
static uint16_t FAST_RAM_ZERO_INIT   dynNotchMinHz;
static bool FAST_RAM dualNotch = true;
static uint16_t FAST_RAM_ZERO_INIT dynNotchMaxFFT;

// Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
static FAST_RAM_ZERO_INIT float hanningWindow[FFT_WINDOW_SIZE];

void gyroDataAnalyseInit(uint32_t targetLooptimeUs)
{
#ifdef USE_MULTI_GYRO
    static bool gyroAnalyseInitialized;
    if (gyroAnalyseInitialized) {
        return;
    }
    gyroAnalyseInitialized = true;
#endif

    dynamicFilterRange = gyroConfig()->dyn_notch_range;
    fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_LOW;
    dynNotch1Ctr = 1 - gyroConfig()->dyn_notch_width_percent / 100.0f;
    dynNotch2Ctr = 1 + gyroConfig()->dyn_notch_width_percent / 100.0f;
    dynNotchQ = gyroConfig()->dyn_notch_q / 100.0f;
    dynNotchMinHz = gyroConfig()->dyn_notch_min_hz;

    if (gyroConfig()->dyn_notch_width_percent == 0) {
        dualNotch = false;
    }

    if (dynamicFilterRange == DYN_NOTCH_RANGE_AUTO) {
        if (gyroConfig()->dyn_lpf_gyro_max_hz > 333) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_MEDIUM;
        }
        if (gyroConfig()->dyn_lpf_gyro_max_hz > 610) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_HIGH;
        }
    } else {
        if (dynamicFilterRange == DYN_NOTCH_RANGE_HIGH) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_HIGH;
        }
        else if (dynamicFilterRange == DYN_NOTCH_RANGE_MEDIUM) {
            fftSamplingRateHz = DYN_NOTCH_RANGE_HZ_MEDIUM;
        }
    }
    // If we get at least 3 samples then use the default FFT sample frequency
    // otherwise we need to calculate a FFT sample frequency to ensure we get 3 samples (gyro loops < 4K)
    const int gyroLoopRateHz = lrintf((1.0f / targetLooptimeUs) * 1e6f);
    
    fftSamplingRateHz = MIN((gyroLoopRateHz / 3), fftSamplingRateHz);

    fftResolution = (float)fftSamplingRateHz / FFT_WINDOW_SIZE;

    fftStartBin = dynNotchMinHz / lrintf(fftResolution);

    dynNotchMaxCtrHz = fftSamplingRateHz / 2; //Nyquist

    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        hanningWindow[i] = (0.5f - 0.5f * cos_approx(2 * M_PIf * i / (FFT_WINDOW_SIZE - 1)));
    }
}

void gyroDataAnalyseStateInit(gyroAnalyseState_t *state, uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    // *** can this next line be removed ??? ***
    gyroDataAnalyseInit(targetLooptimeUs);

    const uint16_t samplingFrequency = 1000000 / targetLooptimeUs;
    state->maxSampleCount = samplingFrequency / fftSamplingRateHz;
    state->maxSampleCountRcp = 1.f / state->maxSampleCount;

    arm_rfft_fast_init_f32(&state->fftInstance, FFT_WINDOW_SIZE);

//    recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
//    at 4khz gyro loop rate this means 4khz / 4 / 3 = 333Hz => update every 3ms
//    for gyro rate > 16kHz, we have update frequency of 1kHz => 1ms
    const float looptime = MAX(1000000u / fftSamplingRateHz, targetLooptimeUs * DYN_NOTCH_CALC_TICKS);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // any init value
        state->centerFreq[axis] = dynNotchMaxCtrHz;
        state->prevCenterFreq[axis] = dynNotchMaxCtrHz;
        biquadFilterInitLPF(&state->detectedFrequencyFilter[axis], DYN_NOTCH_SMOOTH_FREQ_HZ, looptime);
    }
}

void gyroDataAnalysePush(gyroAnalyseState_t *state, const int axis, const float sample)
{
    state->oversampledGyroAccumulator[axis] += sample;
}

static void gyroDataAnalyseUpdate(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn, biquadFilter_t *notchFilterDyn2);

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
void gyroDataAnalyse(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn, biquadFilter_t *notchFilterDyn2)
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
        gyroDataAnalyseUpdate(state, notchFilterDyn, notchFilterDyn2);
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
static FAST_CODE_NOINLINE void gyroDataAnalyseUpdate(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn, biquadFilter_t *notchFilterDyn2)
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
            bool fftIncreased = false;
            float dataMax = 0;
            uint8_t binStart = 0;
            uint8_t binMax = 0;
            //for bins after initial decline, identify start bin and max bin 
            for (int i = fftStartBin; i < FFT_BIN_COUNT; i++) {
                if (fftIncreased || (state->fftData[i] > state->fftData[i - 1])) {
                    if (!fftIncreased) {
                        binStart = i; // first up-step bin
                        fftIncreased = true;
                    }
                    if (state->fftData[i] > dataMax) {
                        dataMax = state->fftData[i];
                        binMax = i;  // tallest bin
                    }
                }
            }
            // accumulate fftSum and fftWeightedSum from peak bin, and shoulder bins either side of peak
            float cubedData = state->fftData[binMax] * state->fftData[binMax] * state->fftData[binMax];
            float fftSum = cubedData;
            float fftWeightedSum = cubedData * (binMax + 1);
            // accumulate upper shoulder
            for (int i = binMax; i < FFT_BIN_COUNT - 1; i++) {
                if (state->fftData[i] > state->fftData[i + 1]) {
                    cubedData = state->fftData[i] * state->fftData[i] * state->fftData[i];
                    fftSum += cubedData;
                    fftWeightedSum += cubedData * (i + 1);
                } else {
                break;
                }
            }
            // accumulate lower shoulder
            for (int i = binMax; i > binStart + 1; i--) {
                if (state->fftData[i] > state->fftData[i - 1]) {
                    cubedData = state->fftData[i] * state->fftData[i] * state->fftData[i];
                    fftSum += cubedData;
                    fftWeightedSum += cubedData * (i + 1);
                } else {
                break;
                }
            }
            // get weighted center of relevant frequency range (this way we have a better resolution than 31.25Hz)
            float centerFreq = dynNotchMaxCtrHz;
            float fftMeanIndex = 0;
             // idx was shifted by 1 to start at 1, not 0
            if (fftSum > 0) {
                fftMeanIndex = (fftWeightedSum / fftSum) - 1;
                // the index points at the center frequency of each bin so index 0 is actually 16.125Hz
                centerFreq = fftMeanIndex * fftResolution;
            } else {
                centerFreq = state->prevCenterFreq[state->updateAxis];
            }
            centerFreq = fmax(centerFreq, dynNotchMinHz);
            centerFreq = biquadFilterApply(&state->detectedFrequencyFilter[state->updateAxis], centerFreq);
            state->prevCenterFreq[state->updateAxis] = state->centerFreq[state->updateAxis];
            state->centerFreq[state->updateAxis] = centerFreq;

            if(calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
                dynNotchMaxFFT = MAX(dynNotchMaxFFT, state->centerFreq[state->updateAxis]);
            }

            if (state->updateAxis == 0) {
                DEBUG_SET(DEBUG_FFT, 3, lrintf(fftMeanIndex * 100));
                DEBUG_SET(DEBUG_FFT_FREQ, 0, state->centerFreq[state->updateAxis]);
                DEBUG_SET(DEBUG_DYN_LPF, 1, state->centerFreq[state->updateAxis]);
            }
            if (state->updateAxis == 1) {
                DEBUG_SET(DEBUG_FFT_FREQ, 1, state->centerFreq[state->updateAxis]);
            }
            // Debug FFT_Freq carries raw gyro, gyro after first filter set, FFT centre for roll and for pitch
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);
            break;
        }
        case STEP_UPDATE_FILTERS:
        {
            // 7us
            // calculate cutoffFreq and notch Q, update notch filter  =1.8+((A2-150)*0.004)
            if (state->prevCenterFreq[state->updateAxis] != state->centerFreq[state->updateAxis]) {
                if (dualNotch) {
                    biquadFilterUpdate(&notchFilterDyn[state->updateAxis], state->centerFreq[state->updateAxis] * dynNotch1Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                    biquadFilterUpdate(&notchFilterDyn2[state->updateAxis], state->centerFreq[state->updateAxis] * dynNotch2Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                } else {
                    biquadFilterUpdate(&notchFilterDyn[state->updateAxis], state->centerFreq[state->updateAxis], gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                }
            }
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


uint16_t getMaxFFT(void) {
    return dynNotchMaxFFT;
}

void resetMaxFFT(void) {
    dynNotchMaxFFT = 0;
}

#endif // USE_GYRO_DATA_ANALYSE
