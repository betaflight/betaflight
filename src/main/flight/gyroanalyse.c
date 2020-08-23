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

#include "fc/core.h"

#include "gyroanalyse.h"

// FFT_WINDOW_SIZE defaults to 32 (gyroanalyse.h)
// We get 16 frequency bins from 32 consecutive data values
// Bin 0 is DC and can't be used.
// Only bins 1 to 15 are usable.

// A gyro sample is collected every gyro loop
// maxSampleCount recent gyro values are accumulated and averaged
// to ensure that 32 samples are collected at the right rate for the required FFT bandwidth

// For an 8k gyro loop, at default 600hz max, 6 sequential gyro data points are averaged, FFT runs 1333Hz.
// Upper limit of FFT is half that frequency, eg 666Hz by default.
// At 8k, if user sets a max of 300Hz, int(8000/600) = 13, fftSamplingRateHz = 615Hz, range 307Hz
// Note that lower max requires more samples to be averaged, increasing precision but taking longer to get enough samples.
// For Bosch at 3200Hz gyro, max of 600, int(3200/1200) = 2, fftSamplingRateHz = 1600, range to 800hz
// For Bosch on XClass, better to set a max of 300, int(3200/600) = 5, fftSamplingRateHz = 640, range to 320Hz
//
// When sampleCount reaches maxSampleCount, the averaged gyro value is put into the circular buffer of 32 samples
// At 8k, with 600Hz max, maxSampleCount = 6, this happens every 6 * 0.125us, or every 0.75ms
// Hence to completely replace all 32 samples of the FFT input buffer with clean new data takes 24ms

// The FFT code is split into steps.  It takes 4 gyro loops to calculate the FFT for one axis
//   (gyroDataAnalyseUpdate has 8 steps, but only four breaks)
// Since there are three axes, it takes 12 gyro loops to completely update all axes.
// At 8k, any one axis gets updated at 8000 / 12 or 666hz or every 1.5ms
// In this time, 2 points in the FFT buffer will have changed.
// At 4k, it takes twice as long to update an axis, i.e. each axis updates only every 3ms
// Four points in the buffer will have changed in that time, and each point will be the average of three samples.
// Hence output jitter at 4k is about four times worse than at 8k.  At 2k output jitter is quite bad.

// The Hanning window step loads gyro data (32 data points) for one axis from the circular buffer into fftData[i]
//   and applies the hanning window to the edge values
// Calculation steps 1 and 2 then calculate the fft output (32 data points) and put that back into the same fftData[i] array.
// We then use fftData[i] array for frequency centre calculations for that axis

// Each FFT output bin has width fftSamplingRateHz/32, ie 41.65Hz per bin at 1333Hz
// Usable bandwidth is half this, ie 666Hz if fftSamplingRateHz is 1333Hz, i.e. bin 1 is 41.65hz, bin 2 83.3hz etc

#define DYN_NOTCH_SMOOTH_HZ       4
#define FFT_BIN_COUNT             (FFT_WINDOW_SIZE / 2) // 16
#define DYN_NOTCH_CALC_TICKS      (XYZ_AXIS_COUNT * 4) // 4 steps per axis
#define DYN_NOTCH_OSD_MIN_THROTTLE 20

static uint16_t FAST_DATA_ZERO_INIT   fftSamplingRateHz;
static float FAST_DATA_ZERO_INIT      fftResolution;
static uint8_t FAST_DATA_ZERO_INIT    fftStartBin;
static float FAST_DATA_ZERO_INIT      dynNotchQ;
static float FAST_DATA_ZERO_INIT      dynNotch1Ctr;
static float FAST_DATA_ZERO_INIT      dynNotch2Ctr;
static uint16_t FAST_DATA_ZERO_INIT   dynNotchMinHz;
static uint16_t FAST_DATA_ZERO_INIT   dynNotchMaxHz;
static bool FAST_DATA                 dualNotch = true;
static uint16_t FAST_DATA_ZERO_INIT   dynNotchMaxFFT;
static float FAST_DATA_ZERO_INIT      smoothFactor;
static uint8_t FAST_DATA_ZERO_INIT    samples;
// Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
static FAST_DATA_ZERO_INIT float hanningWindow[FFT_WINDOW_SIZE];

void gyroDataAnalyseInit(uint32_t targetLooptimeUs)
{
#ifdef USE_MULTI_GYRO
    static bool gyroAnalyseInitialized;
    if (gyroAnalyseInitialized) {
        return;
    }
    gyroAnalyseInitialized = true;
#endif

    dynNotch1Ctr = 1 - gyroConfig()->dyn_notch_width_percent / 100.0f;
    dynNotch2Ctr = 1 + gyroConfig()->dyn_notch_width_percent / 100.0f;
    dynNotchQ = gyroConfig()->dyn_notch_q / 100.0f;
    dynNotchMinHz = gyroConfig()->dyn_notch_min_hz;
    dynNotchMaxHz = MAX(2 * dynNotchMinHz, gyroConfig()->dyn_notch_max_hz);

    if (gyroConfig()->dyn_notch_width_percent == 0) {
        dualNotch = false;
    }

    const int gyroLoopRateHz = lrintf((1.0f / targetLooptimeUs) * 1e6f);
    samples = MAX(1, gyroLoopRateHz / (2 * dynNotchMaxHz)); //600hz, 8k looptime, 13.333

    fftSamplingRateHz = gyroLoopRateHz / samples;
    // eg 8k, user max 600hz, int(8000/1200) = 6 (6.666), fftSamplingRateHz = 1333hz, range 666Hz
    // eg 4k, user max 600hz, int(4000/1200) = 3 (3.333), fftSamplingRateHz = 1333hz, range 666Hz
    // eg 2k, user max 600hz, int(2000/1200) = 1 (1.666) fftSamplingRateHz = 2000hz, range 1000Hz
    // eg 2k, user max 400hz, int(2000/800) = 2 (2.5) fftSamplingRateHz = 1000hz, range 500Hz
    // eg 1k, user max 600hz, int(1000/1200) = 1 (max(1,0.8333)) fftSamplingRateHz = 1000hz, range 500Hz
    // the upper limit of DN is always going to be Nyquist

    fftResolution = (float)fftSamplingRateHz / FFT_WINDOW_SIZE; // 41.65hz per bin for medium
    fftStartBin = MAX(2, dynNotchMinHz / lrintf(fftResolution)); // can't use bin 0 because it is DC.
    smoothFactor = 2 * M_PIf * DYN_NOTCH_SMOOTH_HZ / (gyroLoopRateHz / 12); // minimum PT1 k value

    for (int i = 0; i < FFT_WINDOW_SIZE; i++) {
        hanningWindow[i] = (0.5f - 0.5f * cos_approx(2 * M_PIf * i / (FFT_WINDOW_SIZE - 1)));
    }
}

void gyroDataAnalyseStateInit(gyroAnalyseState_t *state, uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    gyroDataAnalyseInit(targetLooptimeUs);
    state->maxSampleCount = samples;
    state->maxSampleCountRcp = 1.0f / state->maxSampleCount;
    arm_rfft_fast_init_f32(&state->fftInstance, FFT_WINDOW_SIZE);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // any init value
        state->centerFreq[axis] = dynNotchMaxHz;
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
    // if gyro sampling is > 1kHz, accumulate and average multiple gyro samples
    state->sampleCount++;

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
        // recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
        // at 4kHz gyro loop rate this means 8kHz / 4 / 3 = 666Hz => update every 1.5ms
        // at 4kHz gyro loop rate this means 4kHz / 4 / 3 = 333Hz => update every 3ms
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
 * Analyse gyro data
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
            // identify max bin and max/min heights
            float dataMax = 0.0f;
            float dataMin = 1.0f;
            uint8_t binMax = 0;
            float dataMinHi = 1.0f;
            for (int i = fftStartBin; i < FFT_BIN_COUNT; i++) {
                if (state->fftData[i] > state->fftData[i - 1]) { // bin height increased
                    if (state->fftData[i] > dataMax) {
                        dataMax = state->fftData[i];
                        binMax = i;  // tallest bin so far
                    }
                }
            }
            if (binMax == 0) { // no bin increase, hold prev max bin, dataMin = 1 dataMax = 0, ie move slow
                binMax = lrintf(state->centerFreq[state->updateAxis] / fftResolution);
            } else { // there was a max, find min
                for (int i = binMax - 1; i > 1; i--) { // look for min below max
                    dataMin = state->fftData[i];
                    if (state->fftData[i - 1] > state->fftData[i]) { // up step below this one
                        break;
                    }
                }
                for (int i = binMax + 1; i < (FFT_BIN_COUNT - 1); i++) { // // look for min above max
                    dataMinHi = state->fftData[i];
                    if (state->fftData[i] < state->fftData[i + 1]) { // up step above this one
                        break;
                    }
                }
            }
            dataMin = fminf(dataMin, dataMinHi);

            // accumulate fftSum and fftWeightedSum from peak bin, and shoulder bins either side of peak
            float squaredData = state->fftData[binMax] * state->fftData[binMax];
            float fftSum = squaredData;
            float fftWeightedSum = squaredData * binMax;

            // accumulate upper shoulder unless it would be FFT_BIN_COUNT
            uint8_t shoulderBin = binMax + 1;
            if (shoulderBin < FFT_BIN_COUNT) {
                squaredData = state->fftData[shoulderBin] * state->fftData[shoulderBin];
                fftSum += squaredData;
                fftWeightedSum += squaredData * shoulderBin;
            }

            // accumulate lower shoulder unless lower shoulder would be bin 0 (DC)
            if (binMax > 1) {
                shoulderBin = binMax - 1;
                squaredData = state->fftData[shoulderBin] * state->fftData[shoulderBin];
                fftSum += squaredData;
                fftWeightedSum += squaredData * shoulderBin;
            }

            // get centerFreq in Hz from weighted bins
            float centerFreq = dynNotchMaxHz;
            float fftMeanIndex = 0;
            if (fftSum > 0) {
                fftMeanIndex = (fftWeightedSum / fftSum);
                centerFreq = fftMeanIndex * fftResolution;
                // In theory, the index points to the centre frequency of the bin.
                // at 1333hz, bin widths are 41.65Hz, so bin 2 has the range 83,3Hz to 124,95Hz
                // Rav feels that maybe centerFreq = (fftMeanIndex + 0.5) * fftResolution; is better
                // empirical checking shows that not adding 0.5 works better
            } else {
                centerFreq = state->centerFreq[state->updateAxis];
            }
            centerFreq = constrainf(centerFreq, dynNotchMinHz, dynNotchMaxHz);

            // PT1 style dynamic smoothing moves rapidly towards big peaks and slowly away, up to 8x faster
            float dynamicFactor = constrainf(dataMax / dataMin, 1.0f, 8.0f);
            state->centerFreq[state->updateAxis] = state->centerFreq[state->updateAxis] + smoothFactor * dynamicFactor * (centerFreq - state->centerFreq[state->updateAxis]);

            if(calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
                dynNotchMaxFFT = MAX(dynNotchMaxFFT, state->centerFreq[state->updateAxis]);
            }

            if (state->updateAxis == 0) {
                DEBUG_SET(DEBUG_FFT, 3, lrintf(fftMeanIndex * 100));
                DEBUG_SET(DEBUG_FFT_FREQ, 0, state->centerFreq[state->updateAxis]);
                DEBUG_SET(DEBUG_FFT_FREQ, 1, lrintf(dynamicFactor * 100));
                DEBUG_SET(DEBUG_DYN_LPF, 1, state->centerFreq[state->updateAxis]);
            }
//            if (state->updateAxis == 1) {
//            DEBUG_SET(DEBUG_FFT_FREQ, 1, state->centerFreq[state->updateAxis]);
//            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_UPDATE_FILTERS:
        {
            // 7us
            // calculate cutoffFreq and notch Q, update notch filter
            if (dualNotch) {
                biquadFilterUpdate(&notchFilterDyn[state->updateAxis], state->centerFreq[state->updateAxis] * dynNotch1Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
                biquadFilterUpdate(&notchFilterDyn2[state->updateAxis], state->centerFreq[state->updateAxis] * dynNotch2Ctr, gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
            } else {
                biquadFilterUpdate(&notchFilterDyn[state->updateAxis], state->centerFreq[state->updateAxis], gyro.targetLooptime, dynNotchQ, FILTER_NOTCH);
            }
            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            state->updateAxis = (state->updateAxis + 1) % XYZ_AXIS_COUNT;
            state->updateStep++;
            FALLTHROUGH;
        }
        case STEP_HANNING:
        {
            // 5us
            // apply hanning window to gyro samples and store result in fftData[i] to be used in step 1 and 2 and 3
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
