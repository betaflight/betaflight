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

#include <stdint.h>

#include "platform.h"

#ifdef USE_GYRO_DATA_ANALYSE

#include "arm_math.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/system.h"

#include "sensors/gyro.h"
#include "sensors/gyroanalyse.h"

#define FFT_WINDOW_SIZE 512

static bool gyroDataReady = false;

static arm_rfft_fast_instance_f32 fftInstance;
static float fftOut[FFT_WINDOW_SIZE];

static float gyroData[3][FFT_WINDOW_SIZE];


#define SAMPLING_FREQ 8000
#define FFT_BIN(freq) ((FFT_WINDOW_SIZE / 2 - 1) * freq) / (SAMPLING_FREQ / 2)

#define FFT_MAX_FREQ 800
#define FFT_SAMPLE_COUNT (FFT_BIN(FFT_MAX_FREQ) + 1)


// Hanning window
// values taken from https://github.com/BrainFPV/betaflight/blob/brainre1/src/main/target/BRAINRE1/spectrograph.c
static const float FFT_WINDOW[FFT_WINDOW_SIZE] = {
    0.00000, 0.00004, 0.00015, 0.00034, 0.00060, 0.00094, 0.00136, 0.00185, 0.00242, 0.00306, 0.00377, 0.00457, 0.00543, 0.00637, 0.00739, 0.00848,
    0.00964, 0.01088, 0.01220, 0.01358, 0.01504, 0.01658, 0.01818, 0.01986, 0.02161, 0.02344, 0.02533, 0.02730, 0.02934, 0.03145, 0.03363, 0.03589,
    0.03821, 0.04060, 0.04306, 0.04559, 0.04819, 0.05086, 0.05359, 0.05640, 0.05927, 0.06220, 0.06521, 0.06827, 0.07141, 0.07461, 0.07787, 0.08120,
    0.08459, 0.08804, 0.09155, 0.09513, 0.09877, 0.10247, 0.10623, 0.11004, 0.11392, 0.11786, 0.12185, 0.12590, 0.13001, 0.13417, 0.13839, 0.14266,
    0.14699, 0.15137, 0.15580, 0.16029, 0.16483, 0.16941, 0.17405, 0.17874, 0.18347, 0.18826, 0.19309, 0.19796, 0.20288, 0.20785, 0.21286, 0.21792,
    0.22301, 0.22815, 0.23333, 0.23855, 0.24381, 0.24911, 0.25445, 0.25982, 0.26523, 0.27068, 0.27616, 0.28167, 0.28722, 0.29280, 0.29841, 0.30405,
    0.30972, 0.31542, 0.32115, 0.32691, 0.33269, 0.33849, 0.34432, 0.35018, 0.35605, 0.36195, 0.36787, 0.37381, 0.37977, 0.38574, 0.39174, 0.39775,
    0.40377, 0.40981, 0.41587, 0.42193, 0.42801, 0.43410, 0.44020, 0.44631, 0.45243, 0.45855, 0.46468, 0.47081, 0.47695, 0.48310, 0.48924, 0.49539,
    0.50154, 0.50768, 0.51383, 0.51998, 0.52612, 0.53225, 0.53839, 0.54451, 0.55063, 0.55675, 0.56285, 0.56894, 0.57503, 0.58110, 0.58716, 0.59321,
    0.59924, 0.60526, 0.61126, 0.61725, 0.62321, 0.62916, 0.63509, 0.64100, 0.64689, 0.65275, 0.65860, 0.66441, 0.67021, 0.67598, 0.68172, 0.68743,
    0.69312, 0.69877, 0.70440, 0.70999, 0.71556, 0.72109, 0.72658, 0.73205, 0.73748, 0.74287, 0.74822, 0.75354, 0.75882, 0.76406, 0.76926, 0.77442,
    0.77954, 0.78462, 0.78965, 0.79464, 0.79958, 0.80448, 0.80934, 0.81414, 0.81890, 0.82361, 0.82827, 0.83289, 0.83745, 0.84196, 0.84642, 0.85083,
    0.85518, 0.85948, 0.86373, 0.86792, 0.87205, 0.87613, 0.88015, 0.88412, 0.88802, 0.89187, 0.89566, 0.89939, 0.90306, 0.90667, 0.91021, 0.91370,
    0.91712, 0.92048, 0.92377, 0.92700, 0.93017, 0.93327, 0.93630, 0.93927, 0.94218, 0.94501, 0.94778, 0.95048, 0.95312, 0.95568, 0.95818, 0.96061,
    0.96296, 0.96525, 0.96747, 0.96961, 0.97169, 0.97369, 0.97562, 0.97748, 0.97927, 0.98099, 0.98263, 0.98420, 0.98570, 0.98712, 0.98847, 0.98975,
    0.99095, 0.99207, 0.99313, 0.99411, 0.99501, 0.99584, 0.99659, 0.99727, 0.99788, 0.99840, 0.99886, 0.99923, 0.99954, 0.99976, 0.99991, 0.99999,
    0.99999, 0.99991, 0.99976, 0.99954, 0.99923, 0.99886, 0.99840, 0.99788, 0.99727, 0.99659, 0.99584, 0.99501, 0.99411, 0.99313, 0.99207, 0.99095,
    0.98975, 0.98847, 0.98712, 0.98570, 0.98420, 0.98263, 0.98099, 0.97927, 0.97748, 0.97562, 0.97369, 0.97169, 0.96961, 0.96747, 0.96525, 0.96296,
    0.96061, 0.95818, 0.95568, 0.95312, 0.95048, 0.94778, 0.94501, 0.94218, 0.93927, 0.93630, 0.93327, 0.93017, 0.92700, 0.92377, 0.92048, 0.91712,
    0.91370, 0.91021, 0.90667, 0.90306, 0.89939, 0.89566, 0.89187, 0.88802, 0.88412, 0.88015, 0.87613, 0.87205, 0.86792, 0.86373, 0.85948, 0.85518,
    0.85083, 0.84642, 0.84196, 0.83745, 0.83289, 0.82827, 0.82361, 0.81890, 0.81414, 0.80934, 0.80448, 0.79958, 0.79464, 0.78965, 0.78462, 0.77954,
    0.77442, 0.76926, 0.76406, 0.75882, 0.75354, 0.74822, 0.74287, 0.73748, 0.73205, 0.72658, 0.72109, 0.71556, 0.70999, 0.70440, 0.69877, 0.69312,
    0.68743, 0.68172, 0.67598, 0.67021, 0.66441, 0.65860, 0.65275, 0.64689, 0.64100, 0.63509, 0.62916, 0.62321, 0.61725, 0.61126, 0.60526, 0.59924,
    0.59321, 0.58716, 0.58110, 0.57503, 0.56894, 0.56285, 0.55675, 0.55063, 0.54451, 0.53839, 0.53225, 0.52612, 0.51998, 0.51383, 0.50768, 0.50154,
    0.49539, 0.48924, 0.48310, 0.47695, 0.47081, 0.46468, 0.45855, 0.45243, 0.44631, 0.44020, 0.43410, 0.42801, 0.42193, 0.41587, 0.40981, 0.40377,
    0.39775, 0.39174, 0.38574, 0.37977, 0.37381, 0.36787, 0.36195, 0.35605, 0.35018, 0.34432, 0.33849, 0.33269, 0.32691, 0.32115, 0.31542, 0.30972,
    0.30405, 0.29841, 0.29280, 0.28722, 0.28167, 0.27616, 0.27068, 0.26523, 0.25982, 0.25445, 0.24911, 0.24381, 0.23855, 0.23333, 0.22815, 0.22301,
    0.21792, 0.21286, 0.20785, 0.20288, 0.19796, 0.19309, 0.18826, 0.18347, 0.17874, 0.17405, 0.16941, 0.16483, 0.16029, 0.15580, 0.15137, 0.14699,
    0.14266, 0.13839, 0.13417, 0.13001, 0.12590, 0.12185, 0.11786, 0.11392, 0.11004, 0.10623, 0.10247, 0.09877, 0.09513, 0.09155, 0.08804, 0.08459,
    0.08120, 0.07787, 0.07461, 0.07141, 0.06827, 0.06521, 0.06220, 0.05927, 0.05640, 0.05359, 0.05086, 0.04819, 0.04559, 0.04306, 0.04060, 0.03821,
    0.03589, 0.03363, 0.03145, 0.02934, 0.02730, 0.02533, 0.02344, 0.02161, 0.01986, 0.01818, 0.01658, 0.01504, 0.01358, 0.01220, 0.01088, 0.00964,
    0.00848, 0.00739, 0.00637, 0.00543, 0.00457, 0.00377, 0.00306, 0.00242, 0.00185, 0.00136, 0.00094, 0.00060, 0.00034, 0.00015, 0.00004, 0.00000
};


void gyroDataAnalyseInit(void)
{
    arm_rfft_fast_init_f32(&fftInstance, FFT_WINDOW_SIZE);
}

void gyroDataAnalyse(const gyroDev_t *gyroDev, const gyro_t *gyro)
{
    UNUSED(gyro);

    static uint16_t index = 0;

    if (gyroDataReady == false) {
        gyroData[0][index] = gyroDev->gyroADC[0] * gyroDev->scale;
        gyroData[1][index] = gyroDev->gyroADC[1] * gyroDev->scale;
        gyroData[2][index] = gyroDev->gyroADC[2] * gyroDev->scale;
        ++index;
        if (index == FFT_WINDOW_SIZE) {
            gyroDataReady = true;
            index = 0;
        }
    }
}

void stage_rfft_f32(arm_rfft_fast_instance_f32 * S, float32_t * p, float32_t * pOut);
typedef enum {
    STEP_MULTIPLY,
    //STEP_RFFT_FAST,
    STEP_CFFT,
    STEP_STAGE_RFFT,
    STEP_CMPLX_MAG,
    STEP_MAX,
    STEP_COUNT
} UpdateStep_e;

void gyroDataAnalyseUpdate(timeUs_t currentTimeUs)
{
    static int axis = 0;
    static int step = 0;
    UNUSED(currentTimeUs);

    if (gyroDataReady == false) {
        // no new data, so return
        return;
    }
    uint32_t startTime;
    if (debugMode == DEBUG_FFT) {startTime = micros();}
    arm_cfft_instance_f32 * Sint = &(fftInstance.Sint);
    Sint->fftLen = fftInstance.fftLenRFFT / 2;

#define MAX_START_FREQ 50
    float maxVal;
    uint32_t maxIdx;
    switch (step) {
    case STEP_MULTIPLY:
        // 36us
        for (int ii = 0; ii < FFT_WINDOW_SIZE; ++ii) {
            gyroData[axis][ii] *= FFT_WINDOW[ii];
        }
        break;
    /*case STEP_RFFT_FAST:
        // 217us
        arm_rfft_fast_f32(&fftInstance, gyroData[axis], fftOut, 0);
        // arm_rfft_fast_f32 is split into component parts see lib/main/DSP_lib/TransformFunctions/arm_rfft_fast_f32.c
        break;*/
    case STEP_CFFT:
        // 153us
        arm_cfft_f32(Sint, gyroData[axis], 0, 1);
        break;
    case STEP_STAGE_RFFT:
        // 58us
        stage_rfft_f32(&fftInstance, gyroData[axis], fftOut);
        break;
    case STEP_CMPLX_MAG:
        // 37us
        arm_cmplx_mag_f32(fftOut, fftOut, FFT_SAMPLE_COUNT);
        break;
    case STEP_MAX:
        // us
        arm_max_f32(&fftOut[FFT_BIN(MAX_START_FREQ)], FFT_SAMPLE_COUNT - FFT_BIN(MAX_START_FREQ) - 1, &maxVal, &maxIdx);
        //DEBUG_SET(DEBUG_FFT, 1, maxVal);
        //DEBUG_SET(DEBUG_FFT, 2, maxIdx);
        break;
    }
    if (step<4) DEBUG_SET(DEBUG_FFT, step, micros() - startTime);

    ++step;
    if (step >= STEP_COUNT) {
        step = 0;
        ++axis;
        if (axis >= 3) {
            axis = 0;
            // finished processing data, so set data ready false
            gyroDataReady = false;
        }
    }
}

#endif // USE_GYRO_DATA_ANALYSE
