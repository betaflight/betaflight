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
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/debug.h"

#include "common/filter.h"
#include "common/utils.h"

#include "config/feature.h"

#include "sensors/acceleration_init.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

#include "acceleration.h"

FAST_DATA_ZERO_INIT acc_t acc;                       // acc access functions
#ifdef USE_CRSF_ACCGYRO_TELEMETRY
// Two-counter seqlock pattern for lock-free synchronization:
// - accelSeq: only incremented by accUpdate (accumulator writes)
// - resetSeq: only incremented by telemetry task (accumulator resets)
// This ensures each seqlock has exactly one writer, avoiding dual-writer race conditions.
// When resetSeq is odd (reset in progress), accUpdate skips accumulation to avoid corruption.
// At most one sample per telemetry cycle may be skipped - negligible at 1kHz accel rate.
static volatile uint32_t accelSeq;   // Sequence counter for accel writes (odd = write in progress)
static volatile uint32_t resetSeq;   // Sequence counter for resets (odd = reset in progress)
static volatile FAST_DATA_ZERO_INIT uint32_t downSampleCount;
static volatile FAST_DATA_ZERO_INIT float downSampleSum[XYZ_AXIS_COUNT];
static FAST_DATA_ZERO_INIT uint32_t snapshotCount;
static FAST_DATA_ZERO_INIT float snapshotSum[XYZ_AXIS_COUNT];
#endif

static inline void alignAccelerometer(void)
{
    switch (acc.dev.accAlign) {
        case ALIGN_CUSTOM:
            alignSensorViaMatrix(&acc.accADC, &acc.dev.rotationMatrix);
            break;
        default:
            alignSensorViaRotation(&acc.accADC, acc.dev.accAlign);
            break;
    }
}

static inline void calibrateAccelerometer(void)
{
    if (!accIsCalibrationComplete()) {
        // acc.accADC is held at 0 until calibration is completed
        performAccelerometerCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }
}

static inline void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC.x -= accelerationTrims->raw[X];
    acc.accADC.y -= accelerationTrims->raw[Y];
    acc.accADC.z -= accelerationTrims->raw[Z];
}

static inline void postProcessAccelerometer(void)
{
    static vector3_t accAdcPrev;

    for (unsigned axis = 0; axis < XYZ_AXIS_COUNT; axis++) {

        // Apply anti-alias filter for attitude task (if enabled)
        if (axis == gyro.gyroDebugAxis) {
            DEBUG_SET(DEBUG_ACCELEROMETER, 0, lrintf(acc.accADC.v[axis]));
        }

        if (accelerationRuntime.accLpfCutHz) {
            acc.accADC.v[axis] = pt2FilterApply(&accelerationRuntime.accFilter[axis], acc.accADC.v[axis]);
        }

        // Calculate derivative of acc (jerk)
        acc.jerk.v[axis] = (acc.accADC.v[axis] - accAdcPrev.v[axis]) * acc.sampleRateHz;
        accAdcPrev.v[axis] = acc.accADC.v[axis];

        if (axis == gyro.gyroDebugAxis) {
            DEBUG_SET(DEBUG_ACCELEROMETER, 1, lrintf(acc.accADC.v[axis]));
            DEBUG_SET(DEBUG_ACCELEROMETER, 3, lrintf(acc.jerk.v[axis] * 1e-2f));
        }
    }

    acc.accMagnitude = vector3Norm(&acc.accADC) * acc.dev.acc_1G_rec;
    acc.jerkMagnitude = vector3Norm(&acc.jerk) * acc.dev.acc_1G_rec;

    DEBUG_SET(DEBUG_ACCELEROMETER, 2, lrintf(acc.accMagnitude * 1e3f));
    DEBUG_SET(DEBUG_ACCELEROMETER, 4, lrintf(acc.jerkMagnitude * 1e3f));
}

void accUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc.accADC.v[axis] = acc.dev.ADCRaw[axis];
    }

    alignAccelerometer();
    calibrateAccelerometer();
    applyAccelerationTrims(accelerationRuntime.accelerationTrims);
    postProcessAccelerometer();

#ifdef USE_CRSF_ACCGYRO_TELEMETRY
    // Skip accumulation if reset is in progress (resetSeq odd) to avoid corruption.
    // At most one sample per telemetry cycle is skipped - negligible at 1kHz accel rate.
    if (!(resetSeq & 1)) {
        // Seqlock write: increment before modifying (makes accelSeq odd)
        accelSeq++;
        __asm volatile ("" ::: "memory");

        downSampleSum[X] += acc.accADC.v[X];
        downSampleSum[Y] += acc.accADC.v[Y];
        downSampleSum[Z] += acc.accADC.v[Z];
        downSampleCount++;

        __asm volatile ("" ::: "memory");
        accelSeq++;  // Increment after modifying (makes accelSeq even)
    }
#endif

    acc.isAccelUpdatedAtLeastOnce = true;
}

#ifdef USE_CRSF_ACCGYRO_TELEMETRY
bool accelHasDownsampledData(void)
{
    return snapshotCount > 0;
}

float accelGetDownsampled(int axis)
{
    if (snapshotCount == 0) {
        return 0;  // No valid data yet
    }
    return snapshotSum[axis] / snapshotCount;
}

bool accelStartDownsampledCycle(void)
{
    // Phase 1: Read using accelSeq (protects against accUpdate writes)
    uint32_t seq1;
    float sum[XYZ_AXIS_COUNT];
    uint32_t count;

    do {
        seq1 = accelSeq;
        if (!(seq1 & 1)) {
            // Even accelSeq means no write in progress, safe to copy
            sum[X] = downSampleSum[X];
            sum[Y] = downSampleSum[Y];
            sum[Z] = downSampleSum[Z];
            count = downSampleCount;

            __asm volatile ("" ::: "memory");
            if (seq1 == accelSeq) {
                break;
            }
        }
    } while (true);

    // Only update snapshot if we got valid data
    if (count > 0) {
        snapshotSum[X] = sum[X];
        snapshotSum[Y] = sum[Y];
        snapshotSum[Z] = sum[Z];
        snapshotCount = count;
    }

    // Phase 2: Reset using resetSeq (signals accUpdate to skip accumulation)
    resetSeq++;  // Makes resetSeq odd - accUpdate will skip
    __asm volatile ("" ::: "memory");

    downSampleSum[X] = 0;
    downSampleSum[Y] = 0;
    downSampleSum[Z] = 0;
    downSampleCount = 0;

    __asm volatile ("" ::: "memory");
    resetSeq++;  // Makes resetSeq even - accUpdate resumes

    return count > 0;
}
#endif

#endif // USE_ACC
