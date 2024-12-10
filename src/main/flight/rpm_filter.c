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

#include <math.h>

#include "platform.h"

#ifdef USE_RPM_FILTER

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "drivers/dshot.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/motor.h"

#include "scheduler/scheduler.h"

#include "sensors/gyro.h"

#include "rpm_filter.h"

#define RPM_FILTER_DURATION_S    0.001f  // Maximum duration allowed to update all RPM notches once

typedef struct rpmFilter_s {

    int numHarmonics;
    float weights[RPM_FILTER_HARMONICS_MAX];
    float minHz;
    float maxHz;
    float fadeRangeHz;
    float q;

    timeUs_t looptimeUs;
    biquadFilter_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_HARMONICS_MAX];

} rpmFilter_t;

// Singleton
FAST_DATA_ZERO_INIT static rpmFilter_t rpmFilter;

// batch processing of RPM notches
FAST_DATA_ZERO_INIT static int notchUpdatesPerIteration;
FAST_DATA_ZERO_INIT static int motorIndex;
FAST_DATA_ZERO_INIT static int harmonicIndex;

void rpmFilterInit(const rpmFilterConfig_t *config, const timeUs_t looptimeUs)
{
    motorIndex = 0;
    harmonicIndex = 0;
    rpmFilter.numHarmonics = 0; // disable RPM Filtering

    // if bidirectional DShot is not available
    if (!useDshotTelemetry) {
        return;
    }

    // if RPM Filtering is configured to be off
    if (!config->rpm_filter_harmonics) {
        return;
    }

    // if we get to this point, enable and init RPM filtering
    rpmFilter.numHarmonics = config->rpm_filter_harmonics;
    rpmFilter.minHz = config->rpm_filter_min_hz;
    rpmFilter.maxHz = 0.48f * 1e6f / looptimeUs; // don't go quite to nyquist to avoid oscillations
    rpmFilter.fadeRangeHz = config->rpm_filter_fade_range_hz;
    rpmFilter.q = config->rpm_filter_q / 100.0f;
    rpmFilter.looptimeUs = looptimeUs;

    for (int n = 0; n < RPM_FILTER_HARMONICS_MAX; n++) {
        rpmFilter.weights[n] = constrainf(config->rpm_filter_weights[n] / 100.0f, 0.0f, 1.0f);
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int i = 0; i < rpmFilter.numHarmonics; i++) {
                biquadFilterInit(&rpmFilter.notch[axis][motor][i], rpmFilter.minHz * i, rpmFilter.looptimeUs, rpmFilter.q, FILTER_NOTCH, 0.0f);
            }
        }
    }

    const float loopIterationsPerUpdate = RPM_FILTER_DURATION_S / (looptimeUs * 1e-6f);
    const float numNotchesPerAxis = getMotorCount() * rpmFilter.numHarmonics;
    notchUpdatesPerIteration = ceilf(numNotchesPerAxis / loopIterationsPerUpdate); // round to ceiling
}

FAST_CODE_NOINLINE void rpmFilterUpdate(void)
{
    if (!useDshotTelemetry) {
        return;
    }

    for (int motor = 0; motor < getMotorCount() && motor < DEBUG16_VALUE_COUNT; motor++) {
        DEBUG_SET(DEBUG_RPM_FILTER, motor, lrintf(getMotorFrequencyHz(motor)));
    }

    if (!isRpmFilterEnabled()) {
        return;
    }

    // update RPM notches
    for (int i = 0; i < notchUpdatesPerIteration; i++) {

        // Only bother updating notches which have an effect on filtered output
        if (rpmFilter.weights[harmonicIndex] > 0.0f) {

            // select current notch on ROLL
            biquadFilter_t *template = &rpmFilter.notch[0][motorIndex][harmonicIndex];

            const float frequencyHz = constrainf((harmonicIndex + 1) * getMotorFrequencyHz(motorIndex), rpmFilter.minHz, rpmFilter.maxHz);
            const float marginHz = frequencyHz - rpmFilter.minHz;
            float weight = 1.0f;

            // fade out notch when approaching minHz (turn it off)
            if (marginHz < rpmFilter.fadeRangeHz) {
                weight *= marginHz / rpmFilter.fadeRangeHz;
            }

            // attenuate notches per harmonics group
            weight *= rpmFilter.weights[harmonicIndex];

            // update notch
            biquadFilterUpdate(template, frequencyHz, rpmFilter.looptimeUs, rpmFilter.q, FILTER_NOTCH, weight);

            // copy notch properties to corresponding notches on PITCH and YAW
            for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilter_t *dest = &rpmFilter.notch[axis][motorIndex][harmonicIndex];
                dest->b0 = template->b0;
                dest->b1 = template->b1;
                dest->b2 = template->b2;
                dest->a1 = template->a1;
                dest->a2 = template->a2;
                dest->weight = template->weight;
            }
        }

        // cycle through all notches on ROLL (takes RPM_FILTER_DURATION_S at max.)
        harmonicIndex = (harmonicIndex + 1) % rpmFilter.numHarmonics;
        if (harmonicIndex == 0) {
            motorIndex = (motorIndex + 1) % getMotorCount();
        }
    }
}

FAST_CODE float rpmFilterApply(const int axis, float value)
{
    // Iterate over all notches on axis and apply each one to value.
    // Order of application doesn't matter because biquads are linear time-invariant filters.
    for (int i = 0; i < rpmFilter.numHarmonics; i++) {

        if (rpmFilter.weights[i] <= 0.0f) {
            continue;  // skip harmonics which have no effect on filtered output
        }

        for (int motor = 0; motor < getMotorCount(); motor++) {
            value = biquadFilterApplyDF1Weighted(&rpmFilter.notch[axis][motor][i], value);
        }
    }

    return value;
}

bool isRpmFilterEnabled(void)
{
    return rpmFilter.numHarmonics > 0;
}

#endif // USE_RPM_FILTER
