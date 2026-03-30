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
    // settings
    int numHarmonics;
    float weights[RPM_FILTER_HARMONICS_MAX];
    float minHz;
    float maxHz;
    float fadeRangeHz;
    float q;

    float dt;

    // state
    rpmNotch_t notch[MAX_SUPPORTED_MOTORS][RPM_FILTER_HARMONICS_MAX];
    int notchUpdatesPerIteration;
    int motorIndex;
    int harmonicIndex;
} rpmFilter_t;

// Singleton
FAST_DATA_ZERO_INIT static rpmFilter_t rpmFilter;

void rpmFilterInit(const rpmFilterConfig_t *config, const timeUs_t looptimeUs)
{
    rpmFilter.motorIndex = 0;
    rpmFilter.harmonicIndex = 0;
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
    rpmFilter.dt = looptimeUs * 1e-6f;

    for (int n = 0; n < RPM_FILTER_HARMONICS_MAX; n++) {
        rpmFilter.weights[n] = constrainf(config->rpm_filter_weights[n] / 100.0f, 0.0f, 1.0f);
    }

    for (int motor = 0; motor < getMotorCount(); motor++) {
        for (int i = 0; i < rpmFilter.numHarmonics; i++) {
            rpmNotchInit(&rpmFilter.notch[motor][i], rpmFilter.minHz * (i + 1), rpmFilter.dt, rpmFilter.q, 0.0f);
        }
    }

    const float loopIterationsPerUpdate = RPM_FILTER_DURATION_S / (looptimeUs * 1e-6f);
    const float numNotchesPerAxis = getMotorCount() * rpmFilter.numHarmonics;
    rpmFilter.notchUpdatesPerIteration = ceilf(numNotchesPerAxis / loopIterationsPerUpdate); // round to ceiling
}

static inline void rpmFilterUpdate(void)
{
    if (!useDshotTelemetry || !rpmFilter.numHarmonics) {
        return;
    }

    const float dtCompensation = schedulerGetCycleTimeMultiplier();
    const float correctedDt = rpmFilter.dt * dtCompensation;

    // update RPM notches
    for (int i = 0; i < rpmFilter.notchUpdatesPerIteration; i++) {

        // Only bother updating notches which have an effect on filtered output
        if (rpmFilter.weights[rpmFilter.harmonicIndex] > 0.0f) {
            const float frequencyHz = constrainf((rpmFilter.harmonicIndex + 1) * getMotorFrequencyHz(rpmFilter.motorIndex), rpmFilter.minHz, rpmFilter.maxHz);
            const float marginHz = frequencyHz - rpmFilter.minHz;
            float weight = 1.0f;

            // fade out notch when approaching minHz (turn it off)
            if (marginHz < rpmFilter.fadeRangeHz) {
                weight *= marginHz / rpmFilter.fadeRangeHz;
            }

            // attenuate notches per harmonics group
            weight *= rpmFilter.weights[rpmFilter.harmonicIndex];

            // update notch
            rpmNotchUpdate(&rpmFilter.notch[rpmFilter.motorIndex][rpmFilter.harmonicIndex], frequencyHz, correctedDt, rpmFilter.q, weight);
        }

        // cycle through all notches on ROLL (takes RPM_FILTER_DURATION_S at max.)
        rpmFilter.harmonicIndex = (rpmFilter.harmonicIndex + 1) % rpmFilter.numHarmonics;
        if (rpmFilter.harmonicIndex == 0) {
            rpmFilter.motorIndex = (rpmFilter.motorIndex + 1) % getMotorCount();
        }
    }
}

static inline void rpmFilterApply(float input[3])
{
    // Iterate over all notches on axis and apply each one to value.
    // Order of application doesn't matter because biquads are linear time-invariant filters.
    for (int i = 0; i < rpmFilter.numHarmonics; i++) {

        if (rpmFilter.weights[i] <= 0.0f) {
            continue;  // skip harmonics which have no effect on filtered output
        }

        for (int motor = 0; motor < getMotorCount(); motor++) {
            rpmNotchApply(&rpmFilter.notch[motor][i], input);
        }
    }
}

FAST_CODE void rpmFilterRun(float input[3])
{
    rpmFilterUpdate();
    rpmFilterApply(input);
}

#endif // USE_RPM_FILTER
