/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#define SITL_RATE_WARMUP_SECONDS 0.1
#define SITL_RATE_FILTER_WEIGHT 0.01
#define SITL_PAUSE_SECONDS 0.1

typedef struct sitlRateEstimator_s {
    double simulatedInterval;
    double realInterval;
    double warmupSimulatedTime;
    double warmupRealTime;
    unsigned warmupSamples;
    bool initialized;
} sitlRateEstimator_t;

static inline void sitlRateEstimatorReset(sitlRateEstimator_t *estimator)
{
    estimator->simulatedInterval = 0.0;
    estimator->realInterval = 0.0;
    estimator->warmupSimulatedTime = 0.0;
    estimator->warmupRealTime = 0.0;
    estimator->warmupSamples = 0;
    estimator->initialized = false;
}

static inline bool sitlRateEstimatorUpdate(sitlRateEstimator_t *estimator, double deltaSim, double deltaReal, double *rate)
{
    // Exclude slow simulator intervals without discarding warm-up progress.
    if (deltaSim <= 0.0 || deltaSim >= 0.02 || deltaReal < 0.0) {
        return false;
    }

    if (!estimator->initialized) {
        // Start from an interval average so an early packet burst cannot set an
        // extreme rate before the filter has a representative real-time span.
        estimator->warmupSimulatedTime += deltaSim;
        estimator->warmupRealTime += deltaReal;
        estimator->warmupSamples++;

        if (estimator->warmupRealTime + 1e-12 < SITL_RATE_WARMUP_SECONDS) {
            return false;
        }

        estimator->simulatedInterval = estimator->warmupSimulatedTime / estimator->warmupSamples;
        estimator->realInterval = estimator->warmupRealTime / estimator->warmupSamples;
        estimator->initialized = true;
    } else {
        // Filter both intervals before division. Filtering their ratio would
        // still let one near-zero real interval create an unbounded sample.
        estimator->simulatedInterval += (deltaSim - estimator->simulatedInterval) * SITL_RATE_FILTER_WEIGHT;
        estimator->realInterval += (deltaReal - estimator->realInterval) * SITL_RATE_FILTER_WEIGHT;
    }

    if (estimator->realInterval <= 0.0) {
        return false;
    }

    *rate = estimator->simulatedInterval / estimator->realInterval;
    return true;
}

typedef struct sitlPause_s {
    double stillRealTime;
    bool paused;
} sitlPause_t;

// Paused: the timestamp did not change for SITL_PAUSE_SECONDS. Returns true when the state changes
static inline bool sitlPauseUpdate(sitlPause_t *pause, double deltaSim, double deltaReal)
{
    const bool wasPaused = pause->paused;

    if (deltaSim > 0.0) {
        pause->stillRealTime = 0.0;
        pause->paused = false;
    } else {
        pause->stillRealTime += deltaReal;
        pause->paused = pause->stillRealTime >= SITL_PAUSE_SECONDS;
    }

    return pause->paused != wasPaused;
}
