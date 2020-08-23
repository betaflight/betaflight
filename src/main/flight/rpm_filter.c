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
#include <stdint.h>

#include "platform.h"

#if defined(USE_RPM_FILTER)

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

#define RPM_FILTER_MAXHARMONICS 3
#define SECONDS_PER_MINUTE      60.0f
#define ERPM_PER_LSB            100.0f
#define MIN_UPDATE_T            0.001f


static pt1Filter_t rpmFilters[MAX_SUPPORTED_MOTORS];

typedef struct rpmNotchFilter_s
{
    uint8_t harmonics;
    float   minHz;
    float   maxHz;
    float   q;
    float   loopTime;

    biquadFilter_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_MAXHARMONICS];
} rpmNotchFilter_t;

FAST_DATA_ZERO_INIT static float   erpmToHz;
FAST_DATA_ZERO_INIT static float   filteredMotorErpm[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT static float   minMotorFrequency;
FAST_DATA_ZERO_INIT static uint8_t numberFilters;
FAST_DATA_ZERO_INIT static uint8_t numberRpmNotchFilters;
FAST_DATA_ZERO_INIT static uint8_t filterUpdatesPerIteration;
FAST_DATA_ZERO_INIT static float   pidLooptime;
FAST_DATA_ZERO_INIT static rpmNotchFilter_t filters[2];
FAST_DATA_ZERO_INIT static rpmNotchFilter_t* gyroFilter;

FAST_DATA_ZERO_INIT static uint8_t currentMotor;
FAST_DATA_ZERO_INIT static uint8_t currentHarmonic;
FAST_DATA_ZERO_INIT static uint8_t currentFilterNumber;
FAST_DATA static rpmNotchFilter_t* currentFilter = &filters[0];



PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 4);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    config->gyro_rpm_notch_harmonics = 3;
    config->gyro_rpm_notch_min = 100;
    config->gyro_rpm_notch_q = 500;

    config->rpm_lpf = 150;
}

static void rpmNotchFilterInit(rpmNotchFilter_t* filter, int harmonics, int minHz, int q, float looptime)
{
    filter->harmonics = harmonics;
    filter->minHz = minHz;
    filter->q = q / 100.0f;
    filter->loopTime = looptime;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int i = 0; i < harmonics; i++) {
                biquadFilterInit(
                    &filter->notch[axis][motor][i], minHz * i, looptime, filter->q, FILTER_NOTCH);
            }
        }
    }
}

void rpmFilterInit(const rpmFilterConfig_t *config)
{
    currentFilter = &filters[0];
    currentMotor = currentHarmonic = currentFilterNumber = 0;

    numberRpmNotchFilters = 0;
    if (!motorConfig()->dev.useDshotTelemetry) {
        gyroFilter = NULL;
        return;
    }

    pidLooptime = gyro.targetLooptime;
    if (config->gyro_rpm_notch_harmonics) {
        gyroFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(gyroFilter, config->gyro_rpm_notch_harmonics,
                           config->gyro_rpm_notch_min, config->gyro_rpm_notch_q, gyro.targetLooptime);
        // don't go quite to nyquist to avoid oscillations
        gyroFilter->maxHz = 0.48f / (gyro.targetLooptime * 1e-6f);
    } else {
        gyroFilter = NULL;
    }

    for (int i = 0; i < getMotorCount(); i++) {
        pt1FilterInit(&rpmFilters[i], pt1FilterGain(config->rpm_lpf, pidLooptime * 1e-6f));
    }

    erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE  / (motorConfig()->motorPoleCount / 2.0f);

    const float loopIterationsPerUpdate = MIN_UPDATE_T / (pidLooptime * 1e-6f);
    numberFilters = getMotorCount() * (filters[0].harmonics + filters[1].harmonics);
    const float filtersPerLoopIteration = numberFilters / loopIterationsPerUpdate;
    filterUpdatesPerIteration = rintf(filtersPerLoopIteration + 0.49f);
}

static float applyFilter(rpmNotchFilter_t* filter, int axis, float value)
{
    if (filter == NULL) {
        return value;
    }
    for (int motor = 0; motor < getMotorCount(); motor++) {
        for (int i = 0; i < filter->harmonics; i++) {
            value = biquadFilterApplyDF1(&filter->notch[axis][motor][i], value);
        }
    }
    return value;
}

float rpmFilterGyro(int axis, float value)
{
    return applyFilter(gyroFilter, axis, value);
}

FAST_DATA_ZERO_INIT static float motorFrequency[MAX_SUPPORTED_MOTORS];

FAST_CODE_NOINLINE void rpmFilterUpdate()
{
    if (gyroFilter == NULL) {
        return;
    }

    for (int motor = 0; motor < getMotorCount(); motor++) {
        filteredMotorErpm[motor] = pt1FilterApply(&rpmFilters[motor], getDshotTelemetry(motor));
        if (motor < 4) {
            DEBUG_SET(DEBUG_RPM_FILTER, motor, motorFrequency[motor]);
        }
    }

    for (int i = 0; i < filterUpdatesPerIteration; i++) {
        float frequency = constrainf(
            (currentHarmonic + 1) * motorFrequency[currentMotor], currentFilter->minHz, currentFilter->maxHz);
        biquadFilter_t* template = &currentFilter->notch[0][currentMotor][currentHarmonic];
        // uncomment below to debug filter stepping. Need to also comment out motor rpm DEBUG_SET above
        /* DEBUG_SET(DEBUG_RPM_FILTER, 0, harmonic); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 1, motor); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 2, currentFilter == &gyroFilter); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 3, frequency) */
        biquadFilterUpdate(
            template, frequency, currentFilter->loopTime, currentFilter->q, FILTER_NOTCH);
        for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilter_t* clone = &currentFilter->notch[axis][currentMotor][currentHarmonic];
            clone->b0 = template->b0;
            clone->b1 = template->b1;
            clone->b2 = template->b2;
            clone->a1 = template->a1;
            clone->a2 = template->a2;
        }

        if (++currentHarmonic == currentFilter->harmonics) {
            currentHarmonic = 0;
            if (++currentFilterNumber == numberRpmNotchFilters) {
                currentFilterNumber = 0;
                if (++currentMotor == getMotorCount()) {
                    currentMotor = 0;
                }
                motorFrequency[currentMotor] = erpmToHz * filteredMotorErpm[currentMotor];
                minMotorFrequency = 0.0f;
            }
            currentFilter = &filters[currentFilterNumber];
        }

    }
}

bool isRpmFilterEnabled(void)
{
    return (motorConfig()->dev.useDshotTelemetry && rpmFilterConfig()->gyro_rpm_notch_harmonics);
}

float rpmMinMotorFrequency()
{
    if (minMotorFrequency == 0.0f) {
        minMotorFrequency = 10000.0f;
        for (int i = getMotorCount(); i--;) {
            if (motorFrequency[i] < minMotorFrequency) {
                minMotorFrequency = motorFrequency[i];
            }
        }
    }
    return minMotorFrequency;
}


#endif
