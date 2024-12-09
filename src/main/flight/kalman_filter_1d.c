/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"
#include "math.h"
#include <stdbool.h>

#include "kalman_filter_1d.h"


void kf_init(KalmanFilter *kf, float initialValue, float initialVariance, float processVariance) {
    kf->estimatedValue    = initialValue;
    kf->estimatedVariance = initialVariance;
    kf->processVariance   = processVariance;
}

void kf_update_variance(KalmanFilter *kf) {
    // Increase the estimated variance by the process variance
    kf->estimatedVariance += kf->processVariance;
}

void kf_update(KalmanFilter *kf, SensorMeasurement sensorMeas) {
    float kalmanGain = kf->estimatedVariance / (kf->estimatedVariance + sensorMeas.variance);

    kf->estimatedValue += kalmanGain * (sensorMeas.value - kf->estimatedValue);

    kf->estimatedVariance = (1 - kalmanGain) * kf->estimatedVariance;
}

#ifdef USE_BARO
void updateBaroVariance2(SensorMeasurement * sensorMeas) {
    static uint16_t n = 0;
    if (n == 0) {
        sensorMeas->variance = 0.0f;
    }
    sensorMeas->variance = (sensorMeas->variance * n + powf(sensorMeas->value, 2)) / (n + 1);
    n++;
}
#endif