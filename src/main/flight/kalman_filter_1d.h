/*
 * This file is part of Betaflight.
 *
 * Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

typedef struct {
    float estimatedValue;
    float estimatedVariance;
    float processVariance;
} KalmanFilter;

typedef struct {
    float value;
    float variance;
} SensorMeasurement;

void kf_init(KalmanFilter *kf, float initialValue, float initialVariance, float processVariance);
void kf_update_variance(KalmanFilter *kf);
void kf_update(KalmanFilter *kf, SensorMeasurement sensorMeas);
void updateBaroVariance(SensorMeasurement * sensorMeas);
