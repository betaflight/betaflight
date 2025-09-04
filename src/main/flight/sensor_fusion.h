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

#pragma once

#include "common/axis.h"
#include "common/filter.h"
#include "platform.h"

#if GYRO_COUNT > 1

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#pragma message ("GYRO_COUNT = " TOSTRING(GYRO_COUNT))

typedef enum {
    AVERAGING,
    NOISE_APPROX,
    MEDIAN,
    VOTING,
} fusionType_e;

typedef struct varCovApprox_s {
    pt1Filter_t inputHighpass[GYRO_COUNT][XYZ_AXIS_COUNT];
    pt1Filter_t average[GYRO_COUNT][XYZ_AXIS_COUNT];
    pt1Filter_t variance[GYRO_COUNT][XYZ_AXIS_COUNT];
    pt1Filter_t covariance[GYRO_COUNT * (GYRO_COUNT - 1) / 2][XYZ_AXIS_COUNT];
} varCovApprox_t;

typedef struct sensorFusion_s {
    varCovApprox_t varCov;
    uint8_t clusterSize;
} sensorFusion_t;

#endif
