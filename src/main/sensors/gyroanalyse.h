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

#pragma once

#include "common/time.h"
#include "common/filter.h"

#define GYRO_FFT_BIN_COUNT      16 // FFT_WINDOW_SIZE / 2
typedef struct gyroFftData_s {
    float maxVal;
    uint16_t centerFreq;
} gyroFftData_t;

void gyroDataAnalyseInit(uint32_t targetLooptime);
const gyroFftData_t *gyroFftData(int axis);
struct gyroDev_s;
void gyroDataAnalyse(const struct gyroDev_s *gyroDev, biquadFilter_t *notchFilterDyn);
void gyroDataAnalyseUpdate(biquadFilter_t *notchFilterDyn);
