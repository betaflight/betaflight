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

#include <stdbool.h>

#include "common/time.h"
#include "common/filter.h"

#include "pg/motor.h"
#include "pg/rpm_filter.h"

#define RPM_FILTER_DURATION_S    0.001f  // Maximum duration allowed to update all RPM notches once

typedef struct rpmFilter_s {

    int numHarmonics;
    float weights[RPM_FILTER_HARMONICS_MAX];
    float minHz;
    float maxHz;
    float fadeRangeHz;
    float q;

    float dt;
    biquadFilterVec3_t notch[MAX_SUPPORTED_MOTORS][RPM_FILTER_HARMONICS_MAX];

} rpmFilter_t;

void rpmFilterInit(const rpmFilterConfig_t *config, const float dt);
void rpmFilterUpdate(void);
float rpmFilterApply(const int axis, float value);
bool isRpmFilterEnabled(void);
