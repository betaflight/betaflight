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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"
#include "common/vector.h"
#include "common/time.h"

// Unified position estimate from Kalman-filter sensor fusion.
// All values in local ENU (East-North-Up) centimeters, zeroed at arm point.
typedef struct positionEstimate3d_s {
    vector3_t position;        // cm, ENU
    vector3_t velocity;        // cm/s, ENU
    float trustXY;             // 0-1, derived from KF XY covariance
    float trustZ;              // 0-1, derived from KF Z covariance
    bool isValidXY;            // true if at least one XY measurement source active
    bool isValidZ;             // true if at least one Z measurement source active
} positionEstimate3d_t;

void positionEstimatorInit(void);
void positionEstimatorUpdate(void);

// Enable/disable XY estimation (gated by position hold / rescue modes)
void positionEstimatorEnableXY(bool enable);

// Read the unified estimate
const positionEstimate3d_t *positionEstimatorGetEstimate(void);

float positionEstimatorGetAltitudeCm(void);
float positionEstimatorGetAltitudeDerivative(void);
bool positionEstimatorIsValidXY(void);
bool positionEstimatorIsValidZ(void);
float positionEstimatorGetTrustXY(void);

// Reset (e.g. on arm/disarm)
void positionEstimatorResetZ(void);
void positionEstimatorResetXY(void);
