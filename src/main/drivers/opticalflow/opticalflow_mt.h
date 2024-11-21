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

#include <stdint.h>

#include "drivers/rangefinder/rangefinder_lidarmt.h"

#include "sensors/opticalflow.h"

#define DT_OPTICALFLOW_MIN_RANGE 80  // mm

bool mtOpticalflowDetect(opticalflowDev_t * dev);
void mtOpticalflowReceiveNewData(const uint8_t * bufferPtr);
bool isMTRangefinderDetected(void);
