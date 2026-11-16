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

#include <stdbool.h>
#include <stdint.h>

#include "drivers/rangefinder/rangefinder.h"

// The datasheet quotes up to 126fps in good light, but the module was measured
// framing every 20ms, so the tasks run at twice that to avoid aliasing against
// it.
#define RANGEFINDER_PB04_TASK_PERIOD_MS 10

bool pb04RangefinderDetect(rangefinderDev_t *dev);

#ifdef USE_OPTICALFLOW_PB04
#include "drivers/opticalflow/opticalflow.h"
bool pb04OpticalflowDetect(opticalflowDev_t *dev);
#endif
