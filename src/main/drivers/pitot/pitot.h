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

#pragma once

#include "drivers/bus.h"

struct pitotDev_s;

// Fills differential pressure (Pa, signed) and temperature (kelvin) from one
// sensor read. Returns false if the transfer failed or the sample was stale.
typedef bool (*pitotReadFuncPtr)(struct pitotDev_s *pitot, float *diffPressurePa, float *temperatureK);

typedef struct pitotDev_s {
    extDevice_t dev;
    pitotReadFuncPtr read;
} pitotDev_t;
