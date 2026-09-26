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
#include "flight/pos_hold_multirotor.h"
#include "flight/autopilot_multirotor.h"
#include "flight/autopilot_wing.h"

// The most position error the position controller acts on under nav (cm): a nav reference held
// further from the craft than this adds nothing to the correction and only winds up.
#define NAV_ERROR_DISTANCE_LIMIT 500.0f
