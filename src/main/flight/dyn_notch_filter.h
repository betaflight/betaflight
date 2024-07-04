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

#include "pg/dyn_notch.h"

#define DYN_NOTCH_COUNT_MAX 7

void dynNotchInit(const dynNotchConfig_t *config, const timeUs_t targetLooptimeUs);
void dynNotchPush(const int axis, const float sample);
void dynNotchUpdate(void);
float dynNotchFilter(const int axis, float value);
bool isDynNotchActive(void);
int getMaxFFT(void);
void resetMaxFFT(void);
