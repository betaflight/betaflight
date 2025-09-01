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

void dynNotchInit(const dynNotchConfig_t *config, const float dt);
void dynNotchPush(const vector3_t *sample);
void dynNotchUpdate(void);
void dynNotchFilter(vector3_t* dst, const vector3_t* src);
bool isDynNotchActive(void);
int getMaxFFT(void);
void resetMaxFFT(void);
