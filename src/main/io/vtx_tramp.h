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

#include <stdint.h>

#define VTX_TRAMP_POWER_COUNT 5

#define VTX_TRAMP_MIN_FREQUENCY_MHZ 5000             //min freq in MHz
#define VTX_TRAMP_MAX_FREQUENCY_MHZ 5999             //max freq in MHz

bool vtxTrampInit(void);

uint16_t vtxTrampGetCurrentActualPower();
uint16_t vtxTrampGetCurrentTemp();
