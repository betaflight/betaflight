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

#ifndef PINIO_COUNT
#define PINIO_COUNT 8
#endif

#define PINIO_CONFIG_OUT_INVERTED 0x80
#define PINIO_CONFIG_MODE_MASK    0x7F
#define PINIO_CONFIG_MODE_OUT_PP  0x01

struct pinioConfig_s;

void pinioInit(const struct pinioConfig_s *pinioConfig);
void pinioSet(int index, bool on);
