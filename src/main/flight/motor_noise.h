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

/*
 * Motor command noise index: high-frequency RMS of each motor's final output command.
 * Proxy for winding/ESC heating caused by D-term and filter noise (see research/motor-thermal).
 */
#pragma once

#include <stdint.h>

void motorNoiseInit(float looptimeUs);
void motorNoiseUpdate(const float *motorOutput, int motorCount);
float motorNoiseRms(int motor);   // command ripple in the 20 Hz..2 kHz band, external (1000..2000) command units
float motorNoiseMsq(int motor);   // its mean square
