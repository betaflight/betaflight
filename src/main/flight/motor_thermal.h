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

/*
 * Motor thermal estimator: winding temperature per motor from rpm, battery voltage and the
 * motor-command noise index. Two thermal nodes (windings, stator core), convection vs air speed.
 * Calibrated on a 2203.5 / 2800 KV motor; scaled by stator size and KV.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"

void motorThermalInit(void);
void motorThermalUpdate(timeUs_t currentTimeUs);                     // scheduler task, 100 Hz
float motorThermalGetTemp(int motor);                                  // estimated winding temperature, °C
float motorThermalGetMaxTemp(void);
bool motorThermalWarning(void);
// steady winding temperature rise from command ripple (motorNoiseRms(), external units) at rpm and vbat
float motorThermalRippleRise(float noiseRms, float rpm, float vbat);
