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
 * Motor thermal estimator configuration. See research/motor-thermal/RESULTS.md (bench calibration).
 */
#pragma once

#include <stdint.h>
#include "pg/pg.h"

typedef struct motorThermalConfig_s {
    uint8_t stator_diameter_mm;    // e.g. 22 for a 2203.5 motor
    uint8_t stator_height_dmm;     // 0.1 mm units, e.g. 35 for 3.5 mm
    uint8_t prop_diameter_dinch;   // 0.1 inch units, 0 = auto from stator size
    uint8_t gain_percent;          // copper-loss gain, 100 = bench reference; raise if motors run hotter than estimated
    uint8_t warning_c;             // winding temperature warning threshold, 0 = off (default)
    uint16_t resistance_mohm;      // line-to-line winding resistance, milliohm, 4-wire measured; 0 = size law from KV and stator (CLI only)
} motorThermalConfig_t;

PG_DECLARE(motorThermalConfig_t, motorThermalConfig);
