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

#include "platform.h"

#ifdef USE_MOTOR_THERMAL

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/motor_thermal.h"

PG_REGISTER_WITH_RESET_TEMPLATE(motorThermalConfig_t, motorThermalConfig, PG_MOTOR_THERMAL_CONFIG, 1);

PG_RESET_TEMPLATE(motorThermalConfig_t, motorThermalConfig,
    // default geometry is the common 5" motor (2207); the bench reference (2203.5) is the calibration
    // anchor the constants are scaled from, not the default a user is most likely to fly
    .stator_diameter_mm = 22,
    .stator_height_dmm = 70,
    .prop_diameter_dinch = 0,
    .gain_percent = 100,
    .warning_c = 0,               // off: the model is bench-checked only, a false MOTOR HOT is worse than none
    .resistance_mohm = 0,
    .idle_current_ma = 0
);

#endif
