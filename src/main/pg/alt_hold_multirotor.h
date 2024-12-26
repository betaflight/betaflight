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

#pragma once

#ifndef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef struct altHoldConfig_s {
    uint8_t alt_hold_adjust_rate;
    uint8_t alt_hold_deadband;
} altHoldConfig_t;

PG_DECLARE(altHoldConfig_t, altHoldConfig);

#endif // !USE_WING
