/*
 * This file is part of Betaflight.
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

#include "../drivers/pin_up_down.h"

#include "pg/pg.h"
#include "drivers/io_types.h"

typedef struct pinUpDownConfig_s {
    ioTag_t ioTag[PIN_UP_DOWN_COUNT];
} pinUpDownConfig_t;

PG_DECLARE(pinUpDownConfig_t, pinPullupConfig);
PG_DECLARE(pinUpDownConfig_t, pinPulldownConfig);
