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

#include "drivers/io_types.h"
#include "drivers/pin_pull_up_down.h"
#include "pg/pg.h"

typedef struct pinPullUpDownConfig_s {
    ioTag_t ioTag;
} pinPullUpDownConfig_t;

PG_DECLARE_ARRAY(pinPullUpDownConfig_t, PIN_PULL_UP_DOWN_COUNT, pinPullupConfig);
PG_DECLARE_ARRAY(pinPullUpDownConfig_t, PIN_PULL_UP_DOWN_COUNT, pinPulldownConfig);
