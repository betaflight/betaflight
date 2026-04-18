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

#include "drivers/can/can.h"
#include "drivers/io_types.h"

#include "pg/pg.h"

typedef struct canPinConfig_s {
    ioTag_t ioTagTx;
    ioTag_t ioTagRx;
} canPinConfig_t;

PG_DECLARE_ARRAY(canPinConfig_t, CANDEV_COUNT, canPinConfig);
