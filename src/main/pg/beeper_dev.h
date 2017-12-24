/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/io_types.h"

#include "pg/pg.h"
#include "drivers/io_types.h"

typedef struct beeperDevConfig_s {
    ioTag_t ioTag;
    uint8_t isInverted;
    uint8_t isOpenDrain;
    uint16_t frequency;
} beeperDevConfig_t;

PG_DECLARE(beeperDevConfig_t, beeperDevConfig);
