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

#include "pg/pg.h"
#include "drivers/io_types.h"

#define MAX_PINIO 4

typedef struct pinioConfig_s {
    ioTag_t ioTag[MAX_PINIO];
    uint8_t config[MAX_PINIO];
} pinioConfig_t;

#define PINIO_CONFIG_OUT_PP       0x01
#define PINIO_CONFIG_OUT_INVERTED 0x80

PG_DECLARE(pinioConfig_t, pinioConfig);

void pinioInit(const pinioConfig_t *pinioConfig);
void pinioON(int index);
void pinioOFF(int index);
