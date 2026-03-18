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

#include <stdbool.h>
#include <stdint.h>
#include "common/time.h"
#include "pg/pg.h"

typedef enum {
    OSD_CUSTOM_TEXT_TERMINATOR_NULL = 0,
    OSD_CUSTOM_TEXT_TERMINATOR_LF = 1,
} osdCustomTextTerminator_e;

typedef struct osdCustomTextConfig_s {
    uint8_t terminator;
} osdCustomTextConfig_t;

PG_DECLARE(osdCustomTextConfig_t, osdCustomTextConfig);

bool osdCustomTextInit(void);
void osdCustomTextUpdate(timeUs_t currentTimeUs);
const char* osdCustomTextGet(void);
