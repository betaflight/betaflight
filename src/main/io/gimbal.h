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

#include "config/parameter_group.h"

typedef enum {
    GIMBAL_MODE_NORMAL = 0,
    GIMBAL_MODE_MIXTILT = 1
} gimbalMode_e;

#define GIMBAL_MODE_MAX (GIMBAL_MODE_MIXTILT)

typedef struct gimbalConfig_s {
    uint8_t mode;
} gimbalConfig_t;

PG_DECLARE(gimbalConfig_t, gimbalConfig);
