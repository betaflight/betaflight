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

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "debug.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "config/config.h"

#include "io/gimbal.h"

#ifdef USE_SERVOS

gimbalConfig_t gimbalConfigStorage[MAX_PROFILE_COUNT];
gimbalConfig_t *gimbalConfig;

static const pgRegistry_t gimbalConfigRegistry PG_REGISTRY_SECTION =
{
    .base = (uint8_t *)&gimbalConfigStorage,
    .ptr = (uint8_t **)&gimbalConfig,
    .size = sizeof(gimbalConfigStorage[0]),
    .pgn = PG_GIMBAL_CONFIG,
    .format = 0,
    .flags = PGC_PROFILE
};
#endif
