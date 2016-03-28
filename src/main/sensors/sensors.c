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

#include "sensors/sensors.h"

sensorSelectionConfig_t sensorSelectionConfig;
sensorAlignmentConfig_t sensorAlignmentConfig;
sensorTrims_t sensorTrims;

static const pgRegistry_t sensorSelectionConfigRegistry PG_REGISTRY_SECTION =
{
    .base = (uint8_t *)&sensorSelectionConfig,
    .size = sizeof(sensorSelectionConfig),
    .pgn = PG_SENSOR_SELECTION_CONFIG,
    .format = 0,
    .flags = PGC_SYSTEM
};

static const pgRegistry_t sensorAlignmentConfigRegistry PG_REGISTRY_SECTION =
{
    .base = (uint8_t *)&sensorAlignmentConfig,
    .size = sizeof(sensorAlignmentConfig),
    .pgn = PG_SENSOR_ALIGNMENT_CONFIG,
    .format = 0,
    .flags = PGC_SYSTEM
};

static const pgRegistry_t sensorTrimsRegistry PG_REGISTRY_SECTION =
{
    .base = (uint8_t *)&sensorTrims,
    .size = sizeof(sensorTrims),
    .pgn = PG_SENSOR_TRIMS,
    .format = 0,
    .flags = PGC_SYSTEM
};
