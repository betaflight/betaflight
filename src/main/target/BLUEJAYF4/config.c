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

#include <stdint.h>
#include <stdbool.h>

#include <platform.h>

#include "config/config_master.h"

#include "hardware_revision.h"

// alternative defaults settings for BlueJayF4 targets
void targetConfiguration(master_t *config)
{
    if (hardwareRevision == BJF4_REV1 || hardwareRevision == BJF4_REV2) {
        config->sensorAlignmentConfig.gyro_align = CW180_DEG;
        config->sensorAlignmentConfig.acc_align  = CW180_DEG;
        config->beeperConfig.ioTag = IO_TAG(BEEPER_OPT);
    }
}
