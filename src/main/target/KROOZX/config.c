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

#include "config/config_master.h"

#define VBAT_SCALE       113
#define CURRENT_SCALE    1000
#define CURRENT_OFFSET   0

#define OSD_POS(x,y)  (x | (y << 5))

#ifdef TARGET_CONFIG
void targetConfiguration(master_t *config)
{
    config->batteryConfig.vbatscale = VBAT_SCALE;
    config->batteryConfig.currentMeterScale = CURRENT_SCALE;
    config->batteryConfig.currentMeterOffset = CURRENT_OFFSET;
    config->barometerConfig.baro_hardware = 0;
    config->compassConfig.mag_hardware = 0;
// Disabled to make it build for 3.1.7
//    config->osdConfig.item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(12, 1) | VISIBLE_FLAG;
}
#endif
