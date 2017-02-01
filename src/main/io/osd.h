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

#include "common/time.h"
#include "config/parameter_group.h"

#define VISIBLE_FLAG  0x0800
#define VISIBLE(x)    (x & VISIBLE_FLAG)
#define OSD_POS_MAX   0x3FF
#define OSD_POSCFG_MAX   (VISIBLE_FLAG|0x3FF) // For CLI values

typedef enum {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
    OSD_CROSSHAIRS,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ONTIME,
    OSD_FLYTIME,
    OSD_FLYMODE,
    OSD_CRAFT_NAME,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PIDRATE_PROFILE,
    OSD_MAIN_BATT_WARNING,
    OSD_ITEM_COUNT // MUST BE LAST
} osd_items_e;

typedef enum {
    OSD_UNIT_IMPERIAL,
    OSD_UNIT_METRIC
} osd_unit_e;

typedef struct osd_profile_s {
    uint16_t item_pos[OSD_ITEM_COUNT];

    // Alarms
    uint8_t rssi_alarm;
    uint16_t cap_alarm;
    uint16_t time_alarm;
    uint16_t alt_alarm;

    osd_unit_e units;
} osd_profile_t;

// !!TODO change to osdConfig_t
PG_DECLARE(osd_profile_t, osdConfig);

struct displayPort_s;
void osdInit(struct displayPort_s *osdDisplayPort);
void osdResetConfig(osd_profile_t *osdProfile);
void osdResetAlarms(void);
void osdUpdate(timeUs_t currentTimeUs);
