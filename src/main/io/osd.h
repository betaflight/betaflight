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
#include "pg/pg.h"

#define OSD_NUM_TIMER_TYPES 3
extern const char * const osdTimerSourceNames[OSD_NUM_TIMER_TYPES];

#define OSD_ELEMENT_BUFFER_LENGTH 32

#define VISIBLE_FLAG  0x0800
#define VISIBLE(x)    (x & VISIBLE_FLAG)
#define OSD_POS_MAX   0x3FF
#define OSD_POSCFG_MAX   (VISIBLE_FLAG|0x3FF) // For CLI values

// Character coordinate
#define OSD_POSITION_BITS 5 // 5 bits gives a range 0-31
#define OSD_POSITION_XY_MASK ((1 << OSD_POSITION_BITS) - 1)
#define OSD_POS(x,y)  ((x & OSD_POSITION_XY_MASK) | ((y & OSD_POSITION_XY_MASK) << OSD_POSITION_BITS))
#define OSD_X(x)      (x & OSD_POSITION_XY_MASK)
#define OSD_Y(x)      ((x >> OSD_POSITION_BITS) & OSD_POSITION_XY_MASK)

// Timer configuration
// Stored as 15[alarm:8][precision:4][source:4]0
#define OSD_TIMER(src, prec, alarm) ((src & 0x0F) | ((prec & 0x0F) << 4) | ((alarm & 0xFF ) << 8))
#define OSD_TIMER_SRC(timer)        (timer & 0x0F)
#define OSD_TIMER_PRECISION(timer)  ((timer >> 4) & 0x0F)
#define OSD_TIMER_ALARM(timer)      ((timer >> 8) & 0xFF)

typedef enum {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
    OSD_CROSSHAIRS,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ITEM_TIMER_1,
    OSD_ITEM_TIMER_2,
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
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
    OSD_GPS_LON,
    OSD_GPS_LAT,
    OSD_DEBUG,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATT_USAGE,
    OSD_DISARMED,
    OSD_HOME_DIR,
    OSD_HOME_DIST,
    OSD_NUMERICAL_HEADING,
    OSD_NUMERICAL_VARIO,
    OSD_COMPASS_BAR,
    OSD_ESC_TMP,
    OSD_ESC_RPM,
    OSD_REMAINING_TIME_ESTIMATE,
    OSD_RTC_DATETIME,
    OSD_ADJUSTMENT_RANGE,
    OSD_CORE_TEMPERATURE,
    OSD_ITEM_COUNT // MUST BE LAST
} osd_items_e;

typedef enum {
    OSD_STAT_MAX_SPEED,
    OSD_STAT_MIN_BATTERY,
    OSD_STAT_MIN_RSSI,
    OSD_STAT_MAX_CURRENT,
    OSD_STAT_USED_MAH,
    OSD_STAT_MAX_ALTITUDE,
    OSD_STAT_BLACKBOX,
    OSD_STAT_END_BATTERY,
    OSD_STAT_TIMER_1,
    OSD_STAT_TIMER_2,
    OSD_STAT_MAX_DISTANCE,
    OSD_STAT_BLACKBOX_NUMBER,
    OSD_STAT_RTC_DATE_TIME,
    OSD_STAT_COUNT // MUST BE LAST
} osd_stats_e;

typedef enum {
    OSD_UNIT_IMPERIAL,
    OSD_UNIT_METRIC
} osd_unit_e;

typedef enum {
    OSD_TIMER_1,
    OSD_TIMER_2,
    OSD_TIMER_COUNT
} osd_timer_e;

typedef enum {
    OSD_TIMER_SRC_ON,
    OSD_TIMER_SRC_TOTAL_ARMED,
    OSD_TIMER_SRC_LAST_ARMED,
    OSD_TIMER_SRC_COUNT
} osd_timer_source_e;

typedef enum {
    OSD_TIMER_PREC_SECOND,
    OSD_TIMER_PREC_HUNDREDTHS,
    OSD_TIMER_PREC_COUNT
} osd_timer_precision_e;

typedef enum {
    OSD_WARNING_ARMING_DISABLE    = (1 << 0),
    OSD_WARNING_BATTERY_NOT_FULL  = (1 << 1),
    OSD_WARNING_BATTERY_WARNING   = (1 << 2),
    OSD_WARNING_BATTERY_CRITICAL  = (1 << 3),
    OSD_WARNING_VISUAL_BEEPER     = (1 << 4),
    OSD_WARNING_CRASH_FLIP        = (1 << 5)
} osdWarningsFlags_e;

typedef struct osdConfig_s {
    uint16_t item_pos[OSD_ITEM_COUNT];

    // Alarms
    uint16_t cap_alarm;
    uint16_t alt_alarm;
    uint8_t rssi_alarm;

    osd_unit_e units;

    uint16_t timers[OSD_TIMER_COUNT];
    uint16_t enabledWarnings;

    uint8_t ahMaxPitch;
    uint8_t ahMaxRoll;
    bool enabled_stats[OSD_STAT_COUNT];
} osdConfig_t;

extern timeUs_t resumeRefreshAt;

PG_DECLARE(osdConfig_t, osdConfig);

extern uint32_t resumeRefreshAt;

struct displayPort_s;
void osdInit(struct displayPort_s *osdDisplayPort);
void osdResetAlarms(void);
void osdUpdate(timeUs_t currentTimeUs);
