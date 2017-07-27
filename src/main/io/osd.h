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

#ifdef OSD
#include "common/time.h"
#include "config/parameter_group.h"

#define OSD_NUM_TIMER_TYPES 3
extern const char * const osdTimerSourceNames[OSD_NUM_TIMER_TYPES];

#define OSD_ELEMENT_BUFFER_LENGTH 32

#define VISIBLE_FLAG  0x0800
#define VISIBLE(x)    (x & VISIBLE_FLAG)
#define OSD_POS_MAX   0x3FF
#define OSD_POSCFG_MAX   (VISIBLE_FLAG|0x3FF) // For CLI values

// Character coordinate
//#define OSD_POSITION_BITS 5 // 5 bits gives a range 0-31
//#define OSD_POSITION_XY_MASK ((1 << OSD_POSITION_BITS) - 1)
//#define OSD_POS(x,y)  {(x), (y)}
//#define OSD_X(x)      (x & OSD_POSITION_XY_MASK)
//#define OSD_Y(x)      ((x >> OSD_POSITION_BITS) & OSD_POSITION_XY_MASK)

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
    // visible, bit 0
    OSD_FLAG_VISIBLE = (1 << 0),

    // bit 1 .. 3 reserved for future use

    // origin, bits 4.7
    OSD_FLAG_ORIGIN_C  = 0,
    OSD_FLAG_ORIGIN_N  = (1<<4),
    OSD_FLAG_ORIGIN_E  = (1<<5),
    OSD_FLAG_ORIGIN_S  = (1<<6),
    OSD_FLAG_ORIGIN_W  = (1<<7),
    OSD_FLAG_ORIGIN_NE = OSD_FLAG_ORIGIN_N | OSD_FLAG_ORIGIN_E,
    OSD_FLAG_ORIGIN_SE = OSD_FLAG_ORIGIN_S | OSD_FLAG_ORIGIN_E,
    OSD_FLAG_ORIGIN_SW = OSD_FLAG_ORIGIN_S | OSD_FLAG_ORIGIN_W,
    OSD_FLAG_ORIGIN_NW = OSD_FLAG_ORIGIN_N | OSD_FLAG_ORIGIN_W
} osdFlag_e;

#define OSD_FLAG_VISIBLE_MASK (OSD_FLAG_VISIBLE)
#define OSD_FLAG_ORIGIN_MASK  (OSD_FLAG_ORIGIN_N | OSD_FLAG_ORIGIN_E | OSD_FLAG_ORIGIN_S | OSD_FLAG_ORIGIN_W)

#define OSD_INIT(_config, _item, _x, _y, _flags) { (_config)->item[(_item)] = (osdItem_t) {(_x), (_y), (int8_t)(_flags)}; }


typedef struct {
    int8_t x;
    int8_t y;
    int8_t flags;
} osdItem_t;

typedef struct osdConfig_s {
    osdItem_t item[OSD_ITEM_COUNT];

    bool enabled_stats[OSD_STAT_COUNT];

    // Alarms
    uint16_t cap_alarm;
    uint16_t alt_alarm;
    uint8_t rssi_alarm;

    osd_unit_e units;

    uint16_t timers[OSD_TIMER_COUNT];
} osdConfig_t;

extern uint32_t resumeRefreshAt;

PG_DECLARE(osdConfig_t, osdConfig);

struct displayPort_s;
void osdInit(struct displayPort_s *osdDisplayPort);
void osdResetConfig(osdConfig_t *osdProfile);
void osdResetAlarms(void);
void osdUpdate(timeUs_t currentTimeUs);

#endif
