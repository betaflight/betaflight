/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
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

// NB: to ensure backwards compatibility, new enum values must be appended at the end but before the OSD_XXXX_COUNT entry.

// *** IMPORTANT ***
// If you are adding additional elements that do not require any conditional display logic,
// you must add the elements to the osdElementDisplayOrder[] array in src/main/io/osd.c
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
    OSD_ANTI_GRAVITY,
    OSD_MOTOR_DIAG,
    OSD_G_FORCE,
    OSD_LOG_STATUS,
    OSD_FLIP_ARROW,
    OSD_LINK_QUALITY,
    OSD_TOTAL_DIST,
    OSD_ITEM_COUNT // MUST BE LAST
} osd_items_e;

// *** IMPORTANT ***
// The order of the OSD stats enumeration *must* match the order they're displayed on-screen
// This is because the fields are presented in the configurator in the order of the enumeration
// and we want the configuration order to match the on-screen display order.
// Changes to the stats display order *must* be implemented in the configurator otherwise the
// stats selections will not be populated correctly and the settings will become corrupted.
//
// Also - if the stats are reordered then the PR version must be incremented. Otherwise there
// is no indication that the stored config must be reset and the bitmapped values will be incorrect.
typedef enum {
    OSD_STAT_RTC_DATE_TIME,
    OSD_STAT_TIMER_1,
    OSD_STAT_TIMER_2,
    OSD_STAT_MAX_SPEED,
    OSD_STAT_MAX_DISTANCE,
    OSD_STAT_MIN_BATTERY,
    OSD_STAT_END_BATTERY,
    OSD_STAT_BATTERY,
    OSD_STAT_MIN_RSSI,
    OSD_STAT_MAX_CURRENT,
    OSD_STAT_USED_MAH,
    OSD_STAT_MAX_ALTITUDE,
    OSD_STAT_BLACKBOX,
    OSD_STAT_BLACKBOX_NUMBER,
    OSD_STAT_MAX_G_FORCE,
    OSD_STAT_MAX_ESC_TEMP,
    OSD_STAT_MAX_ESC_RPM,
    OSD_STAT_MIN_LINK_QUALITY,
    OSD_STAT_TOTAL_DISTANCE,
    OSD_STAT_COUNT // MUST BE LAST
} osd_stats_e;

// Make sure the number of stats do not exceed the available 32bit storage
STATIC_ASSERT(OSD_STAT_COUNT <= 32, osdstats_overflow);

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
    OSD_WARNING_ARMING_DISABLE,
    OSD_WARNING_BATTERY_NOT_FULL,
    OSD_WARNING_BATTERY_WARNING,
    OSD_WARNING_BATTERY_CRITICAL,
    OSD_WARNING_VISUAL_BEEPER,
    OSD_WARNING_CRASH_FLIP,
    OSD_WARNING_ESC_FAIL,
    OSD_WARNING_CORE_TEMPERATURE,
    OSD_WARNING_RC_SMOOTHING,
    OSD_WARNING_FAIL_SAFE,
    OSD_WARNING_LAUNCH_CONTROL,
    OSD_WARNING_COUNT // MUST BE LAST
} osdWarningsFlags_e;

// Make sure the number of warnings do not exceed the available 16bit storage
STATIC_ASSERT(OSD_WARNING_COUNT <= 16, osdwarnings_overflow);

#define ESC_RPM_ALARM_OFF -1
#define ESC_TEMP_ALARM_OFF INT8_MIN
#define ESC_CURRENT_ALARM_OFF -1

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
    uint32_t enabled_stats;
    int8_t esc_temp_alarm;
    int16_t esc_rpm_alarm;
    int16_t esc_current_alarm;
    uint8_t core_temp_alarm;
    uint8_t ahInvert;         // invert the artificial horizon
} osdConfig_t;

PG_DECLARE(osdConfig_t, osdConfig);

extern timeUs_t resumeRefreshAt;

struct displayPort_s;
void osdInit(struct displayPort_s *osdDisplayPort);
bool osdInitialized(void);
void osdResetAlarms(void);
void osdUpdate(timeUs_t currentTimeUs);
void osdStatSetState(uint8_t statIndex, bool enabled);
bool osdStatGetState(uint8_t statIndex);
void osdWarnSetState(uint8_t warningIndex, bool enabled);
bool osdWarnGetState(uint8_t warningIndex);
void osdSuppressStats(bool flag);


