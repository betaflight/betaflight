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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#if defined(USE_OSD) && defined(USE_CMS)

#include "build/version.h"

#include "cli/settings.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_osd.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/display.h"

#include "io/displayport_max7456.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/crsf.h"

#include "sensors/battery.h"

#ifdef USE_EXTENDED_CMS_MENUS
static uint16_t osdConfig_item_pos[OSD_ITEM_COUNT];

static const void *menuOsdActiveElemsOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    memcpy(&osdConfig_item_pos[0], &osdElementConfig()->item_pos[0], sizeof(uint16_t) * OSD_ITEM_COUNT);
    return NULL;
}

static const void *menuOsdActiveElemsOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    memcpy(&osdElementConfigMutable()->item_pos[0], &osdConfig_item_pos[0], sizeof(uint16_t) * OSD_ITEM_COUNT);
    osdAnalyzeActiveElements();
    return NULL;
}

const OSD_Entry menuOsdActiveElemsEntries[] =
{
    {"--- ACTIV ELEM ---", OME_Label,   NULL, NULL},
    {"RSSI",               OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_RSSI_VALUE]},
#ifdef USE_RX_RSSI_DBM
    {"RSSI DBM",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_RSSI_DBM_VALUE]},
#endif
#ifdef USE_RX_RSNR
    {"RSNR",               OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_RSNR_VALUE]},
#endif
    {"BATTERY VOLTAGE",    OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_MAIN_BATT_VOLTAGE]},
    {"BATTERY USAGE",      OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_MAIN_BATT_USAGE]},
    {"AVG CELL VOLTAGE",   OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_AVG_CELL_VOLTAGE]},
#ifdef USE_GPS
    {"BATTERY EFFICIENCY", OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_EFFICIENCY]},
#endif // GPS
    {"CROSSHAIRS",         OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_CROSSHAIRS]},
    {"HORIZON",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ARTIFICIAL_HORIZON]},
    {"HORIZON SIDEBARS",   OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_HORIZON_SIDEBARS]},
    {"UP/DOWN REFERENCE",  OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_UP_DOWN_REFERENCE]},
    {"TIMER 1",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ITEM_TIMER_1]},
    {"TIMER 2",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ITEM_TIMER_2]},
    {"REMAINING TIME ESTIMATE",       OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_REMAINING_TIME_ESTIMATE]},
#ifdef USE_RTC_TIME
    {"RTC DATETIME",       OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_RTC_DATETIME]},
#endif
#ifdef USE_OSD_ADJUSTMENTS
    {"ADJUSTMENT RANGE",   OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ADJUSTMENT_RANGE]},
#endif
#ifdef USE_ADC_INTERNAL
    {"CORE TEMPERATURE",   OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_CORE_TEMPERATURE]},
#endif
    {"ANTI GRAVITY",       OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ANTI_GRAVITY]},
    {"FLY MODE",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_FLYMODE]},
    {"CRAFT NAME",         OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_CRAFT_NAME]},
    {"THROTTLE",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_THROTTLE_POS]},
#ifdef USE_VTX_CONTROL
    {"VTX CHAN",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_VTX_CHANNEL]},
#endif // VTX
    {"CURRENT (A)",        OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_CURRENT_DRAW]},
    {"USED MAH",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_MAH_DRAWN]},
#ifdef USE_GPS
    {"GPS SPEED",          OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_GPS_SPEED]},
    {"GPS SATS",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_GPS_SATS]},
    {"GPS LAT",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_GPS_LAT]},
    {"GPS LON",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_GPS_LON]},
    {"HOME DIR",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_HOME_DIR]},
    {"HOME DIST",          OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_HOME_DIST]},
    {"FLIGHT DIST",        OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_FLIGHT_DIST]},
#endif // GPS
    {"COMPASS BAR",        OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_COMPASS_BAR]},
#ifdef USE_ESC_SENSOR
    {"ESC TEMPERATURE",    OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ESC_TMP]},
    {"ESC RPM",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ESC_RPM]},
#endif
    {"ALTITUDE",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ALTITUDE]},
    {"POWER",              OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_POWER]},
    {"ROLL PID",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ROLL_PIDS]},
    {"PITCH PID",          OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_PITCH_PIDS]},
    {"YAW PID",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_YAW_PIDS]},
    {"PROFILES",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_PIDRATE_PROFILE]},
#ifdef USE_PROFILE_NAMES
    {"PID PROFILE NAME",   OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_PID_PROFILE_NAME]},
    {"RATE PROFILE NAME",  OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_RATE_PROFILE_NAME]},
#endif
#ifdef USE_OSD_PROFILES
    {"OSD PROFILE NAME",   OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_PROFILE_NAME]},
#endif
    {"DEBUG",              OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_DEBUG]},
    {"WARNINGS",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_WARNINGS]},
    {"DISARMED",           OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_DISARMED]},
    {"PIT ANG",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_PITCH_ANGLE]},
    {"ROL ANG",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_ROLL_ANGLE]},
    {"HEADING",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_NUMERICAL_HEADING]},
#ifdef USE_VARIO
    {"VARIO",              OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_NUMERICAL_VARIO]},
#endif
    {"G-FORCE",            OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_G_FORCE]},
    {"MOTOR DIAGNOSTIC",   OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_MOTOR_DIAG]},
#ifdef USE_BLACKBOX
    {"LOG STATUS",         OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_LOG_STATUS]},
#endif
    {"FLIP ARROW",         OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_FLIP_ARROW]},
#ifdef USE_RX_LINK_QUALITY_INFO
    {"LINK QUALITY",       OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_LINK_QUALITY]},
#endif
#ifdef USE_OSD_STICK_OVERLAY
    {"STICK OVERLAY LEFT", OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_STICK_OVERLAY_LEFT]},
    {"STICK OVERLAY RIGHT",OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_STICK_OVERLAY_RIGHT]},
#endif
    {"PILOT NAME",         OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_PILOT_NAME]},
    {"RC CHANNELS",        OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_RC_CHANNELS]},
    {"CAMERA FRAME",       OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_CAMERA_FRAME]},
    {"TOTAL FLIGHTS",      OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_TOTAL_FLIGHTS]},
    {"AUX VALUE",          OME_VISIBLE | DYNAMIC, NULL, &osdConfig_item_pos[OSD_AUX_VALUE]},
    {"BACK",               OME_Back,    NULL, NULL},
    {NULL,                 OME_END,     NULL, NULL}
};

static CMS_Menu menuOsdActiveElems = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSDACT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = menuOsdActiveElemsOnEnter,
    .onExit = menuOsdActiveElemsOnExit,
    .onDisplayUpdate = NULL,
    .entries = menuOsdActiveElemsEntries
};

static uint8_t osdConfig_rssi_alarm;
static uint16_t osdConfig_link_quality_alarm;
static int16_t osdConfig_rssi_dbm_alarm;
static int16_t osdConfig_rsnr_alarm;
static uint16_t osdConfig_cap_alarm;
static uint16_t osdConfig_alt_alarm;
static uint16_t osdConfig_distance_alarm;
static uint8_t batteryConfig_vbatDurationForWarning;
static uint8_t batteryConfig_vbatDurationForCritical;

static const void *menuAlarmsOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    osdConfig_rssi_alarm = osdConfig()->rssi_alarm;
    osdConfig_link_quality_alarm = osdConfig()->link_quality_alarm;
    osdConfig_rssi_dbm_alarm = osdConfig()->rssi_dbm_alarm;
    osdConfig_rsnr_alarm = osdConfig()->rsnr_alarm;
    osdConfig_cap_alarm = osdConfig()->cap_alarm;
    osdConfig_alt_alarm = osdConfig()->alt_alarm;
    osdConfig_distance_alarm = osdConfig()->distance_alarm;
    batteryConfig_vbatDurationForWarning = batteryConfig()->vbatDurationForWarning;
    batteryConfig_vbatDurationForCritical = batteryConfig()->vbatDurationForCritical;

    return NULL;
}

static const void *menuAlarmsOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    osdConfigMutable()->rssi_alarm = osdConfig_rssi_alarm;
    osdConfigMutable()->link_quality_alarm = osdConfig_link_quality_alarm;
    osdConfigMutable()->rssi_dbm_alarm = osdConfig_rssi_dbm_alarm;
    osdConfigMutable()->rsnr_alarm = osdConfig_rsnr_alarm;
    osdConfigMutable()->cap_alarm = osdConfig_cap_alarm;
    osdConfigMutable()->alt_alarm = osdConfig_alt_alarm;
    osdConfigMutable()->distance_alarm = osdConfig_distance_alarm;
    batteryConfigMutable()->vbatDurationForWarning = batteryConfig_vbatDurationForWarning;
    batteryConfigMutable()->vbatDurationForCritical = batteryConfig_vbatDurationForCritical;

    return NULL;
}

const OSD_Entry menuAlarmsEntries[] =
{
    {"--- ALARMS ---", OME_Label, NULL, NULL},
    {"RSSI",     OME_UINT8,  NULL, &(OSD_UINT8_t){&osdConfig_rssi_alarm, 5, 90, 5}},
    {"LINK QUALITY", OME_UINT16,  NULL, &(OSD_UINT16_t){&osdConfig_link_quality_alarm, 5, 300, 5}},
    {"RSSI DBM", OME_INT16,  NULL, &(OSD_INT16_t){&osdConfig_rssi_dbm_alarm, CRSF_RSSI_MIN, CRSF_SNR_MAX, 5}},
    {"RSNR", OME_INT16,  NULL, &(OSD_INT16_t){&osdConfig_rsnr_alarm, CRSF_SNR_MIN, CRSF_SNR_MAX, 5}},
    {"MAIN BAT", OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_cap_alarm, 50, 30000, 50}},
    {"MAX ALT",  OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_alt_alarm, 1, 200, 1}},
    {"MAX DISTANCE", OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_distance_alarm, 0, UINT16_MAX, 10}},
    {"VBAT WARN DUR", OME_UINT8, NULL, &(OSD_UINT8_t){ &batteryConfig_vbatDurationForWarning, 0, 200, 1 } },
    {"VBAT CRIT DUR", OME_UINT8, NULL, &(OSD_UINT8_t){ &batteryConfig_vbatDurationForCritical, 0, 200, 1 } },
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL,}
};

static CMS_Menu menuAlarms = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUALARMS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = menuAlarmsOnEnter,
    .onExit = menuAlarmsOnExit,
    .onDisplayUpdate = NULL,
    .entries = menuAlarmsEntries,
};

osd_timer_source_e timerSource[OSD_TIMER_COUNT];
osd_timer_precision_e timerPrecision[OSD_TIMER_COUNT];
uint8_t timerAlarm[OSD_TIMER_COUNT];

static const void *menuTimersOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        const uint16_t timer = osdConfig()->timers[i];
        timerSource[i] = OSD_TIMER_SRC(timer);
        timerPrecision[i] = OSD_TIMER_PRECISION(timer);
        timerAlarm[i] = OSD_TIMER_ALARM(timer);
    }

    return NULL;
}

static const void *menuTimersOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        osdConfigMutable()->timers[i] = OSD_TIMER(timerSource[i], timerPrecision[i], timerAlarm[i]);
    }

    return NULL;
}

static const char * osdTimerPrecisionNames[] = {"SCND", "HDTH"};

const OSD_Entry menuTimersEntries[] =
{
    {"--- TIMERS ---", OME_Label, NULL, NULL},
    {"1 SRC",          OME_TAB,   NULL, &(OSD_TAB_t){&timerSource[OSD_TIMER_1], OSD_TIMER_SRC_COUNT - 1, osdTimerSourceNames} },
    {"1 PREC",         OME_TAB,   NULL, &(OSD_TAB_t){&timerPrecision[OSD_TIMER_1], OSD_TIMER_PREC_COUNT - 1, osdTimerPrecisionNames}},
    {"1 ALARM",        OME_UINT8, NULL, &(OSD_UINT8_t){&timerAlarm[OSD_TIMER_1], 0, 0xFF, 1}},
    {"2 SRC",          OME_TAB,   NULL, &(OSD_TAB_t){&timerSource[OSD_TIMER_2], OSD_TIMER_SRC_COUNT - 1, osdTimerSourceNames} },
    {"2 PREC",         OME_TAB,   NULL, &(OSD_TAB_t){&timerPrecision[OSD_TIMER_2], OSD_TIMER_PREC_COUNT - 1, osdTimerPrecisionNames}},
    {"2 ALARM",        OME_UINT8, NULL, &(OSD_UINT8_t){&timerAlarm[OSD_TIMER_2], 0, 0xFF, 1}},
    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

static CMS_Menu menuTimers = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUTIMERS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = menuTimersOnEnter,
    .onExit = menuTimersOnExit,
    .onDisplayUpdate = NULL,
    .entries = menuTimersEntries,
};
#endif /* USE_EXTENDED_CMS_MENUS */

#ifdef USE_MAX7456
static bool displayPortProfileMax7456_invert;
static uint8_t displayPortProfileMax7456_blackBrightness;
static uint8_t displayPortProfileMax7456_whiteBrightness;
#endif

#ifdef USE_OSD_PROFILES
static uint8_t osdConfig_osdProfileIndex;
#endif

static displayPortBackground_e osdMenuBackgroundType;

static const void *cmsx_menuOsdOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

#ifdef USE_OSD_PROFILES
    osdConfig_osdProfileIndex = osdConfig()->osdProfileIndex;
#endif

#ifdef USE_MAX7456
    displayPortProfileMax7456_invert = displayPortProfileMax7456()->invert;
    displayPortProfileMax7456_blackBrightness = displayPortProfileMax7456()->blackBrightness;
    displayPortProfileMax7456_whiteBrightness = displayPortProfileMax7456()->whiteBrightness;
    osdMenuBackgroundType = osdConfig()->cms_background_type;
#endif

    return NULL;
}

static const void *cmsx_menuOsdOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

#ifdef USE_OSD_PROFILES
    changeOsdProfileIndex(osdConfig_osdProfileIndex);
#endif

    return NULL;
}

#ifdef USE_MAX7456
static const void *cmsx_max7456Update(displayPort_t *pDisp, const void *self)
{
    UNUSED(self);

    displayPortProfileMax7456Mutable()->invert = displayPortProfileMax7456_invert;
    displayPortProfileMax7456Mutable()->blackBrightness = displayPortProfileMax7456_blackBrightness;
    displayPortProfileMax7456Mutable()->whiteBrightness = displayPortProfileMax7456_whiteBrightness;

    displayClearScreen(pDisp, DISPLAY_CLEAR_WAIT);

    return NULL;
}
#endif // USE_MAX7456

static const void *cmsx_osdBackgroundUpdate(displayPort_t *pDisp, const void *self)
{
    UNUSED(self);
    osdConfigMutable()->cms_background_type = osdMenuBackgroundType;
    displaySetBackgroundType(pDisp, osdMenuBackgroundType);
    return NULL;
}

const OSD_Entry cmsx_menuOsdEntries[] =
{
    {"---OSD---",   OME_Label,   NULL,          NULL},
#ifdef USE_OSD_PROFILES
    {"OSD PROFILE", OME_UINT8, NULL, &(OSD_UINT8_t){&osdConfig_osdProfileIndex, 1, 3, 1}},
#endif
#ifdef USE_EXTENDED_CMS_MENUS
    {"ACTIVE ELEM", OME_Submenu, cmsMenuChange, &menuOsdActiveElems},
    {"TIMERS",      OME_Submenu, cmsMenuChange, &menuTimers},
    {"ALARMS",      OME_Submenu, cmsMenuChange, &menuAlarms},
#endif
#ifdef USE_MAX7456
    {"INVERT",    OME_Bool,  cmsx_max7456Update, &displayPortProfileMax7456_invert},
    {"BRT BLACK", OME_UINT8, cmsx_max7456Update, &(OSD_UINT8_t){&displayPortProfileMax7456_blackBrightness, 0, 3, 1}},
    {"BRT WHITE", OME_UINT8, cmsx_max7456Update, &(OSD_UINT8_t){&displayPortProfileMax7456_whiteBrightness, 0, 3, 1}},
#endif
    {"BACKGROUND",OME_TAB,   cmsx_osdBackgroundUpdate, &(OSD_TAB_t){&osdMenuBackgroundType, DISPLAY_BACKGROUND_COUNT - 1, lookupTableCMSMenuBackgroundType}},
    {"BACK", OME_Back, NULL, NULL},
    {NULL,   OME_END,  NULL, NULL}
};

CMS_Menu cmsx_menuOsd = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUOSD",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuOsdOnEnter,
    .onExit = cmsx_menuOsdOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuOsdEntries
};
#endif // CMS
