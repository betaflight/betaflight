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
#include <string.h>
#include <ctype.h>

#include "platform.h"

#if defined(OSD) && defined(CMS)

#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_osd.h"

#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "io/displayport_max7456.h"
#include "io/osd.h"

#ifndef DISABLE_EXTENDED_CMS_OSD_MENU
static uint16_t osdConfig_item[OSD_ITEM_COUNT];

static long menuOsdActiveElemsOnEnter(void)
{
    memcpy(&osdConfig_item[0], &osdConfig()->item[0], sizeof(uint16_t) * OSD_ITEM_COUNT);
    return 0;
}

static long menuOsdActiveElemsOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    memcpy(&osdConfigMutable()->item[0], &osdConfig_item[0], sizeof(uint16_t) * OSD_ITEM_COUNT);
    return 0;
}

OSD_Entry menuOsdActiveElemsEntries[] =
{
    {"--- ACTIV ELEM ---", OME_Label,   NULL, NULL, 0},
    {"RSSI",               OME_VISIBLE, NULL, &osdConfig_item[OSD_RSSI_VALUE], 0},
    {"BATTERY VOLTAGE",    OME_VISIBLE, NULL, &osdConfig_item[OSD_MAIN_BATT_VOLTAGE], 0},
    {"BATTERY USAGE",      OME_VISIBLE, NULL, &osdConfig_item[OSD_MAIN_BATT_USAGE], 0},
    {"AVG CELL VOLTAGE",   OME_VISIBLE, NULL, &osdConfig_item[OSD_AVG_CELL_VOLTAGE], 0},
    {"CROSSHAIRS",         OME_VISIBLE, NULL, &osdConfig_item[OSD_CROSSHAIRS], 0},
    {"HORIZON",            OME_VISIBLE, NULL, &osdConfig_item[OSD_ARTIFICIAL_HORIZON], 0},
    {"HORIZON SIDEBARS",   OME_VISIBLE, NULL, &osdConfig_item[OSD_HORIZON_SIDEBARS], 0},
    {"TIMER 1",            OME_VISIBLE, NULL, &osdConfig_item[OSD_ITEM_TIMER_1], 0},
    {"TIMER 2",            OME_VISIBLE, NULL, &osdConfig_item[OSD_ITEM_TIMER_2], 0},
    {"FLY MODE",           OME_VISIBLE, NULL, &osdConfig_item[OSD_FLYMODE], 0},
    {"NAME",               OME_VISIBLE, NULL, &osdConfig_item[OSD_CRAFT_NAME], 0},
    {"THROTTLE",           OME_VISIBLE, NULL, &osdConfig_item[OSD_THROTTLE_POS], 0},
#ifdef VTX_CONTROL
    {"VTX CHAN",           OME_VISIBLE, NULL, &osdConfig_item[OSD_VTX_CHANNEL], 0},
#endif // VTX
    {"CURRENT (A)",        OME_VISIBLE, NULL, &osdConfig_item[OSD_CURRENT_DRAW], 0},
    {"USED MAH",           OME_VISIBLE, NULL, &osdConfig_item[OSD_MAH_DRAWN], 0},
#ifdef GPS
    {"GPS SPEED",          OME_VISIBLE, NULL, &osdConfig_item[OSD_GPS_SPEED], 0},
    {"GPS SATS",           OME_VISIBLE, NULL, &osdConfig_item[OSD_GPS_SATS], 0},
    {"GPS LAT",            OME_VISIBLE, NULL, &osdConfig_item[OSD_GPS_LAT], 0},
    {"GPS LON",            OME_VISIBLE, NULL, &osdConfig_item[OSD_GPS_LON], 0},
    {"HOME DIR",           OME_VISIBLE, NULL, &osdConfig_item[OSD_HOME_DIR], 0},
    {"HOME DIST",          OME_VISIBLE, NULL, &osdConfig_item[OSD_HOME_DIST], 0},
#endif // GPS
    {"COMPASS BAR",        OME_VISIBLE, NULL, &osdConfig_item[OSD_COMPASS_BAR], 0},
    {"ALTITUDE",           OME_VISIBLE, NULL, &osdConfig_item[OSD_ALTITUDE], 0},
    {"POWER",              OME_VISIBLE, NULL, &osdConfig_item[OSD_POWER], 0},
    {"ROLL PID",           OME_VISIBLE, NULL, &osdConfig_item[OSD_ROLL_PIDS], 0},
    {"PITCH PID",          OME_VISIBLE, NULL, &osdConfig_item[OSD_PITCH_PIDS], 0},
    {"YAW PID",            OME_VISIBLE, NULL, &osdConfig_item[OSD_YAW_PIDS], 0},
    {"PROFILES",           OME_VISIBLE, NULL, &osdConfig_item[OSD_PIDRATE_PROFILE], 0},
    {"DEBUG",              OME_VISIBLE, NULL, &osdConfig_item[OSD_DEBUG], 0},
    {"WARNINGS",           OME_VISIBLE, NULL, &osdConfig_item[OSD_WARNINGS], 0},
    {"DISARMED",           OME_VISIBLE, NULL, &osdConfig_item[OSD_DISARMED], 0},
    {"PIT ANG",            OME_VISIBLE, NULL, &osdConfig_item[OSD_PITCH_ANGLE], 0},
    {"ROL ANG",            OME_VISIBLE, NULL, &osdConfig_item[OSD_ROLL_ANGLE], 0},
    {"HEADING",            OME_VISIBLE, NULL, &osdConfig_item[OSD_NUMERICAL_HEADING], 0},
    {"VARIO",              OME_VISIBLE, NULL, &osdConfig_item[OSD_NUMERICAL_VARIO], 0},
    {"BACK",               OME_Back,    NULL, NULL, 0},
    {NULL,                 OME_END,     NULL, NULL, 0}
};

CMS_Menu menuOsdActiveElems = {
    .GUARD_text = "MENUOSDACT",
    .GUARD_type = OME_MENU,
    .onEnter = menuOsdActiveElemsOnEnter,
    .onExit = menuOsdActiveElemsOnExit,
    .onGlobalExit = NULL,
    .entries = menuOsdActiveElemsEntries
};

static uint8_t osdConfig_rssi_alarm;
static uint16_t osdConfig_cap_alarm;
static uint16_t osdConfig_alt_alarm;

static long menuAlarmsOnEnter(void)
{
    osdConfig_rssi_alarm = osdConfig()->rssi_alarm;
    osdConfig_cap_alarm = osdConfig()->cap_alarm;
    osdConfig_alt_alarm = osdConfig()->alt_alarm;

    return 0;
}

static long menuAlarmsOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    osdConfigMutable()->rssi_alarm = osdConfig_rssi_alarm;
    osdConfigMutable()->cap_alarm = osdConfig_cap_alarm;
    osdConfigMutable()->alt_alarm = osdConfig_alt_alarm;

    return 0;
}

OSD_Entry menuAlarmsEntries[] =
{
    {"--- ALARMS ---", OME_Label, NULL, NULL, 0},
    {"RSSI",     OME_UINT8,  NULL, &(OSD_UINT8_t){&osdConfig_rssi_alarm, 5, 90, 5}, 0},
    {"MAIN BAT", OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_cap_alarm, 50, 30000, 50}, 0},
    {"MAX ALT",  OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_alt_alarm, 1, 200, 1}, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu menuAlarms = {
    .GUARD_text = "MENUALARMS",
    .GUARD_type = OME_MENU,
    .onEnter = menuAlarmsOnEnter,
    .onExit = menuAlarmsOnExit,
    .onGlobalExit = NULL,
    .entries = menuAlarmsEntries,
};

osd_timer_source_e timerSource[OSD_TIMER_COUNT];
osd_timer_precision_e timerPrecision[OSD_TIMER_COUNT];
uint8_t timerAlarm[OSD_TIMER_COUNT];

static long menuTimersOnEnter(void)
{
    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        const uint16_t timer = osdConfig()->timers[i];
        timerSource[i] = OSD_TIMER_SRC(timer);
        timerPrecision[i] = OSD_TIMER_PRECISION(timer);
        timerAlarm[i] = OSD_TIMER_ALARM(timer);
    }

    return 0;
}

static long menuTimersOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    for (int i = 0; i < OSD_TIMER_COUNT; i++) {
        osdConfigMutable()->timers[i] = OSD_TIMER(timerSource[i], timerPrecision[i], timerAlarm[i]);
    }

    return 0;
}

static const char * osdTimerPrecisionNames[] = {"SCND", "HDTH"};

OSD_Entry menuTimersEntries[] =
{
    {"--- TIMERS ---", OME_Label, NULL, NULL, 0},
    {"1 SRC",          OME_TAB,   NULL, &(OSD_TAB_t){&timerSource[OSD_TIMER_1], OSD_TIMER_SRC_COUNT - 1, osdTimerSourceNames}, 0 },
    {"1 PREC",         OME_TAB,   NULL, &(OSD_TAB_t){&timerPrecision[OSD_TIMER_1], OSD_TIMER_PREC_COUNT - 1, osdTimerPrecisionNames}, 0},
    {"1 ALARM",        OME_UINT8, NULL, &(OSD_UINT8_t){&timerAlarm[OSD_TIMER_1], 0, 0xFF, 1}, 0},
    {"2 SRC",          OME_TAB,   NULL, &(OSD_TAB_t){&timerSource[OSD_TIMER_2], OSD_TIMER_SRC_COUNT - 1, osdTimerSourceNames}, 0 },
    {"2 PREC",         OME_TAB,   NULL, &(OSD_TAB_t){&timerPrecision[OSD_TIMER_2], OSD_TIMER_PREC_COUNT - 1, osdTimerPrecisionNames}, 0},
    {"2 ALARM",        OME_UINT8, NULL, &(OSD_UINT8_t){&timerAlarm[OSD_TIMER_2], 0, 0xFF, 1}, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu menuTimers = {
    .GUARD_text = "MENUTIMERS",
    .GUARD_type = OME_MENU,
    .onEnter = menuTimersOnEnter,
    .onExit = menuTimersOnExit,
    .onGlobalExit = NULL,
    .entries = menuTimersEntries,
};
#endif /* DISABLE_EXTENDED_CMS_OSD_MENU */

#ifdef USE_MAX7456
static bool displayPortProfileMax7456_invert;
static uint8_t displayPortProfileMax7456_blackBrightness;
static uint8_t displayPortProfileMax7456_whiteBrightness;
#endif

static long cmsx_menuOsdOnEnter(void)
{
#ifdef USE_MAX7456
    displayPortProfileMax7456_invert = displayPortProfileMax7456()->invert;
    displayPortProfileMax7456_blackBrightness = displayPortProfileMax7456()->blackBrightness;
    displayPortProfileMax7456_whiteBrightness = displayPortProfileMax7456()->whiteBrightness;
#endif

    return 0;
}

static long cmsx_menuOsdOnExit(const OSD_Entry *self)
{
    UNUSED(self);

#ifdef USE_MAX7456
    displayPortProfileMax7456Mutable()->invert = displayPortProfileMax7456_invert;
    displayPortProfileMax7456Mutable()->blackBrightness = displayPortProfileMax7456_blackBrightness;
    displayPortProfileMax7456Mutable()->whiteBrightness = displayPortProfileMax7456_whiteBrightness;
#endif

  return 0;
}

OSD_Entry cmsx_menuOsdEntries[] =
{
    {"---OSD---",   OME_Label,   NULL,          NULL,                0},
#ifndef DISABLE_EXTENDED_CMS_OSD_MENU
    {"ACTIVE ELEM", OME_Submenu, cmsMenuChange, &menuOsdActiveElems, 0},
    {"TIMERS",      OME_Submenu, cmsMenuChange, &menuTimers,         0},
    {"ALARMS",      OME_Submenu, cmsMenuChange, &menuAlarms,         0},
#endif
#ifdef USE_MAX7456
    {"INVERT",    OME_Bool,  NULL, &displayPortProfileMax7456_invert,                                   0},
    {"BRT BLACK", OME_UINT8, NULL, &(OSD_UINT8_t){&displayPortProfileMax7456_blackBrightness, 0, 3, 1}, 0},
    {"BRT WHITE", OME_UINT8, NULL, &(OSD_UINT8_t){&displayPortProfileMax7456_whiteBrightness, 0, 3, 1}, 0},
#endif
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL,   OME_END,  NULL, NULL, 0}
};

CMS_Menu cmsx_menuOsd = {
    .GUARD_text = "MENUOSD",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_menuOsdOnEnter,
    .onExit = cmsx_menuOsdOnExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuOsdEntries
};
#endif // CMS
