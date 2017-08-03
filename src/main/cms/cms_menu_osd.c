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

#include "platform.h"

#if defined(OSD) && defined(CMS)

#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_osd.h"

#include "io/osd.h"


static uint8_t osdConfig_rssi_alarm;
static uint16_t osdConfig_cap_alarm;
static uint16_t osdConfig_time_alarm;
static uint16_t osdConfig_alt_alarm;

static long cmsx_menuAlarmsOnEnter(void)
{
    osdConfig_rssi_alarm = osdConfig()->rssi_alarm;
    osdConfig_cap_alarm = osdConfig()->cap_alarm;
    osdConfig_time_alarm = osdConfig()->time_alarm;
    osdConfig_alt_alarm = osdConfig()->alt_alarm;
    return 0;
}

static long cmsx_menuAlarmsOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    osdConfigMutable()->rssi_alarm = osdConfig_rssi_alarm;
    osdConfigMutable()->cap_alarm = osdConfig_cap_alarm;
    osdConfigMutable()->time_alarm = osdConfig_time_alarm;
    osdConfigMutable()->alt_alarm = osdConfig_alt_alarm;
    return 0;
}

OSD_Entry cmsx_menuAlarmsEntries[] =
{
    {"--- ALARMS ---", OME_Label, NULL, NULL, 0},
    {"RSSI",     OME_UINT8,  NULL, &(OSD_UINT8_t){&osdConfig_rssi_alarm, 5, 90, 5}, 0},
    {"MAIN BAT", OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_cap_alarm, 50, 30000, 50}, 0},
    {"FLY TIME", OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_time_alarm, 1, 200, 1}, 0},
    {"MAX ALT",  OME_UINT16, NULL, &(OSD_UINT16_t){&osdConfig_alt_alarm, 1, 200, 1}, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuAlarms = {
    .GUARD_text = "MENUALARMS",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_menuAlarmsOnEnter,
    .onExit = cmsx_menuAlarmsOnExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuAlarmsEntries,
};

static uint16_t osdConfig_item_pos[OSD_ITEM_COUNT];

static long menuOsdActiveElemsOnEnter(void)
{
    memcpy(&osdConfig_item_pos[0], &osdConfig()->item_pos[0], sizeof(uint16_t) * OSD_ITEM_COUNT);
    return 0;
}

static long menuOsdActiveElemsOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    memcpy(&osdConfigMutable()->item_pos[0], &osdConfig_item_pos[0], sizeof(uint16_t) * OSD_ITEM_COUNT);
    return 0;
}

OSD_Entry menuOsdActiveElemsEntries[] =
{
    {"--- ACTIV ELEM ---", OME_Label, NULL, NULL, 0},
    {"RSSI", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_RSSI_VALUE], 0},
    {"MAIN BATTERY", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_MAIN_BATT_VOLTAGE], 0},
    {"HORIZON", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_ARTIFICIAL_HORIZON], 0},
    {"HORIZON SIDEBARS", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_HORIZON_SIDEBARS], 0},
    {"UPTIME", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_ONTIME], 0},
    {"FLY TIME", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_FLYTIME], 0},
    {"FLY MODE", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_FLYMODE], 0},
    {"NAME", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_CRAFT_NAME], 0},
    {"THROTTLE", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_THROTTLE_POS], 0},
#ifdef VTX
    {"VTX CHAN", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_VTX_CHANNEL]},
#endif // VTX
    {"CURRENT (A)", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_CURRENT_DRAW], 0},
    {"USED MAH", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_MAH_DRAWN], 0},
#ifdef GPS
    {"HOME DIR.", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_HOME_DIR], 0},
    {"HOME DIST.", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_HOME_DIST], 0},
    {"GPS SPEED", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_GPS_SPEED], 0},
    {"GPS SATS.", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_GPS_SATS], 0},
    {"GPS LAT", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_GPS_LAT], 0},
    {"GPS LON.", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_GPS_LON], 0},
    {"HEADING", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_HEADING], 0},
#endif // GPS
#if defined(BARO) || defined(GPS)
    {"VARIO", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_VARIO], 0},
    {"VARIO NUM", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_VARIO_NUM], 0},
#endif // defined
    {"ALTITUDE", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_ALTITUDE], 0},
    {"AIR SPEED", OME_VISIBLE, NULL, &osdConfig_item_pos[OSD_AIR_SPEED], 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu menuOsdActiveElems = {
    .GUARD_text = "MENUOSDACT",
    .GUARD_type = OME_MENU,
    .onEnter = menuOsdActiveElemsOnEnter,
    .onExit = menuOsdActiveElemsOnExit,
    .onGlobalExit = NULL,
    .entries = menuOsdActiveElemsEntries
};

OSD_Entry cmsx_menuOsdLayoutEntries[] =
{
    {"---SCREEN LAYOUT---", OME_Label, NULL, NULL, 0},
    {"ACTIVE ELEM", OME_Submenu, cmsMenuChange, &menuOsdActiveElems, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuOsdLayout = {
    .GUARD_text = "MENULAYOUT",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuOsdLayoutEntries
};
#endif // CMS
