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
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_osd.h"

#include "common/axis.h"
#include "io/gimbal.h"
#include "flight/pid.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#if defined(OSD) && defined(CMS)

OSD_UINT8_t entryAlarmRssi = {&osdProfile()->rssi_alarm, 5, 90, 5};
OSD_UINT16_t entryAlarmCapacity = {&osdProfile()->cap_alarm, 50, 30000, 50};
OSD_UINT16_t enryAlarmFlyTime = {&osdProfile()->time_alarm, 1, 200, 1};
OSD_UINT16_t entryAlarmAltitude = {&osdProfile()->alt_alarm, 1, 200, 1};

OSD_Entry cmsx_menuAlarmsEntries[] =
{
    {"--- ALARMS ---", OME_Label, NULL, NULL, 0},
    {"RSSI", OME_UINT8, NULL, &entryAlarmRssi, 0},
    {"MAIN BAT", OME_UINT16, NULL, &entryAlarmCapacity, 0},
    {"FLY TIME", OME_UINT16, NULL, &enryAlarmFlyTime, 0},
    {"MAX ALT", OME_UINT16, NULL, &entryAlarmAltitude, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuAlarms = {
    .GUARD_text = "MENUALARMS",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuAlarmsEntries,
};

OSD_Entry menuOsdActiveElemsEntries[] =
{
    {"--- ACTIV ELEM ---", OME_Label, NULL, NULL, 0},
    {"RSSI", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_RSSI_VALUE], 0},
    {"MAIN BATTERY", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_MAIN_BATT_VOLTAGE], 0},
    {"HORIZON", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_ARTIFICIAL_HORIZON], 0},
    {"HORIZON SIDEBARS", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_HORIZON_SIDEBARS], 0},
    {"UPTIME", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_ONTIME], 0},
    {"FLY TIME", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_FLYTIME], 0},
    {"FLY MODE", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_FLYMODE], 0},
    {"NAME", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_CRAFT_NAME], 0},
    {"THROTTLE", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_THROTTLE_POS], 0},
#ifdef VTX
    {"VTX CHAN", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_VTX_CHANNEL]},
#endif // VTX
    {"CURRENT (A)", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_CURRENT_DRAW], 0},
    {"USED MAH", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_MAH_DRAWN], 0},
#ifdef GPS
    {"GPS SPEED", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_GPS_SPEED], 0},
    {"GPS SATS.", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_GPS_SATS], 0},
#endif // GPS
    {"ALTITUDE", OME_VISIBLE, NULL, &osdProfile()->item_pos[OSD_ALTITUDE], 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu menuOsdActiveElems = {
    .GUARD_text = "MENUOSDACT",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
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
