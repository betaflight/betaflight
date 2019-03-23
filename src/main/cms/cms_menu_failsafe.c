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

#ifdef USE_CMS_FAILSAFE_MENU

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_failsafe.h"

#ifdef USE_CMS_GPS_RESCUE_MENU
#include "cms/cms_menu_gps_rescue.h"
#endif

#include "config/feature.h"

#include "fc/config.h"

#include "flight/failsafe.h"

#include "rx/rx.h"

uint8_t failsafeConfig_failsafe_procedure;
uint8_t failsafeConfig_failsafe_delay;
uint8_t failsafeConfig_failsafe_off_delay;
uint16_t failsafeConfig_failsafe_throttle;

static long cmsx_Failsafe_onEnter(void)
{
    failsafeConfig_failsafe_procedure = failsafeConfig()->failsafe_procedure;
    failsafeConfig_failsafe_delay = failsafeConfig()->failsafe_delay;
    failsafeConfig_failsafe_off_delay = failsafeConfig()->failsafe_off_delay;
    failsafeConfig_failsafe_throttle = failsafeConfig()->failsafe_throttle;
    return 0;
}

static long cmsx_Failsafe_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    failsafeConfigMutable()->failsafe_procedure = failsafeConfig_failsafe_procedure;
    failsafeConfigMutable()->failsafe_delay = failsafeConfig_failsafe_delay;
    failsafeConfigMutable()->failsafe_off_delay = failsafeConfig_failsafe_off_delay;
    failsafeConfigMutable()->failsafe_throttle = failsafeConfig_failsafe_throttle;

    return 0;
}

static const OSD_Entry cmsx_menuFailsafeEntries[] =
{
    { "-- FAILSAFE --", OME_Label, NULL, NULL, 0},

    { "PROCEDURE",        OME_TAB,    NULL, &(OSD_TAB_t)    { &failsafeConfig_failsafe_procedure, FAILSAFE_PROCEDURE_COUNT - 1, failsafeProcedureNames }, 0 },
    { "GUARD TIME",       OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &failsafeConfig_failsafe_delay, 0, 200, 1, 100 }, 0 },
    { "STAGE 2 DELAY",    OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &failsafeConfig_failsafe_off_delay, 0, 200, 1, 100 }, 0 },
    { "STAGE 2 THROTTLE", OME_UINT16, NULL, &(OSD_UINT16_t) { &failsafeConfig_failsafe_throttle, PWM_PULSE_MIN, PWM_PULSE_MAX, 1 }, 0 },
#ifdef USE_CMS_GPS_RESCUE_MENU
    { "GPS RESCUE",       OME_Submenu, cmsMenuChange, &cmsx_menuGpsRescue, 0},
#endif
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuFailsafe = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUFS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Failsafe_onEnter,
    .onExit = cmsx_Failsafe_onExit,
    .entries = cmsx_menuFailsafeEntries
};

#endif
