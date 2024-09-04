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

#include "config/config.h"

#include "flight/failsafe.h"

#include "rx/rx.h"

uint8_t failsafeConfig_failsafe_procedure;
uint8_t failsafeConfig_failsafe_delay;
uint8_t failsafeConfig_failsafe_landing_time;
uint16_t failsafeConfig_failsafe_throttle;

static const void *cmsx_Failsafe_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    failsafeConfig_failsafe_procedure = failsafeConfig()->failsafe_procedure;
    failsafeConfig_failsafe_delay = failsafeConfig()->failsafe_delay;
    failsafeConfig_failsafe_landing_time = failsafeConfig()->failsafe_landing_time;
    failsafeConfig_failsafe_throttle = failsafeConfig()->failsafe_throttle;

    return NULL;
}

static const void *cmsx_Failsafe_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    failsafeConfigMutable()->failsafe_procedure = failsafeConfig_failsafe_procedure;
    failsafeConfigMutable()->failsafe_delay = failsafeConfig_failsafe_delay;
    failsafeConfigMutable()->failsafe_landing_time = failsafeConfig_failsafe_landing_time;
    failsafeConfigMutable()->failsafe_throttle = failsafeConfig_failsafe_throttle;

    return NULL;
}

static const OSD_Entry cmsx_menuFailsafeEntries[] =
{
    { "-- FAILSAFE --", OME_Label, NULL, NULL},

    { "PROCEDURE",        OME_TAB | REBOOT_REQUIRED,    NULL, &(OSD_TAB_t)    { &failsafeConfig_failsafe_procedure, FAILSAFE_PROCEDURE_COUNT - 1, failsafeProcedureNames } },
    { "GUARD TIME",       OME_FLOAT | REBOOT_REQUIRED,  NULL, &(OSD_FLOAT_t)  { &failsafeConfig_failsafe_delay, PERIOD_RXDATA_RECOVERY / MILLIS_PER_TENTH_SECOND, 200, 1, 100 } },
    { "LANDING_TIME",     OME_FLOAT | REBOOT_REQUIRED,  NULL, &(OSD_FLOAT_t)  { &failsafeConfig_failsafe_landing_time, 0, 200, 1, 100 } },
    { "STAGE 2 THROTTLE", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t) { &failsafeConfig_failsafe_throttle, PWM_PULSE_MIN, PWM_PULSE_MAX, 1 } },
#ifdef USE_CMS_GPS_RESCUE_MENU
    { "GPS RESCUE",       OME_Submenu, cmsMenuChange, &cmsx_menuGpsRescue},
#endif
    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuFailsafe = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUFS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Failsafe_onEnter,
    .onExit = cmsx_Failsafe_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFailsafeEntries
};

#endif
