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

#ifdef USE_CMS
#ifdef USE_RPM_LIMITER

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_rpm_limit.h"

#include "config/config.h"
#include "pg/stats.h"

#include "flight/mixer.h"

uint16_t rpm_limiter_rpm_limit;
uint16_t motor_kv;
bool rpm_limiter;

static const void *cmsx_RpmLimit_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    rpm_limiter_rpm_limit = mixerConfig()->rpm_limiter_rpm_limit;
    motor_kv = mixerConfig()->motor_kv;
    rpm_limiter = mixerConfig()->rpm_limiter;

    return NULL;
}

static const void *cmsx_RpmLimit_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    mixerConfigMutable()->rpm_limiter_rpm_limit = rpm_limiter_rpm_limit;
    mixerConfigMutable()->motor_kv = motor_kv;
    mixerConfigMutable()->rpm_limiter = rpm_limiter;

    return NULL;
}

static const OSD_Entry cmsx_menuRpmLimitEntries[] =
{
    {"-- RPM LIMIT --", OME_Label, NULL, NULL},
    { "ACTIVE",   OME_Bool | REBOOT_REQUIRED,  NULL, &rpm_limiter },
    {"MAX RPM", OME_UINT16, NULL, &(OSD_UINT16_t){ &rpm_limiter_rpm_limit, 0, UINT16_MAX, 1}},
    {"KV", OME_UINT16, NULL, &(OSD_UINT16_t){ &motor_kv, 0, UINT16_MAX, 1}},

    { "SAVE&REBOOT",     OME_OSD_Exit, cmsMenuExit,   (void *)CMS_POPUP_SAVEREBOOT},
    {"BACK", OME_Back, NULL, NULL},
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuRpmLimit = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "RPMLIMIT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_RpmLimit_onEnter,
    .onExit = cmsx_RpmLimit_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuRpmLimitEntries
};

#endif
#endif // USE_RPM_LIMITER