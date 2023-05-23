/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS_GPS_LAP_TIMER_MENU

#include "cli/settings.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "config/config.h"

#include "fc/gps_lap_timer.h"

static uint16_t gpsLapTimerConfig_minimumLapTimeSeconds;
static uint8_t gpsLapTimerConfig_gateToleranceM;

static const void *cms_menuGpsLapTimerOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    gpsLapTimerConfig_minimumLapTimeSeconds = gpsLapTimerConfig()->minimumLapTimeSeconds;
    gpsLapTimerConfig_gateToleranceM = gpsLapTimerConfig()->gateToleranceM;

    return NULL;
}

static const void *cms_menuGpsLapTimerOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gpsLapTimerConfigMutable()->minimumLapTimeSeconds = gpsLapTimerConfig_minimumLapTimeSeconds;
    gpsLapTimerConfigMutable()->gateToleranceM = gpsLapTimerConfig_gateToleranceM;

    return NULL;
}

static const void *cmsStartSetGate(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    gpsLapTimerStartSetGate();

    return NULL;
}

static const void *cmsEndSetGate(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gpsLapTimerEndSetGate();

    return NULL;
}

static OSD_UINT16_t setGateCMSWaitCount = { &gpsLapTimerData.numberOfSetReadings, 0, 2000, 0 };
static OSD_UINT32_t setGateCMSdistCM = { &gpsLapTimerData.distToPointCM, 0, 200, 0 };

static const OSD_Entry cmsSetGateMenuEntries[] = {
    {"SETTING POSITION", OME_Label,     NULL, NULL},

    {"READINGS",  OME_UINT16 | DYNAMIC, NULL, &setGateCMSWaitCount},
    {"DISTANCE(CM)",  OME_UINT32 | DYNAMIC, NULL, &setGateCMSdistCM},

    {"BACK",       OME_Back,            NULL, NULL},
    { NULL,        OME_END,             NULL, NULL}
};

static CMS_Menu cmsSetGateMenu = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "GATESET",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsStartSetGate,
    .onExit = cmsEndSetGate,
    .onDisplayUpdate = NULL,
    .entries = cmsSetGateMenuEntries,
};

const OSD_Entry cms_menuGpsLapTimerEntries[] =
{
    {"--- GPS LAP TIMER ---", OME_Label,   NULL,          NULL},

    {"SET POSITION",          OME_Funcall, cmsMenuChange, &cmsSetGateMenu},
    {"GATE RADIUS(M)",        OME_UINT8,   NULL,          &(OSD_UINT8_t){&gpsLapTimerConfig_gateToleranceM, 1, 100, 1}},
    {"MIN LAP SECONDS",       OME_UINT16,  NULL,          &(OSD_UINT16_t){&gpsLapTimerConfig_minimumLapTimeSeconds,   0, 3000, 1}},

    {"BACK",                  OME_Back,    NULL,          NULL},
    {NULL,                    OME_END,     NULL,          NULL}};

CMS_Menu cms_menuGpsLapTimer = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUGPSLAPTIMER",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cms_menuGpsLapTimerOnEnter,
    .onExit = cms_menuGpsLapTimerOnExit,
    .onDisplayUpdate = NULL,
    .entries = cms_menuGpsLapTimerEntries,
};

#endif // CMS_GPS_LAP_TIMER_MENU
