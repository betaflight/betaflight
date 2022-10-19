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

#ifdef USE_CMS_GPS_RESCUE_MENU

#include "cli/settings.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_gps_rescue.h"

#include "config/feature.h"

#include "config/config.h"

#include "flight/gps_rescue.h"

static uint16_t gpsRescueConfig_minRescueDth; //meters
static uint8_t gpsRescueConfig_altitudeMode;
static uint8_t gpsRescueConfig_rescueAltitudeBufferM; // meters
static uint16_t gpsRescueConfig_ascendRate;

static uint8_t gpsRescueConfig_initialAltitudeM; //meters
static uint16_t gpsRescueConfig_rescueGroundspeed; // centimeters per second
static uint8_t gpsRescueConfig_angle; //degrees

static uint16_t gpsRescueConfig_descentDistanceM; //meters
static uint16_t gpsRescueConfig_descendRate;
static uint8_t gpsRescueConfig_targetLandingAltitudeM;

static uint16_t gpsRescueConfig_throttleMin;
static uint16_t gpsRescueConfig_throttleMax;
static uint16_t gpsRescueConfig_throttleHover;

static uint8_t gpsRescueConfig_minSats;
static uint8_t gpsRescueConfig_allowArmingWithoutFix;

static uint8_t gpsRescueConfig_throttleP, gpsRescueConfig_throttleI, gpsRescueConfig_throttleD;
static uint8_t gpsRescueConfig_yawP;
static uint8_t gpsRescueConfig_velP, gpsRescueConfig_velI, gpsRescueConfig_velD;


static const void *cms_menuGpsRescuePidOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    gpsRescueConfig_throttleP = gpsRescueConfig()->throttleP;
    gpsRescueConfig_throttleI = gpsRescueConfig()->throttleI;
    gpsRescueConfig_throttleD = gpsRescueConfig()->throttleD;

    gpsRescueConfig_yawP = gpsRescueConfig()->yawP;

    gpsRescueConfig_velP = gpsRescueConfig()->velP;
    gpsRescueConfig_velI = gpsRescueConfig()->velI;
    gpsRescueConfig_velD = gpsRescueConfig()->velD;

    return NULL;
}

static const void *cms_menuGpsRescuePidOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gpsRescueConfigMutable()->throttleP = gpsRescueConfig_throttleP;
    gpsRescueConfigMutable()->throttleI = gpsRescueConfig_throttleI;
    gpsRescueConfigMutable()->throttleD = gpsRescueConfig_throttleD;

    gpsRescueConfigMutable()->yawP = gpsRescueConfig_yawP;

    gpsRescueConfigMutable()->velP = gpsRescueConfig_velP;
    gpsRescueConfigMutable()->velI = gpsRescueConfig_velI;
    gpsRescueConfigMutable()->velD = gpsRescueConfig_velD;

    return NULL;
}

const OSD_Entry cms_menuGpsRescuePidEntries[] =
{
    {"--- GPS RESCUE PID---", OME_Label, NULL, NULL},

    { "THROTTLE P",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_throttleP, 0, 255, 1 } },
    { "THROTTLE I",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_throttleI, 0, 255, 1 } },
    { "THROTTLE D",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_throttleD, 0, 255, 1 } },

    { "YAW P",             OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_yawP, 0, 255, 1 } },

    { "VELOCITY P",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_velP, 0, 255, 1 } },
    { "VELOCITY I",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_velI, 0, 255, 1 } },
    { "VELOCITY D",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_velD, 0, 255, 1 } },

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cms_menuGpsRescuePid = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUGPSRPID",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cms_menuGpsRescuePidOnEnter,
    .onExit = cms_menuGpsRescuePidOnExit,
    .onDisplayUpdate = NULL,
    .entries = cms_menuGpsRescuePidEntries,
};

static const void *cmsx_menuGpsRescueOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    gpsRescueConfig_minRescueDth = gpsRescueConfig()->minRescueDth;
    gpsRescueConfig_altitudeMode = gpsRescueConfig()->altitudeMode;
    gpsRescueConfig_rescueAltitudeBufferM = gpsRescueConfig()->rescueAltitudeBufferM;
    gpsRescueConfig_ascendRate = gpsRescueConfig()->ascendRate;

    gpsRescueConfig_initialAltitudeM = gpsRescueConfig()->initialAltitudeM;
    gpsRescueConfig_rescueGroundspeed = gpsRescueConfig()->rescueGroundspeed;
    gpsRescueConfig_angle = gpsRescueConfig()->angle;

    gpsRescueConfig_descentDistanceM = gpsRescueConfig()->descentDistanceM;
    gpsRescueConfig_descendRate = gpsRescueConfig()->descendRate;
    gpsRescueConfig_targetLandingAltitudeM = gpsRescueConfig()->targetLandingAltitudeM;

    gpsRescueConfig_throttleMin = gpsRescueConfig()->throttleMin;
    gpsRescueConfig_throttleMax = gpsRescueConfig()->throttleMax;
    gpsRescueConfig_throttleHover = gpsRescueConfig()->throttleHover;

    gpsRescueConfig_minSats = gpsRescueConfig()->minSats;
    gpsRescueConfig_allowArmingWithoutFix = gpsRescueConfig()->allowArmingWithoutFix;

    return NULL;
}

static const void *cmsx_menuGpsRescueOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gpsRescueConfigMutable()->minRescueDth = gpsRescueConfig_minRescueDth;
    gpsRescueConfigMutable()->altitudeMode = gpsRescueConfig_altitudeMode;
    gpsRescueConfigMutable()->rescueAltitudeBufferM = gpsRescueConfig_rescueAltitudeBufferM;
    gpsRescueConfigMutable()->ascendRate = gpsRescueConfig_ascendRate;

    gpsRescueConfigMutable()->initialAltitudeM = gpsRescueConfig_initialAltitudeM;
    gpsRescueConfigMutable()->rescueGroundspeed = gpsRescueConfig_rescueGroundspeed;
    gpsRescueConfigMutable()->angle = gpsRescueConfig_angle;

    gpsRescueConfigMutable()->descentDistanceM = gpsRescueConfig_descentDistanceM;
    gpsRescueConfigMutable()->descendRate = gpsRescueConfig_descendRate;
    gpsRescueConfigMutable()->targetLandingAltitudeM = gpsRescueConfig_targetLandingAltitudeM;

    gpsRescueConfigMutable()->throttleMin = gpsRescueConfig_throttleMin;
    gpsRescueConfigMutable()->throttleMax = gpsRescueConfig_throttleMax;
    gpsRescueConfigMutable()->throttleHover = gpsRescueConfig_throttleHover;

    gpsRescueConfigMutable()->minSats = gpsRescueConfig_minSats;
    gpsRescueConfigMutable()->allowArmingWithoutFix = gpsRescueConfig_allowArmingWithoutFix;

    return NULL;
}

const OSD_Entry cmsx_menuGpsRescueEntries[] =
{
    {"--- GPS RESCUE ---", OME_Label, NULL, NULL},

    { "MIN START DIST  M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_minRescueDth, 20, 1000, 1 } },
    { "ALTITUDE MODE"    , OME_TAB | REBOOT_REQUIRED, NULL, &(OSD_TAB_t) { &gpsRescueConfig_altitudeMode, 2, lookupTableRescueAltitudeMode} },
    { "INITAL CLIMB    M", OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_rescueAltitudeBufferM, 0, 100, 1 } },
    { "ASCEND RATE  CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_ascendRate, 50, 2500, 1 } },

    { "RETURN ALT      M", OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_initialAltitudeM, 2, 255, 1 } },
    { "RETURN SPEED CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_rescueGroundspeed, 0, 3000, 1 } },
    { "PITCH ANGLE MAX",   OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_angle, 0, 60, 1 } },

    { "DESCENT DIST    M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_descentDistanceM, 5, 500, 1 } },
    { "DESCENT RATE CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_descendRate, 25, 500, 1 } },
    { "LANDING ALT     M", OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_targetLandingAltitudeM, 3, 15, 1 } },

    { "THROTTLE MIN",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_throttleMin, 1000, 2000, 1 } },
    { "THROTTLE MAX",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_throttleMax, 1000, 2000, 1 } },
    { "THROTTLE HOV",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_throttleHover, 1000, 2000, 1 } },

    { "SATS REQUIRED",     OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_minSats, 5, 50, 1 } },
    { "ARM WITHOUT FIX",   OME_Bool | REBOOT_REQUIRED,  NULL, &gpsRescueConfig_allowArmingWithoutFix },

    { "GPS RESCUE PID",    OME_Submenu, cmsMenuChange, &cms_menuGpsRescuePid},

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuGpsRescue = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUGPSRES",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuGpsRescueOnEnter,
    .onExit = cmsx_menuGpsRescueOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuGpsRescueEntries,
};

#endif
