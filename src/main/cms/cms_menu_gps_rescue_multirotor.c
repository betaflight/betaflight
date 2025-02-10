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

#ifndef USE_WING

#ifdef USE_CMS_GPS_RESCUE_MENU

#include "cli/settings.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_gps_rescue.h"

#include "config/feature.h"

#include "config/config.h"

#include "flight/position.h"

#include "pg/autopilot.h"
#include "pg/gps_rescue.h"

static uint16_t gpsRescueConfig_minStartDistM; //meters
static uint8_t gpsRescueConfig_altitudeMode;
static uint16_t gpsRescueConfig_initialClimbM; // meters
static uint16_t gpsRescueConfig_ascendRate;

static uint16_t gpsRescueConfig_returnAltitudeM; //meters
static uint16_t gpsRescueConfig_groundSpeedCmS; // centimeters per second
static uint8_t autopilotConfig_maxAngle; //degrees

static uint16_t gpsRescueConfig_descentDistanceM; //meters
static uint16_t gpsRescueConfig_descendRate;
static uint8_t autopilotConfig_landingAltitudeM;

static uint16_t autopilotConfig_throttleMin;
static uint16_t autopilotConfig_throttleMax;
static uint16_t autopilotConfig_hoverThrottle;

static uint8_t gpsRescueConfig_minSats;
static uint8_t gpsRescueConfig_allowArmingWithoutFix;

static uint8_t autopilotConfig_altitudeP, autopilotConfig_altitudeI, autopilotConfig_altitudeD, autopilotConfig_altitudeF;
static uint8_t gpsRescueConfig_yawP;
static uint8_t autopilotConfig_positionP, autopilotConfig_positionI, autopilotConfig_positionD;
static uint8_t autopilotConfig_positionCutoff;

static const void *cms_menuGpsRescuePidOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    autopilotConfig_altitudeP = autopilotConfig()->altitudeP;
    autopilotConfig_altitudeI = autopilotConfig()->altitudeI;
    autopilotConfig_altitudeD = autopilotConfig()->altitudeD;
    autopilotConfig_altitudeF = autopilotConfig()->altitudeF;

    gpsRescueConfig_yawP = gpsRescueConfig()->yawP;

    autopilotConfig_positionP = autopilotConfig()->positionP;
    autopilotConfig_positionI = autopilotConfig()->positionI;
    autopilotConfig_positionD = autopilotConfig()->positionD;

    autopilotConfig_positionCutoff = autopilotConfig()->positionCutoff;

    return NULL;
}

static const void *cms_menuGpsRescuePidOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    autopilotConfigMutable()->altitudeP = autopilotConfig_altitudeP;
    autopilotConfigMutable()->altitudeI = autopilotConfig_altitudeI;
    autopilotConfigMutable()->altitudeD = autopilotConfig_altitudeD;
    autopilotConfigMutable()->altitudeF = autopilotConfig_altitudeF;

    gpsRescueConfigMutable()->yawP = gpsRescueConfig_yawP;

    autopilotConfigMutable()->positionP = autopilotConfig_positionP;
    autopilotConfigMutable()->positionI = autopilotConfig_positionI;
    autopilotConfigMutable()->positionD = autopilotConfig_positionD;

    autopilotConfigMutable()->positionCutoff = autopilotConfig_positionCutoff;

    return NULL;
}

const OSD_Entry cms_menuGpsRescuePidEntries[] =
{
    {"--- GPS RESCUE PID---", OME_Label, NULL, NULL},

    { "ALTITUDE P",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_altitudeP, 0, 200, 1 } },
    { "ALTITUDE I",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_altitudeI, 0, 200, 1 } },
    { "ALTITUDE D",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_altitudeD, 0, 200, 1 } },
    { "ALTITUDE F",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_altitudeF, 0, 200, 1 } },

    { "YAW P",             OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_yawP, 0, 200, 1 } },

    { "POSITION P",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_positionP, 0, 200, 1 } },
    { "POSITION I",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_positionI, 0, 200, 1 } },
    { "POSITION D",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_positionD, 0, 200, 1 } },

    { "SMOOTHING",         OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_positionCutoff, 10, 255, 1 } },

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

    gpsRescueConfig_minStartDistM = gpsRescueConfig()->minStartDistM;
    gpsRescueConfig_altitudeMode = gpsRescueConfig()->altitudeMode;
    gpsRescueConfig_initialClimbM = gpsRescueConfig()->initialClimbM;
    gpsRescueConfig_ascendRate = gpsRescueConfig()->ascendRate;

    gpsRescueConfig_returnAltitudeM = gpsRescueConfig()->returnAltitudeM;
    gpsRescueConfig_groundSpeedCmS = gpsRescueConfig()->groundSpeedCmS;
    autopilotConfig_maxAngle = autopilotConfig()->maxAngle;

    gpsRescueConfig_descentDistanceM = gpsRescueConfig()->descentDistanceM;
    gpsRescueConfig_descendRate = gpsRescueConfig()->descendRate;

    autopilotConfig_landingAltitudeM = autopilotConfig()->landingAltitudeM;
    autopilotConfig_throttleMin = autopilotConfig()->throttleMin;
    autopilotConfig_throttleMax = autopilotConfig()->throttleMax;
    autopilotConfig_hoverThrottle = autopilotConfig()->hoverThrottle;

    gpsRescueConfig_minSats = gpsRescueConfig()->minSats;
    gpsRescueConfig_allowArmingWithoutFix = gpsRescueConfig()->allowArmingWithoutFix;

    return NULL;
}

static const void *cmsx_menuGpsRescueOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gpsRescueConfigMutable()->minStartDistM = gpsRescueConfig_minStartDistM;
    gpsRescueConfigMutable()->altitudeMode = gpsRescueConfig_altitudeMode;
    gpsRescueConfigMutable()->initialClimbM = gpsRescueConfig_initialClimbM;
    gpsRescueConfigMutable()->ascendRate = gpsRescueConfig_ascendRate;

    gpsRescueConfigMutable()->returnAltitudeM = gpsRescueConfig_returnAltitudeM;
    gpsRescueConfigMutable()->groundSpeedCmS = gpsRescueConfig_groundSpeedCmS;
    autopilotConfigMutable()->maxAngle = autopilotConfig_maxAngle;

    gpsRescueConfigMutable()->descentDistanceM = gpsRescueConfig_descentDistanceM;
    gpsRescueConfigMutable()->descendRate = gpsRescueConfig_descendRate;

    autopilotConfigMutable()->landingAltitudeM = autopilotConfig_landingAltitudeM;
    autopilotConfigMutable()->throttleMin = autopilotConfig_throttleMin;
    autopilotConfigMutable()->throttleMax = autopilotConfig_throttleMax;
    autopilotConfigMutable()->hoverThrottle = autopilotConfig_hoverThrottle;

    gpsRescueConfigMutable()->minSats = gpsRescueConfig_minSats;
    gpsRescueConfigMutable()->allowArmingWithoutFix = gpsRescueConfig_allowArmingWithoutFix;

    return NULL;
}

const OSD_Entry cmsx_menuGpsRescueEntries[] =
{
    {"--- GPS RESCUE ---", OME_Label, NULL, NULL},

    { "MIN START DIST  M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_minStartDistM, 5, 30, 1 } },
    { "ALTITUDE MODE"    , OME_TAB    | REBOOT_REQUIRED, NULL, &(OSD_TAB_t)   { &gpsRescueConfig_altitudeMode, 2, lookupTableRescueAltitudeMode} },
    { "INITAL CLIMB    M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_initialClimbM, 0, 100, 1 } },
    { "ASCEND RATE  CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_ascendRate, 50, 2500, 1 } },

    { "RETURN ALT      M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_returnAltitudeM, 5, 1000, 1 } },
    { "RETURN SPEED CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_groundSpeedCmS, 0, 3000, 1 } },
    { "PITCH ANGLE MAX",   OME_UINT8  | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t) { &autopilotConfig_maxAngle, 0, 60, 1 } },

    { "DESCENT DIST    M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_descentDistanceM, 5, 500, 1 } },
    { "DESCENT RATE CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_descendRate, 25, 500, 1 } },
    { "LANDING ALT     M", OME_UINT8  | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t) { &autopilotConfig_landingAltitudeM, 1, 15, 1 } },

    { "THROTTLE MIN",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &autopilotConfig_throttleMin, 1050, 1400, 1 } },
    { "THROTTLE MAX",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &autopilotConfig_throttleMax, 1400, 2000, 1 } },
    { "THROTTLE HOV",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &autopilotConfig_hoverThrottle, 1100, 1700, 1 } },

    { "SATS REQUIRED",     OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_minSats, 5, 50, 1 } },
    { "ARM WITHOUT FIX",   OME_Bool  | REBOOT_REQUIRED,  NULL, &gpsRescueConfig_allowArmingWithoutFix },

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

#endif // !USE_WING
