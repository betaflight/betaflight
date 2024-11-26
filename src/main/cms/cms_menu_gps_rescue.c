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

#include "flight/autopilot.h"
#include "flight/gps_rescue.h"
#include "flight/position.h"

static uint16_t gpsRescueConfig_minStartDistM; //meters
static uint8_t gpsRescueConfig_altitudeMode;
static uint16_t gpsRescueConfig_initialClimbM; // meters
static uint16_t gpsRescueConfig_ascendRate;

static uint16_t gpsRescueConfig_returnAltitudeM; //meters
static uint16_t gpsRescueConfig_groundSpeedCmS; // centimeters per second
static uint8_t gpsRescueConfig_angle; //degrees

static uint16_t gpsRescueConfig_descentDistanceM; //meters
static uint16_t gpsRescueConfig_descendRate;
static uint8_t autopilotConfig_landingAltitudeM;

static uint16_t autopilotConfig_throttleMin;
static uint16_t autopilotConfig_throttleMax;
static uint16_t autopilotConfig_hoverThrottle;

static uint8_t gpsRescueConfig_minSats;
static uint8_t gpsRescueConfig_allowArmingWithoutFix;

static uint8_t autopilotConfig_altitude_P, autopilotConfig_altitude_I, autopilotConfig_altitude_D, autopilotConfig_altitude_F;
static uint8_t gpsRescueConfig_yawP;
static uint8_t gpsRescueConfig_velP, gpsRescueConfig_velI, gpsRescueConfig_velD;

static uint8_t gpsRescueConfig_pitchCutoffHz;
static uint8_t gpsRescueConfig_imuYawGain;

static const void *cms_menuGpsRescuePidOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    autopilotConfig_altitude_P = autopilotConfig()->altitude_P;
    autopilotConfig_altitude_I = autopilotConfig()->altitude_I;
    autopilotConfig_altitude_D = autopilotConfig()->altitude_D;
    autopilotConfig_altitude_F = autopilotConfig()->altitude_F;

    gpsRescueConfig_yawP = gpsRescueConfig()->yawP;

    gpsRescueConfig_velP = gpsRescueConfig()->velP;
    gpsRescueConfig_velI = gpsRescueConfig()->velI;
    gpsRescueConfig_velD = gpsRescueConfig()->velD;

    gpsRescueConfig_pitchCutoffHz = gpsRescueConfig()->pitchCutoffHz;
    gpsRescueConfig_imuYawGain = gpsRescueConfig()->imuYawGain;

    return NULL;
}

static const void *cms_menuGpsRescuePidOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    autopilotConfigMutable()->altitude_P = autopilotConfig_altitude_P;
    autopilotConfigMutable()->altitude_I = autopilotConfig_altitude_I;
    autopilotConfigMutable()->altitude_D = autopilotConfig_altitude_D;
    autopilotConfigMutable()->altitude_F = autopilotConfig_altitude_F;

    gpsRescueConfigMutable()->yawP = gpsRescueConfig_yawP;

    gpsRescueConfigMutable()->velP = gpsRescueConfig_velP;
    gpsRescueConfigMutable()->velI = gpsRescueConfig_velI;
    gpsRescueConfigMutable()->velD = gpsRescueConfig_velD;

    gpsRescueConfigMutable()->pitchCutoffHz = gpsRescueConfig_pitchCutoffHz;
    gpsRescueConfigMutable()->imuYawGain = gpsRescueConfig_imuYawGain;

    return NULL;
}

const OSD_Entry cms_menuGpsRescuePidEntries[] =
{
    {"--- GPS RESCUE PID---", OME_Label, NULL, NULL},

    { "ALTITUDE P",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_altitude_P, 0, 200, 1 } },
    { "ALTITUDE I",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_altitude_I, 0, 200, 1 } },
    { "ALTITUDE D",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_altitude_D, 0, 200, 1 } },
    { "ALTITUDE F",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_altitude_F, 0, 200, 1 } },

    { "YAW P",             OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_yawP, 0, 200, 1 } },

    { "VELOCITY P",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_velP, 0, 200, 1 } },
    { "VELOCITY I",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_velI, 0, 200, 1 } },
    { "VELOCITY D",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_velD, 0, 200, 1 } },

    { "SMOOTHING",         OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_pitchCutoffHz, 10, 255, 1 } },
    { "IMU_YAW_GAIN",      OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_imuYawGain, 5, 20, 1 } },

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
    gpsRescueConfig_angle = gpsRescueConfig()->maxRescueAngle;

    gpsRescueConfig_descentDistanceM = gpsRescueConfig()->descentDistanceM;
    gpsRescueConfig_descendRate = gpsRescueConfig()->descendRate;
    autopilotConfig_landingAltitudeM = autopilotConfig()->landing_altitude_m;

    autopilotConfig_throttleMin = autopilotConfig()->throttle_min;
    autopilotConfig_throttleMax = autopilotConfig()->throttle_max;
    autopilotConfig_hoverThrottle = autopilotConfig()->hover_throttle;

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
    gpsRescueConfigMutable()->maxRescueAngle = gpsRescueConfig_angle;

    gpsRescueConfigMutable()->descentDistanceM = gpsRescueConfig_descentDistanceM;
    gpsRescueConfigMutable()->descendRate = gpsRescueConfig_descendRate;
    autopilotConfigMutable()->landing_altitude_m = autopilotConfig_landingAltitudeM;

    autopilotConfigMutable()->throttle_min = autopilotConfig_throttleMin;
    autopilotConfigMutable()->throttle_max = autopilotConfig_throttleMax;
    autopilotConfigMutable()->hover_throttle = autopilotConfig_hoverThrottle;

    gpsRescueConfigMutable()->minSats = gpsRescueConfig_minSats;
    gpsRescueConfigMutable()->allowArmingWithoutFix = gpsRescueConfig_allowArmingWithoutFix;

    return NULL;
}

const OSD_Entry cmsx_menuGpsRescueEntries[] =
{
    {"--- GPS RESCUE ---", OME_Label, NULL, NULL},

    { "MIN START DIST  M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_minStartDistM, 20, 1000, 1 } },
    { "ALTITUDE MODE"    , OME_TAB | REBOOT_REQUIRED, NULL, &(OSD_TAB_t) { &gpsRescueConfig_altitudeMode, 2, lookupTableRescueAltitudeMode} },
    { "INITAL CLIMB    M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_initialClimbM, 0, 100, 1 } },
    { "ASCEND RATE  CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_ascendRate, 50, 2500, 1 } },

    { "RETURN ALT      M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_returnAltitudeM, 2, 255, 1 } },
    { "RETURN SPEED CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_groundSpeedCmS, 0, 3000, 1 } },
    { "PITCH ANGLE MAX",   OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &gpsRescueConfig_angle, 0, 60, 1 } },

    { "DESCENT DIST    M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_descentDistanceM, 5, 500, 1 } },
    { "DESCENT RATE CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &gpsRescueConfig_descendRate, 25, 500, 1 } },
    { "LANDING ALT     M", OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &autopilotConfig_landingAltitudeM, 1, 15, 1 } },

    { "THROTTLE MIN",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &autopilotConfig_throttleMin, 1050, 1400, 1 } },
    { "THROTTLE MAX",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &autopilotConfig_throttleMax, 1400, 2000, 1 } },
    { "THROTTLE HOV",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &autopilotConfig_hoverThrottle, 1100, 1700, 1 } },

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
