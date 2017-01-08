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

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef CMS

#include "build/version.h"

#include "drivers/system.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/utils.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

//
// PID
//
static uint8_t tmpProfileIndex;
static uint8_t profileIndex;
static char profileIndexString[] = " p";
static uint8_t tempPid[3][3];

static uint8_t tmpRateProfileIndex;
static uint8_t rateProfileIndex;
static char rateProfileIndexString[] = " p-r";
static controlRateConfig_t rateProfile;

static long cmsx_menuImu_onEnter(void)
{
    profileIndex = masterConfig.current_profile_index;
    tmpProfileIndex = profileIndex + 1;

    rateProfileIndex = masterConfig.profile[profileIndex].activeRateProfile;
    tmpRateProfileIndex = rateProfileIndex + 1;

    return 0;
}

static long cmsx_menuImu_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    masterConfig.current_profile_index = profileIndex;
    masterConfig.profile[profileIndex].activeRateProfile = rateProfileIndex;

    return 0;
}

static long cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    profileIndex = tmpProfileIndex - 1;

    return 0;
}

static long cmsx_rateProfileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    rateProfileIndex = tmpRateProfileIndex - 1;

    return 0;
}

static long cmsx_PidRead(void)
{

    for (uint8_t i = 0; i < 3; i++) {
        tempPid[i][0] = masterConfig.profile[profileIndex].pidProfile.P8[i];
        tempPid[i][1] = masterConfig.profile[profileIndex].pidProfile.I8[i];
        tempPid[i][2] = masterConfig.profile[profileIndex].pidProfile.D8[i];
    }

    return 0;
}

static long cmsx_PidOnEnter(void)
{
    profileIndexString[1] = '0' + tmpProfileIndex;
    cmsx_PidRead();

    return 0;
}

static long cmsx_PidWriteback(const OSD_Entry *self)
{
    UNUSED(self);

    for (uint8_t i = 0; i < 3; i++) {
        masterConfig.profile[profileIndex].pidProfile.P8[i] = tempPid[i][0];
        masterConfig.profile[profileIndex].pidProfile.I8[i] = tempPid[i][1];
        masterConfig.profile[profileIndex].pidProfile.D8[i] = tempPid[i][2];
    }
    pidInitConfig(&currentProfile->pidProfile);

    return 0;
}

static OSD_Entry cmsx_menuPidEntries[] =
{
    { "-- PID --", OME_Label, NULL, profileIndexString, 0},

    { "ROLL  P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PIDROLL][0],  0, 200, 1 }, 0 },
    { "ROLL  I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PIDROLL][1],  0, 200, 1 }, 0 },
    { "ROLL  D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PIDROLL][2],  0, 200, 1 }, 0 },

    { "PITCH P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PIDPITCH][0], 0, 200, 1 }, 0 },
    { "PITCH I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PIDPITCH][1], 0, 200, 1 }, 0 },
    { "PITCH D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PIDPITCH][2], 0, 200, 1 }, 0 },

    { "YAW   P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PIDYAW][0],   0, 200, 1 }, 0 },
    { "YAW   I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PIDYAW][1],   0, 200, 1 }, 0 },
    { "YAW   D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PIDYAW][2],   0, 200, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuPid = {
    .GUARD_text = "XPID",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_PidOnEnter,
    .onExit = cmsx_PidWriteback,
    .onGlobalExit = NULL,
    .entries = cmsx_menuPidEntries
};

//
// Rate & Expo
//

static long cmsx_RateProfileRead(void)
{
    memcpy(&rateProfile, &masterConfig.profile[profileIndex].controlRateProfile[rateProfileIndex], sizeof(controlRateConfig_t));

    return 0;
}

static long cmsx_RateProfileWriteback(const OSD_Entry *self)
{
    UNUSED(self);

    memcpy(&masterConfig.profile[profileIndex].controlRateProfile[rateProfileIndex], &rateProfile, sizeof(controlRateConfig_t));

    return 0;
}

static long cmsx_RateProfileOnEnter(void)
{
    rateProfileIndexString[1] = '0' + tmpProfileIndex;
    rateProfileIndexString[3] = '0' + tmpRateProfileIndex;
    cmsx_RateProfileRead();

    return 0;
}

static OSD_Entry cmsx_menuRateProfileEntries[] =
{
    { "-- RATE --", OME_Label, NULL, rateProfileIndexString, 0 },

    { "RC RATE",     OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRate8,    0, 255, 1, 10 }, 0 },
    { "RC YAW RATE", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcYawRate8, 0, 255, 1, 10 }, 0 },

    { "ROLL SUPER",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[0],   0, 100, 1, 10 }, 0 },
    { "PITCH SUPER", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[1],   0, 100, 1, 10 }, 0 },
    { "YAW SUPER",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[2],   0, 100, 1, 10 }, 0 },

    { "RC EXPO",     OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo8,    0, 100, 1, 10 }, 0 },
    { "RC YAW EXP",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcYawExpo8, 0, 100, 1, 10 }, 0 },

    { "THRPID ATT",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.dynThrPID,  0, 100, 1, 10}, 0 },
    { "TPA BRKPT",   OME_UINT16, NULL, &(OSD_UINT16_t){ &rateProfile.tpa_breakpoint, 1000, 2000, 10}, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuRateProfile = {
    .GUARD_text = "MENURATE",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_RateProfileOnEnter,
    .onExit = cmsx_RateProfileWriteback,
    .onGlobalExit = NULL,
    .entries = cmsx_menuRateProfileEntries
};

static uint8_t cmsx_dtermSetpointWeight;
static uint8_t cmsx_setpointRelaxRatio;
static uint8_t cmsx_angleStrength;
static uint8_t cmsx_horizonStrength;
static uint8_t cmsx_horizonTransition;

static long cmsx_profileOtherOnEnter(void)
{
    profileIndexString[1] = '0' + tmpProfileIndex;

    cmsx_dtermSetpointWeight = masterConfig.profile[profileIndex].pidProfile.dtermSetpointWeight;
    cmsx_setpointRelaxRatio  = masterConfig.profile[profileIndex].pidProfile.setpointRelaxRatio;

    cmsx_angleStrength =     masterConfig.profile[profileIndex].pidProfile.P8[PIDLEVEL];
    cmsx_horizonStrength =   masterConfig.profile[profileIndex].pidProfile.I8[PIDLEVEL];
    cmsx_horizonTransition = masterConfig.profile[profileIndex].pidProfile.D8[PIDLEVEL];

    return 0;
}

static long cmsx_profileOtherOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    masterConfig.profile[profileIndex].pidProfile.dtermSetpointWeight = cmsx_dtermSetpointWeight;
    masterConfig.profile[profileIndex].pidProfile.setpointRelaxRatio = cmsx_setpointRelaxRatio;
    pidInitConfig(&currentProfile->pidProfile);

    masterConfig.profile[profileIndex].pidProfile.P8[PIDLEVEL] = cmsx_angleStrength;
    masterConfig.profile[profileIndex].pidProfile.I8[PIDLEVEL] = cmsx_horizonStrength;
    masterConfig.profile[profileIndex].pidProfile.D8[PIDLEVEL] = cmsx_horizonTransition;

    return 0;
}

static OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, profileIndexString, 0 },

    { "D SETPT WT",  OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_dtermSetpointWeight, 0, 255, 1, 10 }, 0 },
    { "SETPT TRS",   OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_setpointRelaxRatio,  0, 100, 1, 10 }, 0 },
    { "ANGLE STR",   OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_angleStrength,       0, 200, 1 }    , 0 },
    { "HORZN STR",   OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_horizonStrength,     0, 200, 1 }    , 0 },
    { "HORZN TRS",   OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_horizonTransition,   0, 200, 1 }    , 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuProfileOther = {
    .GUARD_text = "XPROFOTHER",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_profileOtherOnEnter,
    .onExit = cmsx_profileOtherOnExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuProfileOtherEntries,
};

static OSD_Entry cmsx_menuFilterGlobalEntries[] =
{
    { "-- FILTER GLB  --", OME_Label, NULL, NULL, 0 },

    { "GYRO LPF",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &gyroConfig()->gyro_soft_lpf_hz,         0, 255, 1 }, 0 },
    { "GYRO NF1",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig()->gyro_soft_notch_hz_1,     0, 500, 1 }, 0 },
    { "GYRO NF1C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig()->gyro_soft_notch_cutoff_1, 0, 500, 1 }, 0 },
    { "GYRO NF2",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig()->gyro_soft_notch_hz_2,     0, 500, 1 }, 0 },
    { "GYRO NF2C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig()->gyro_soft_notch_cutoff_2, 0, 500, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuFilterGlobal = {
    .GUARD_text = "XFLTGLB",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = cmsx_menuFilterGlobalEntries,
};

static uint16_t cmsx_dterm_lpf_hz;
static uint16_t cmsx_dterm_notch_hz;
static uint16_t cmsx_dterm_notch_cutoff;
static uint16_t cmsx_yaw_lpf_hz;
static uint16_t cmsx_yaw_p_limit;

static long cmsx_FilterPerProfileRead(void)
{
    cmsx_dterm_lpf_hz =       masterConfig.profile[profileIndex].pidProfile.dterm_lpf_hz;
    cmsx_dterm_notch_hz =     masterConfig.profile[profileIndex].pidProfile.dterm_notch_hz;
    cmsx_dterm_notch_cutoff = masterConfig.profile[profileIndex].pidProfile.dterm_notch_cutoff;
    cmsx_yaw_lpf_hz =         masterConfig.profile[profileIndex].pidProfile.yaw_lpf_hz;
    cmsx_yaw_p_limit =        masterConfig.profile[profileIndex].pidProfile.yaw_p_limit;

    return 0;
}

static long cmsx_FilterPerProfileWriteback(const OSD_Entry *self)
{
    UNUSED(self);

    masterConfig.profile[profileIndex].pidProfile.dterm_lpf_hz =       cmsx_dterm_lpf_hz;
    masterConfig.profile[profileIndex].pidProfile.dterm_notch_hz =     cmsx_dterm_notch_hz;
    masterConfig.profile[profileIndex].pidProfile.dterm_notch_cutoff = cmsx_dterm_notch_cutoff;
    masterConfig.profile[profileIndex].pidProfile.yaw_lpf_hz =         cmsx_yaw_lpf_hz;
    masterConfig.profile[profileIndex].pidProfile.yaw_p_limit =        cmsx_yaw_p_limit;

    return 0;
}

static OSD_Entry cmsx_menuFilterPerProfileEntries[] =
{
    { "-- FILTER PP  --", OME_Label, NULL, NULL, 0 },

    { "DTERM LPF",  OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lpf_hz,         0, 500, 1 }, 0 },
    { "DTERM NF",   OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_notch_hz,       0, 500, 1 }, 0 },
    { "DTERM NFCO", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_notch_cutoff,   0, 500, 1 }, 0 },
    { "YAW LPF",    OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_yaw_lpf_hz,           0, 500, 1 }, 0 },
    { "YAW P LIM",  OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_yaw_p_limit,        100, 500, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuFilterPerProfile = {
    .GUARD_text = "XFLTPP",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_FilterPerProfileRead,
    .onExit = cmsx_FilterPerProfileWriteback,
    .onGlobalExit = NULL,
    .entries = cmsx_menuFilterPerProfileEntries,
};

static OSD_Entry cmsx_menuImuEntries[] =
{
    { "-- IMU --", OME_Label, NULL, NULL, 0},

    {"PID PROF",  OME_UINT8,   cmsx_profileIndexOnChange,     &(OSD_UINT8_t){ &tmpProfileIndex, 1, MAX_PROFILE_COUNT, 1},    0},
    {"PID",       OME_Submenu, cmsMenuChange,                 &cmsx_menuPid,                                                 0},
    {"MISC PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther,                                        0},
    {"FILT PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterPerProfile,                                    0},

    {"RATE PROF", OME_UINT8,   cmsx_rateProfileIndexOnChange, &(OSD_UINT8_t){ &tmpRateProfileIndex, 1, MAX_RATEPROFILES, 1}, 0},
    {"RATE",      OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile,                                         0},

    {"FILT GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal,                                        0},

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuImu = {
    .GUARD_text = "XIMU",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_menuImu_onEnter,
    .onExit = cmsx_menuImu_onExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuImuEntries,
};
#endif // CMS
