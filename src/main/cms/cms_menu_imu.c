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

#include "platform.h"

#ifdef CMS

#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/axis.h"

#include "flight/pid.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"

#include "navigation/navigation.h"

#include "sensors/gyro.h"


//
// PID
//
static uint8_t tmpProfileIndex;
static uint8_t profileIndex;
static char profileIndexString[] = " p";

static void cmsx_ReadPidToArray(uint8_t *dst, int pidIndex)
{
    dst[0] = pidBank()->pid[pidIndex].P;
    dst[1] = pidBank()->pid[pidIndex].I;
    dst[2] = pidBank()->pid[pidIndex].D;
}

static void cmsx_WritebackPidFromArray(uint8_t *src, int pidIndex)
{
    pidBankMutable()->pid[pidIndex].P = src[0];
    pidBankMutable()->pid[pidIndex].I = src[1];
    pidBankMutable()->pid[pidIndex].D = src[2];
}

static long cmsx_menuImu_onEnter(void)
{
    profileIndex = getConfigProfile();
    tmpProfileIndex = profileIndex + 1;
    profileIndexString[1] = '0' + tmpProfileIndex;

    return 0;
}

static long cmsx_menuImu_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    setConfigProfile(profileIndex);

    return 0;
}

static long cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    profileIndex = tmpProfileIndex - 1;
    profileIndexString[1] = '0' + tmpProfileIndex;

    return 0;
}

static uint8_t cmsx_pidRoll[3];
static uint8_t cmsx_pidPitch[3];
static uint8_t cmsx_pidYaw[3];

static long cmsx_PidRead(void)
{
    cmsx_ReadPidToArray(cmsx_pidRoll, PID_ROLL);
    cmsx_ReadPidToArray(cmsx_pidPitch, PID_PITCH);
    cmsx_ReadPidToArray(cmsx_pidYaw, PID_YAW);

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

    cmsx_WritebackPidFromArray(cmsx_pidRoll, PID_ROLL);
    cmsx_WritebackPidFromArray(cmsx_pidPitch, PID_PITCH);
    cmsx_WritebackPidFromArray(cmsx_pidYaw, PID_YAW);

    schedulePidGainsUpdate();

    return 0;
}

static OSD_Entry cmsx_menuPidEntries[] =
{
    { "-- PID --", OME_Label, NULL, profileIndexString, 0},

    { "ROLL  P", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidRoll[0],  0, 200, 1 }, 0 },
    { "ROLL  I", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidRoll[1],  0, 200, 1 }, 0 },
    { "ROLL  D", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidRoll[2],  0, 200, 1 }, 0 },

    { "PITCH P", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidPitch[0], 0, 200, 1 }, 0 },
    { "PITCH I", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidPitch[1], 0, 200, 1 }, 0 },
    { "PITCH D", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidPitch[2], 0, 200, 1 }, 0 },

    { "YAW   P", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidYaw[0],   0, 200, 1 }, 0 },
    { "YAW   I", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidYaw[1],   0, 200, 1 }, 0 },
    { "YAW   D", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidYaw[2],   0, 200, 1 }, 0 },

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

static uint8_t cmsx_pidPosZ[3];
static uint8_t cmsx_pidVelZ[3];
static uint8_t cmsx_pidHead[3];

static long cmsx_menuPidAltMag_onEnter(void)
{
    cmsx_ReadPidToArray(cmsx_pidPosZ, PID_POS_Z);
    cmsx_ReadPidToArray(cmsx_pidVelZ, PID_VEL_Z);
    cmsx_pidHead[0] = pidBank()->pid[PID_HEADING].P;

    return 0;
}

static long cmsx_menuPidAltMag_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_WritebackPidFromArray(cmsx_pidPosZ, PID_POS_Z);
    cmsx_WritebackPidFromArray(cmsx_pidVelZ, PID_VEL_Z);
    pidBankMutable()->pid[PID_HEADING].P = cmsx_pidHead[0];

    navigationUsePIDs();

    return 0;
}

static OSD_Entry cmsx_menuPidAltMagEntries[] = {
    { "-- ALT&MAG --", OME_Label, NULL, profileIndexString, 0},

    { "ALT P", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidPosZ[0], 0, 255, 1 }, 0 },
    { "ALT I", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidPosZ[1], 0, 255, 1 }, 0 },
    { "ALT D", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidPosZ[2], 0, 255, 1 }, 0 },
    { "VEL P", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidVelZ[0], 0, 255, 1 }, 0 },
    { "VEL I", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidVelZ[1], 0, 255, 1 }, 0 },
    { "VEL D", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidVelZ[2], 0, 255, 1 }, 0 },
    { "MAG P", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidHead[0], 0, 255, 1 }, 0 },

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

static CMS_Menu cmsx_menuPidAltMag = {
    .GUARD_text = "XALTMAG",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_menuPidAltMag_onEnter,
    .onExit = cmsx_menuPidAltMag_onExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuPidAltMagEntries,
};

static uint8_t cmsx_pidPosXY[3];
static uint8_t cmsx_pidVelXY[3];

static long cmsx_menuPidGpsnav_onEnter(void)
{
    cmsx_ReadPidToArray(cmsx_pidPosXY, PID_POS_XY);
    cmsx_ReadPidToArray(cmsx_pidVelXY, PID_VEL_XY);

    return 0;
}

static long cmsx_menuPidGpsnav_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_WritebackPidFromArray(cmsx_pidPosXY, PID_POS_XY);
    cmsx_WritebackPidFromArray(cmsx_pidVelXY, PID_VEL_XY);

    navigationUsePIDs();

    return 0;
}

static OSD_Entry cmsx_menuPidGpsnavEntries[] = {
    { "-- GPSNAV --", OME_Label, NULL, profileIndexString, 0},

    { "POS  P", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidPosXY[0],  0, 255, 1 }, 0 },
    { "POS  I", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidPosXY[1],  0, 255, 1 }, 0 },
    { "POSR P", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidVelXY[0], 0, 255, 1 }, 0 },
    { "POSR I", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidVelXY[1], 0, 255, 1 }, 0 },
    { "POSR D", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_pidVelXY[2], 0, 255, 1 }, 0 },

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

static CMS_Menu cmsx_menuPidGpsnav = {
    .GUARD_text = "XGPSNAV",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_menuPidGpsnav_onEnter,
    .onExit = cmsx_menuPidGpsnav_onExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuPidGpsnavEntries,
};

//
// Rate & Expo
//
static controlRateConfig_t rateProfile;
static uint16_t cmsx_rateRoll;
static uint16_t cmsx_ratePitch;
static uint16_t cmsx_rateYaw;

static long cmsx_RateProfileRead(void)
{
    memcpy(&rateProfile, controlRateProfiles(profileIndex), sizeof(controlRateConfig_t));

    cmsx_rateRoll  = DEKADEGREES_TO_DEGREES(rateProfile.rates[FD_ROLL]);
    cmsx_ratePitch = DEKADEGREES_TO_DEGREES(rateProfile.rates[FD_PITCH]);
    cmsx_rateYaw   = DEKADEGREES_TO_DEGREES(rateProfile.rates[FD_YAW]);

    return 0;
}

static long cmsx_RateProfileWriteback(const OSD_Entry *self)
{
    UNUSED(self);

    rateProfile.rates[FD_ROLL]  = DEGREES_TO_DEKADEGREES(cmsx_rateRoll);
    rateProfile.rates[FD_PITCH] = DEGREES_TO_DEKADEGREES(cmsx_ratePitch);
    rateProfile.rates[FD_YAW]   = DEGREES_TO_DEKADEGREES(cmsx_rateYaw);

    memcpy((controlRateConfig_t *)controlRateProfiles(profileIndex), &rateProfile, sizeof(controlRateConfig_t));

    return 0;
}

static long cmsx_RateProfileOnEnter(void)
{
    cmsx_RateProfileRead();

    return 0;
}

static OSD_Entry cmsx_menuRateProfileEntries[] =
{
    { "-- RATE --", OME_Label, NULL, profileIndexString, 0 },

#if 0
    { "RC RATE",     OME_FLOAT,  NULL, &(OSD_FLOAT_t){ &rateProfile.rcRate8,    0, 255, 1, 10 }, 0 },
    { "RC YAW RATE", OME_FLOAT,  NULL, &(OSD_FLOAT_t){ &rateProfile.rcYawRate8, 0, 255, 1, 10 }, 0 },
#endif

    { "ROLL RATE",   OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_rateRoll,  (CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MIN * DEGREES_PER_DEKADEGREE), (CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX * DEGREES_PER_DEKADEGREE), DEGREES_PER_DEKADEGREE }, 0 },
    { "PITCH RATE",  OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_ratePitch, (CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MIN * DEGREES_PER_DEKADEGREE), (CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX * DEGREES_PER_DEKADEGREE), DEGREES_PER_DEKADEGREE }, 0 },
    { "YAW RATE",    OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_rateYaw,   (CONTROL_RATE_CONFIG_YAW_RATE_MIN * DEGREES_PER_DEKADEGREE),        (CONTROL_RATE_CONFIG_YAW_RATE_MAX * DEGREES_PER_DEKADEGREE),        DEGREES_PER_DEKADEGREE }, 0 },

    { "RC EXPO",     OME_UINT8,  NULL, &(OSD_UINT8_t){ &rateProfile.rcExpo8,    0, 100, 1 },     0 },
    { "RC YAW EXP",  OME_UINT8,  NULL, &(OSD_UINT8_t){ &rateProfile.rcYawExpo8, 0, 100, 1 },     0 },

    { "THR MID",     OME_UINT8,  NULL, &(OSD_UINT8_t){ &rateProfile.thrMid8,    0, 100, 1 },     0 },
    { "THR EXPO",    OME_UINT8,  NULL, &(OSD_UINT8_t){ &rateProfile.thrExpo8,   0, 100, 1 },     0 },

    { "THRPID ATT",  OME_UINT8,  NULL, &(OSD_UINT8_t){ &rateProfile.dynThrPID,  0, 100, 1 },     0 },
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

#ifdef NOT_YET
static uint8_t cmsx_dtermSetpointWeight;
static uint8_t cmsx_setpointRelaxRatio;
static uint8_t cmsx_angleStrength;
static uint8_t cmsx_horizonStrength;
static uint8_t cmsx_horizonTransition;

static long cmsx_profileOtherOnEnter(void)
{
    profileIndexString[1] = '0' + tmpProfileIndex;

    cmsx_dtermSetpointWeight = pidProfile()->dtermSetpointWeight;
    cmsx_setpointRelaxRatio  = pidProfile()->setpointRelaxRatio;

    cmsx_angleStrength       = pidProfile()[PIDLEVEL].P;
    cmsx_horizonStrength     = pidProfile()[PIDLEVEL].I;
    cmsx_horizonTransition   = pidProfile()[PIDLEVEL].D;

    return 0;
}

static long cmsx_profileOtherOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    pidProfileMutable()->dtermSetpointWeight = cmsx_dtermSetpointWeight;
    pidProfileMutable()->setpointRelaxRatio  = cmsx_setpointRelaxRatio;

    pidProfileMutable()[PIDLEVEL].P        = cmsx_angleStrength;
    pidProfileMutable()[PIDLEVEL].I        = cmsx_horizonStrength;
    pidProfileMutable()[PIDLEVEL].D        = cmsx_horizonTransition;

    return 0;
}

static OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, profileIndexString, 0 },

    { "D SETPT WT",  OME_FLOAT, NULL, &(OSD_FLOAT_t){ &cmsx_dtermSetpointWeight, 0, 255, 1, 10 }, 0 },
    { "SETPT TRS",   OME_FLOAT, NULL, &(OSD_FLOAT_t){ &cmsx_setpointRelaxRatio,  0, 100, 1, 10 }, 0 },
    { "ANGLE STR",   OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_angleStrength,       0, 200, 1 }    , 0 },
    { "HORZN STR",   OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_horizonStrength,     0, 200, 1 }    , 0 },
    { "HORZN TRS",   OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_horizonTransition,   0, 200, 1 }    , 0 },

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
#endif // NOT_YET

//
// Per profile filters
//
static uint8_t cmsx_dterm_lpf_hz;
static uint8_t cmsx_gyroSoftLpf;
static uint16_t cmsx_yaw_p_limit;
static uint8_t cmsx_yaw_lpf_hz;

static long cmsx_FilterPerProfileRead(void)
{
    cmsx_dterm_lpf_hz = pidProfile()->dterm_lpf_hz;
    cmsx_gyroSoftLpf  = gyroConfig()->gyro_soft_lpf_hz;
    cmsx_yaw_p_limit  = pidProfile()->yaw_p_limit;
    cmsx_yaw_lpf_hz   = pidProfile()->yaw_lpf_hz;

    return 0;
}

static long cmsx_FilterPerProfileWriteback(const OSD_Entry *self)
{
    UNUSED(self);

    pidProfileMutable()->dterm_lpf_hz     = cmsx_dterm_lpf_hz;
    gyroConfigMutable()->gyro_soft_lpf_hz = cmsx_gyroSoftLpf;
    pidProfileMutable()->yaw_p_limit      = cmsx_yaw_p_limit;
    pidProfileMutable()->yaw_lpf_hz       = cmsx_yaw_lpf_hz;

    return 0;
}

static OSD_Entry cmsx_menuFilterPerProfileEntries[] =
{
    { "-- FILTER PP  --", OME_Label, NULL, profileIndexString, 0 },

    { "DTERM LPF",  OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_dterm_lpf_hz,   0, 200, 1 }, 0 },
    { "GYRO SLPF",  OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_gyroSoftLpf,    0, 200, 1 }, 0 },
    { "YAW P LIM",  OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_yaw_p_limit, 100, 500, 1 }, 0 },
    { "YAW LPF",    OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_yaw_lpf_hz,     0, 200, 1 }, 0 },

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

static uint8_t cmsx_gyroSync; // Global
static uint8_t cmsx_gyroSyncDenom; // Global
static uint8_t cmsx_gyroLpf; // Global

static long cmsx_menuGyro_onEnter(void)
{
    cmsx_gyroSync =  gyroConfig()->gyroSync;
    cmsx_gyroSyncDenom = gyroConfig()->gyroSyncDenominator;
    cmsx_gyroLpf = gyroConfig()->gyro_lpf;

    return 0;
}

static long cmsx_menuGyro_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    gyroConfigMutable()->gyroSync = cmsx_gyroSync;
    gyroConfigMutable()->gyroSyncDenominator = cmsx_gyroSyncDenom;
    gyroConfigMutable()->gyro_lpf = cmsx_gyroLpf;

    return 0;
}

static const char *cmsx_gyroSyncNames[] = {
    "OFF", "ON "
};

static const char *cmsx_gyroLpfNames[] = {
    "10 ", "20 ", "42 ", "98 ", "188", "256"
};

static OSD_Entry cmsx_menuGyroEntries[] =
{
    { "-- GYRO GLB --", OME_Label, NULL, profileIndexString, 0},

    { "GYRO SYNC",  OME_TAB,   NULL, &(OSD_TAB_t){ &cmsx_gyroSync, 1, cmsx_gyroSyncNames}, 0 },
    { "GYRO DENOM", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_gyroSyncDenom, 1, 32, 1 },      0 },
    { "GYRO LPF",   OME_TAB,   NULL, &(OSD_TAB_t){ &cmsx_gyroLpf, 5, cmsx_gyroLpfNames},   0 },

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

static CMS_Menu cmsx_menuGyro = {
    .GUARD_text = "XGYROGLB",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_menuGyro_onEnter,
    .onExit = cmsx_menuGyro_onExit,
    .onGlobalExit = NULL,
    .entries = cmsx_menuGyroEntries,
};

static OSD_Entry cmsx_menuImuEntries[] =
{
    { "-- PID TUNING --", OME_Label, NULL, NULL, 0},

    // Profile dependent
    {"PID PROF",   OME_UINT8,   cmsx_profileIndexOnChange,     &(OSD_UINT8_t){ &tmpProfileIndex, 1, MAX_PROFILE_COUNT, 1}, 0},
    {"PID",        OME_Submenu, cmsMenuChange,                 &cmsx_menuPid,                                              0},
    {"PID ALTMAG", OME_Submenu, cmsMenuChange,                 &cmsx_menuPidAltMag,                                        0},
    {"PID GPSNAV", OME_Submenu, cmsMenuChange,                 &cmsx_menuPidGpsnav,                                        0},
    {"FILT PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterPerProfile,                                  0},

    // Rate profile dependent
    {"RATE PROF", OME_UINT8,   cmsx_profileIndexOnChange, &(OSD_UINT8_t){ &tmpProfileIndex, 1, MAX_CONTROL_RATE_PROFILE_COUNT, 1}, 0},
    {"RATE",      OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile,                                       0},

    // Global
    {"GYRO GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuGyro,                                              0},

#ifdef NOT_YET
    {"OTHER PP",  OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther,                                      0},
    // Profile independent
    {"FILT GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal,                                      0},
#endif

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
