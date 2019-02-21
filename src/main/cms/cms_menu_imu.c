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

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/pwm_output.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "pg/pg.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"


//
// PID
//
static uint8_t tmpPidProfileIndex;
static uint8_t pidProfileIndex;
static char pidProfileIndexString[] = " p";
static uint8_t tempPid[3][3];
static uint16_t tempPidF[3];

static uint8_t tmpRateProfileIndex;
static uint8_t rateProfileIndex;
static char rateProfileIndexString[] = " p-r";
static controlRateConfig_t rateProfile;

static const char * const osdTableThrottleLimitType[] = {
    "OFF", "SCALE", "CLIP"
};

#if defined(USE_GYRO_DATA_ANALYSE) && defined(USE_EXTENDED_CMS_MENUS)
static const char * const osdTableDynNotchRangeType[] = {
    "HIGH", "MED", "LOW", "AUTO"
};
#endif

static long cmsx_menuImu_onEnter(void)
{
    pidProfileIndex = getCurrentPidProfileIndex();
    tmpPidProfileIndex = pidProfileIndex + 1;

    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex = rateProfileIndex + 1;

    return 0;
}

static long cmsx_menuImu_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    changePidProfile(pidProfileIndex);
    changeControlRateProfile(rateProfileIndex);

    return 0;
}

static long cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    pidProfileIndex = tmpPidProfileIndex - 1;
    changePidProfile(pidProfileIndex);

    return 0;
}

static long cmsx_rateProfileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    rateProfileIndex = tmpRateProfileIndex - 1;
    changeControlRateProfile(rateProfileIndex);

    return 0;
}

static long cmsx_PidRead(void)
{

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    for (uint8_t i = 0; i < 3; i++) {
        tempPid[i][0] = pidProfile->pid[i].P;
        tempPid[i][1] = pidProfile->pid[i].I;
        tempPid[i][2] = pidProfile->pid[i].D;
        tempPidF[i] = pidProfile->pid[i].F;
    }

    return 0;
}

static long cmsx_PidOnEnter(void)
{
    pidProfileIndexString[1] = '0' + tmpPidProfileIndex;
    cmsx_PidRead();

    return 0;
}

static long cmsx_PidWriteback(const OSD_Entry *self)
{
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;
    for (uint8_t i = 0; i < 3; i++) {
        pidProfile->pid[i].P = tempPid[i][0];
        pidProfile->pid[i].I = tempPid[i][1];
        pidProfile->pid[i].D = tempPid[i][2];
        pidProfile->pid[i].F = tempPidF[i];
    }
    pidInitConfig(currentPidProfile);

    return 0;
}

static const OSD_Entry cmsx_menuPidEntries[] =
{
    { "-- PID --", OME_Label, NULL, pidProfileIndexString, 0},

    { "ROLL  P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][0],  0, 200, 1 }, 0 },
    { "ROLL  I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][1],  0, 200, 1 }, 0 },
    { "ROLL  D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][2],  0, 200, 1 }, 0 },
    { "ROLL  F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_ROLL],  0, 2000, 1 }, 0 },

    { "PITCH P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][0], 0, 200, 1 }, 0 },
    { "PITCH I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][1], 0, 200, 1 }, 0 },
    { "PITCH D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][2], 0, 200, 1 }, 0 },
    { "PITCH F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_PITCH], 0, 2000, 1 }, 0 },

    { "YAW   P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][0],   0, 200, 1 }, 0 },
    { "YAW   I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][1],   0, 200, 1 }, 0 },
    { "YAW   D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][2],   0, 200, 1 }, 0 },
    { "YAW   F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_YAW],   0, 2000, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuPid = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPID",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_PidOnEnter,
    .onExit = cmsx_PidWriteback,
    .entries = cmsx_menuPidEntries
};

//
// Rate & Expo
//

static long cmsx_RateProfileRead(void)
{
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));

    return 0;
}

static long cmsx_RateProfileWriteback(const OSD_Entry *self)
{
    UNUSED(self);

    memcpy(controlRateProfilesMutable(rateProfileIndex), &rateProfile, sizeof(controlRateConfig_t));

    return 0;
}

static long cmsx_RateProfileOnEnter(void)
{
    rateProfileIndexString[1] = '0' + tmpPidProfileIndex;
    rateProfileIndexString[3] = '0' + tmpRateProfileIndex;
    cmsx_RateProfileRead();

    return 0;
}

static const OSD_Entry cmsx_menuRateProfileEntries[] =
{
    { "-- RATE --", OME_Label, NULL, rateProfileIndexString, 0 },

    { "RC R RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_ROLL],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 }, 0 },
    { "RC P RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_PITCH],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 }, 0 },
    { "RC Y RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_YAW], 1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 }, 0 },

    { "ROLL SUPER",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_ROLL],   0, 100, 1, 10 }, 0 },
    { "PITCH SUPER", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_PITCH],   0, 100, 1, 10 }, 0 },
    { "YAW SUPER",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_YAW],   0, 100, 1, 10 }, 0 },

    { "RC R EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_ROLL],    0, 100, 1, 10 }, 0 },
    { "RC P EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_PITCH],    0, 100, 1, 10 }, 0 },
    { "RC Y EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_YAW], 0, 100, 1, 10 }, 0 },

    { "THR MID",     OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrMid8,           0,  100,  1}, 0 },
    { "THR EXPO",    OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrExpo8,          0,  100,  1}, 0 },
    { "THRPID ATT",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.dynThrPID,         0,  100,  1, 10}, 0 },
    { "TPA BRKPT",   OME_UINT16, NULL, &(OSD_UINT16_t){ &rateProfile.tpa_breakpoint, 1000, 2000, 10}, 0 },

    { "THR LIM TYPE",OME_TAB,    NULL, &(OSD_TAB_t)   { &rateProfile.throttle_limit_type, THROTTLE_LIMIT_TYPE_COUNT - 1, osdTableThrottleLimitType}, 0 },
    { "THR LIM %",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.throttle_limit_percent, 25,  100,  1}, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuRateProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENURATE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_RateProfileOnEnter,
    .onExit = cmsx_RateProfileWriteback,
    .entries = cmsx_menuRateProfileEntries
};

#ifdef USE_LAUNCH_CONTROL
static uint8_t cmsx_launchControlMode;
static uint8_t cmsx_launchControlAllowTriggerReset;
static uint8_t cmsx_launchControlThrottlePercent;
static uint8_t cmsx_launchControlAngleLimit;
static uint8_t cmsx_launchControlGain;

static long cmsx_launchControlOnEnter(void)
{
    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    
    cmsx_launchControlMode  = pidProfile->launchControlMode;
    cmsx_launchControlAllowTriggerReset  = pidProfile->launchControlAllowTriggerReset;
    cmsx_launchControlThrottlePercent  = pidProfile->launchControlThrottlePercent;
    cmsx_launchControlAngleLimit  = pidProfile->launchControlAngleLimit;
    cmsx_launchControlGain  = pidProfile->launchControlGain;

    return 0;
}

static long cmsx_launchControlOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

    pidProfile->launchControlMode = cmsx_launchControlMode;
    pidProfile->launchControlAllowTriggerReset = cmsx_launchControlAllowTriggerReset;
    pidProfile->launchControlThrottlePercent = cmsx_launchControlThrottlePercent;
    pidProfile->launchControlAngleLimit = cmsx_launchControlAngleLimit;
    pidProfile->launchControlGain = cmsx_launchControlGain;

    return 0;
}

static const OSD_Entry cmsx_menuLaunchControlEntries[] = {
    { "-- LAUNCH CONTROL --", OME_Label, NULL, pidProfileIndexString, 0 },

    { "MODE",             OME_TAB,   NULL, &(OSD_TAB_t)   { &cmsx_launchControlMode, LAUNCH_CONTROL_MODE_COUNT - 1, osdLaunchControlModeNames}, 0 },
    { "ALLOW RESET",      OME_Bool,  NULL, &cmsx_launchControlAllowTriggerReset, 0 },
    { "TRIGGER THROTTLE", OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_launchControlThrottlePercent, 0,  LAUNCH_CONTROL_THROTTLE_TRIGGER_MAX, 1 } , 0 },
    { "ANGLE LIMIT",      OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_launchControlAngleLimit,      0,  80, 1 } , 0 },
    { "ITERM GAIN",       OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_launchControlGain,            0, 200, 1 } , 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuLaunchControl = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "LAUNCH",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_launchControlOnEnter,
    .onExit = cmsx_launchControlOnExit,
    .entries = cmsx_menuLaunchControlEntries,
};
#endif

static uint8_t  cmsx_feedForwardTransition;
static uint8_t  cmsx_angleStrength;
static uint8_t  cmsx_horizonStrength;
static uint8_t  cmsx_horizonTransition;
static uint8_t  cmsx_throttleBoost;
static uint16_t cmsx_itermAcceleratorGain;
static uint16_t cmsx_itermThrottleThreshold;
static uint8_t  cmsx_motorOutputLimit;
static int8_t   cmsx_autoProfileCellCount;
#ifdef USE_D_MIN
static uint8_t  cmsx_d_min[XYZ_AXIS_COUNT];
#endif

static long cmsx_profileOtherOnEnter(void)
{
    pidProfileIndexString[1] = '0' + tmpPidProfileIndex;

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    
    cmsx_feedForwardTransition  = pidProfile->feedForwardTransition;

    cmsx_angleStrength =     pidProfile->pid[PID_LEVEL].P;
    cmsx_horizonStrength =   pidProfile->pid[PID_LEVEL].I;
    cmsx_horizonTransition = pidProfile->pid[PID_LEVEL].D;

    cmsx_itermAcceleratorGain   = pidProfile->itermAcceleratorGain;
    cmsx_itermThrottleThreshold = pidProfile->itermThrottleThreshold;

    cmsx_throttleBoost = pidProfile->throttle_boost;
    cmsx_motorOutputLimit = pidProfile->motor_output_limit;
    cmsx_autoProfileCellCount = pidProfile->auto_profile_cell_count;

#ifdef USE_D_MIN
    for (unsigned i = 0; i < XYZ_AXIS_COUNT; i++) {
        cmsx_d_min[i]  = pidProfile->d_min[i];
    }
#endif

    return 0;
}

static long cmsx_profileOtherOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
    pidProfile->feedForwardTransition = cmsx_feedForwardTransition;
    pidInitConfig(currentPidProfile);

    pidProfile->pid[PID_LEVEL].P = cmsx_angleStrength;
    pidProfile->pid[PID_LEVEL].I = cmsx_horizonStrength;
    pidProfile->pid[PID_LEVEL].D = cmsx_horizonTransition;

    pidProfile->itermAcceleratorGain   = cmsx_itermAcceleratorGain;
    pidProfile->itermThrottleThreshold = cmsx_itermThrottleThreshold;

    pidProfile->throttle_boost = cmsx_throttleBoost;
    pidProfile->motor_output_limit = cmsx_motorOutputLimit;
    pidProfile->auto_profile_cell_count = cmsx_autoProfileCellCount;

#ifdef USE_D_MIN
    for (unsigned i = 0; i < XYZ_AXIS_COUNT; i++) {
        pidProfile->d_min[i] = cmsx_d_min[i];
    }
#endif

    return 0;
}

static const OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, pidProfileIndexString, 0 },

    { "FF TRANS",    OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_feedForwardTransition,  0,    100,   1, 10 }, 0 },
    { "ANGLE STR",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleStrength,          0,    200,   1  }   , 0 },
    { "HORZN STR",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonStrength,        0,    200,   1  }   , 0 },
    { "HORZN TRS",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonTransition,      0,    200,   1  }   , 0 },
    { "AG GAIN",     OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_itermAcceleratorGain,   1000, 30000, 10 }   , 0 },
    { "AG THR",      OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_itermThrottleThreshold, 20,   1000,  1  }   , 0 },
#ifdef USE_THROTTLE_BOOST
    { "THR BOOST",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_throttleBoost,          0,    100,   1  }   , 0 },
#endif
#ifdef USE_LAUNCH_CONTROL
    {"LAUNCH CONTROL", OME_Submenu, cmsMenuChange, &cmsx_menuLaunchControl, 0 },
#endif
    { "MTR OUT LIM %",OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_motorOutputLimit, MOTOR_OUTPUT_LIMIT_PERCENT_MIN,  MOTOR_OUTPUT_LIMIT_PERCENT_MAX,  1}, 0 },

    { "AUTO CELL CNT", OME_INT8, NULL, &(OSD_INT8_t) { &cmsx_autoProfileCellCount, AUTO_PROFILE_CELL_COUNT_CHANGE, MAX_AUTO_DETECT_CELL_COUNT, 1}, 0 },

#ifdef USE_D_MIN
    { "D_MIN_ROLL",  OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min[FD_ROLL],      0, 100, 1 }, 0 },
    { "D_MIN_PITCH", OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min[FD_PITCH],     0, 100, 1 }, 0 },
    { "D_MIN_YAW",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min[FD_YAW],       0, 100, 1 }, 0 },
#endif

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuProfileOther = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFOTHER",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileOtherOnEnter,
    .onExit = cmsx_profileOtherOnExit,
    .entries = cmsx_menuProfileOtherEntries,
};


static uint16_t gyroConfig_gyro_lowpass_hz;
static uint16_t gyroConfig_gyro_lowpass2_hz;
static uint16_t gyroConfig_gyro_soft_notch_hz_1;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_1;
static uint16_t gyroConfig_gyro_soft_notch_hz_2;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_2;

static long cmsx_menuGyro_onEnter(void)
{
    gyroConfig_gyro_lowpass_hz =  gyroConfig()->gyro_lowpass_hz;
    gyroConfig_gyro_lowpass2_hz =  gyroConfig()->gyro_lowpass2_hz;
    gyroConfig_gyro_soft_notch_hz_1 = gyroConfig()->gyro_soft_notch_hz_1;
    gyroConfig_gyro_soft_notch_cutoff_1 = gyroConfig()->gyro_soft_notch_cutoff_1;
    gyroConfig_gyro_soft_notch_hz_2 = gyroConfig()->gyro_soft_notch_hz_2;
    gyroConfig_gyro_soft_notch_cutoff_2 = gyroConfig()->gyro_soft_notch_cutoff_2;

    return 0;
}

static long cmsx_menuGyro_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    gyroConfigMutable()->gyro_lowpass_hz =  gyroConfig_gyro_lowpass_hz;
    gyroConfigMutable()->gyro_lowpass2_hz =  gyroConfig_gyro_lowpass2_hz;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = gyroConfig_gyro_soft_notch_hz_1;
    gyroConfigMutable()->gyro_soft_notch_cutoff_1 = gyroConfig_gyro_soft_notch_cutoff_1;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = gyroConfig_gyro_soft_notch_hz_2;
    gyroConfigMutable()->gyro_soft_notch_cutoff_2 = gyroConfig_gyro_soft_notch_cutoff_2;

    return 0;
}

static const OSD_Entry cmsx_menuFilterGlobalEntries[] =
{
    { "-- FILTER GLB  --", OME_Label, NULL, NULL, 0 },

    { "GYRO LPF",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass_hz, 0, 16000, 1 }, 0 },
#ifdef USE_GYRO_LPF2
    { "GYRO LPF2",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass2_hz,  0, 16000, 1 }, 0 },
#endif
    { "GYRO NF1",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_1,     0, 500, 1 }, 0 },
    { "GYRO NF1C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_1, 0, 500, 1 }, 0 },
    { "GYRO NF2",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_2,     0, 500, 1 }, 0 },
    { "GYRO NF2C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_2, 0, 500, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuFilterGlobal = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTGLB",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuGyro_onEnter,
    .onExit = cmsx_menuGyro_onExit,
    .entries = cmsx_menuFilterGlobalEntries,
};

#if (defined(USE_GYRO_DATA_ANALYSE) || defined(USE_DYN_LPF)) && defined(USE_EXTENDED_CMS_MENUS)

#ifdef USE_GYRO_DATA_ANALYSE
static uint8_t  dynFiltNotchRange;
static uint8_t  dynFiltWidthPercent;
static uint16_t dynFiltNotchQ;
static uint16_t dynFiltNotchMinHz;
#endif
#ifdef USE_DYN_LPF
static uint16_t dynFiltGyroMin;
static uint16_t dynFiltGyroMax;
static uint16_t dynFiltDtermMin;
static uint16_t dynFiltDtermMax;
#endif

static long cmsx_menuDynFilt_onEnter(void)
{
#ifdef USE_GYRO_DATA_ANALYSE
    dynFiltNotchRange   = gyroConfig()->dyn_notch_range;
    dynFiltWidthPercent = gyroConfig()->dyn_notch_width_percent;
    dynFiltNotchQ       = gyroConfig()->dyn_notch_q;
    dynFiltNotchMinHz   = gyroConfig()->dyn_notch_min_hz;
#endif
#ifdef USE_DYN_LPF
    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    dynFiltGyroMin  = gyroConfig()->dyn_lpf_gyro_min_hz;
    dynFiltGyroMax  = gyroConfig()->dyn_lpf_gyro_max_hz;
    dynFiltDtermMin = pidProfile->dyn_lpf_dterm_min_hz;
    dynFiltDtermMax = pidProfile->dyn_lpf_dterm_max_hz;
#endif

    return 0;
}

static long cmsx_menuDynFilt_onExit(const OSD_Entry *self)
{
    UNUSED(self);

#ifdef USE_GYRO_DATA_ANALYSE
    gyroConfigMutable()->dyn_notch_range         = dynFiltNotchRange;
    gyroConfigMutable()->dyn_notch_width_percent = dynFiltWidthPercent;
    gyroConfigMutable()->dyn_notch_q             = dynFiltNotchQ;
    gyroConfigMutable()->dyn_notch_min_hz        = dynFiltNotchMinHz;
#endif
#ifdef USE_DYN_LPF
    pidProfile_t *pidProfile = currentPidProfile;
    gyroConfigMutable()->dyn_lpf_gyro_min_hz = dynFiltGyroMin;
    gyroConfigMutable()->dyn_lpf_gyro_max_hz = dynFiltGyroMax;
    pidProfile->dyn_lpf_dterm_min_hz         = dynFiltDtermMin;
    pidProfile->dyn_lpf_dterm_max_hz         = dynFiltDtermMax;
#endif

    return 0;
}

static const OSD_Entry cmsx_menuDynFiltEntries[] =
{
    { "-- DYN FILT --", OME_Label, NULL, NULL, 0 },

#ifdef USE_GYRO_DATA_ANALYSE
    { "NOTCH RANGE",    OME_TAB,    NULL, &(OSD_TAB_t)    { &dynFiltNotchRange,   3, osdTableDynNotchRangeType}, 0 },
    { "NOTCH WIDTH %",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &dynFiltWidthPercent, 0, 20, 1 }, 0 },
    { "NOTCH Q",        OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchQ,       0, 1000, 1 }, 0 },
    { "NOTCH MIN HZ",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchMinHz,   0, 1000, 1 }, 0 },
#endif

#ifdef USE_DYN_LPF
    { "LPF GYRO MIN",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltGyroMin,  0, 1000, 1 }, 0 },
    { "LPF GYRO MAX",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltGyroMax,  0, 1000, 1 }, 0 },
    { "DTERM DLPF MIN", OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltDtermMin, 0, 1000, 1 }, 0 },
    { "DTERM DLPF MAX", OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltDtermMax, 0, 1000, 1 }, 0 },
#endif

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuDynFilt = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XDYNFLT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuDynFilt_onEnter,
    .onExit = cmsx_menuDynFilt_onExit,
    .entries = cmsx_menuDynFiltEntries,
};

#endif

static uint16_t cmsx_dterm_lowpass_hz;
static uint16_t cmsx_dterm_lowpass2_hz;
static uint16_t cmsx_dterm_notch_hz;
static uint16_t cmsx_dterm_notch_cutoff;
static uint16_t cmsx_yaw_lowpass_hz;

static long cmsx_FilterPerProfileRead(void)
{
    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_dterm_lowpass_hz   = pidProfile->dterm_lowpass_hz;
    cmsx_dterm_lowpass2_hz  = pidProfile->dterm_lowpass2_hz;
    cmsx_dterm_notch_hz     = pidProfile->dterm_notch_hz;
    cmsx_dterm_notch_cutoff = pidProfile->dterm_notch_cutoff;
    cmsx_yaw_lowpass_hz     = pidProfile->yaw_lowpass_hz;

    return 0;
}

static long cmsx_FilterPerProfileWriteback(const OSD_Entry *self)
{
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;

    pidProfile->dterm_lowpass_hz   = cmsx_dterm_lowpass_hz;
    pidProfile->dterm_lowpass2_hz  = cmsx_dterm_lowpass2_hz;
    pidProfile->dterm_notch_hz     = cmsx_dterm_notch_hz;
    pidProfile->dterm_notch_cutoff = cmsx_dterm_notch_cutoff;
    pidProfile->yaw_lowpass_hz     = cmsx_yaw_lowpass_hz;

    return 0;
}

static const OSD_Entry cmsx_menuFilterPerProfileEntries[] =
{
    { "-- FILTER PP  --", OME_Label, NULL, NULL, 0 },

    { "DTERM LPF",  OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass_hz,     0, 500, 1 }, 0 },
    { "DTERM LPF2", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass2_hz,    0, 500, 1 }, 0 },
    { "DTERM NF",   OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_notch_hz,       0, 500, 1 }, 0 },
    { "DTERM NFCO", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_notch_cutoff,   0, 500, 1 }, 0 },
    { "YAW LPF",    OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_yaw_lowpass_hz,       0, 500, 1 }, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuFilterPerProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTPP",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_FilterPerProfileRead,
    .onExit = cmsx_FilterPerProfileWriteback,
    .entries = cmsx_menuFilterPerProfileEntries,
};

#ifdef USE_EXTENDED_CMS_MENUS

static uint8_t cmsx_dstPidProfile;
static uint8_t cmsx_dstControlRateProfile;

static const char * const cmsx_ProfileNames[] = {
    "-",
    "1",
    "2",
    "3"
};

static OSD_TAB_t cmsx_PidProfileTable = { &cmsx_dstPidProfile, 3, cmsx_ProfileNames };
static OSD_TAB_t cmsx_ControlRateProfileTable = { &cmsx_dstControlRateProfile, 3, cmsx_ProfileNames };

static long cmsx_menuCopyProfile_onEnter(void)
{
    cmsx_dstPidProfile = 0;
    cmsx_dstControlRateProfile = 0;

    return 0;
}

static long cmsx_CopyPidProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstPidProfile > 0) {
        pidCopyProfile(cmsx_dstPidProfile - 1, getCurrentPidProfileIndex());
    }

    return 0;
}

static long cmsx_CopyControlRateProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstControlRateProfile > 0) {
        copyControlRateProfile(cmsx_dstControlRateProfile - 1, getCurrentControlRateProfileIndex());
    }

    return 0;
}

static const OSD_Entry cmsx_menuCopyProfileEntries[] =
{
    { "-- COPY PROFILE --", OME_Label, NULL, NULL, 0},

    { "CPY PID PROF TO",   OME_TAB,      NULL,                        &cmsx_PidProfileTable, 0 },
    { "COPY PP",           OME_Funcall,  cmsx_CopyPidProfile,         NULL, 0 },
    { "CPY RATE PROF TO",  OME_TAB,      NULL,                        &cmsx_ControlRateProfileTable, 0 },
    { "COPY RP",           OME_Funcall,  cmsx_CopyControlRateProfile, NULL, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuCopyProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XCPY",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuCopyProfile_onEnter,
    .onExit = NULL,
    .entries = cmsx_menuCopyProfileEntries,
};

#endif

static const OSD_Entry cmsx_menuImuEntries[] =
{
    { "-- PROFILE --", OME_Label, NULL, NULL, 0},

    {"PID PROF",  OME_UINT8,   cmsx_profileIndexOnChange,     &(OSD_UINT8_t){ &tmpPidProfileIndex, 1, PID_PROFILE_COUNT, 1},    0},
    {"PID",       OME_Submenu, cmsMenuChange,                 &cmsx_menuPid,                                                 0},
    {"MISC PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther,                                        0},
    {"FILT PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterPerProfile,                                    0},

    {"RATE PROF", OME_UINT8,   cmsx_rateProfileIndexOnChange, &(OSD_UINT8_t){ &tmpRateProfileIndex, 1, CONTROL_RATE_PROFILE_COUNT, 1}, 0},
    {"RATE",      OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile,                                         0},

    {"FILT GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal,                                        0},
#if  (defined(USE_GYRO_DATA_ANALYSE) || defined(USE_DYN_LPF)) && defined(USE_EXTENDED_CMS_MENUS)
    {"DYN FILT",  OME_Submenu, cmsMenuChange,                 &cmsx_menuDynFilt,                                             0},
#endif

#ifdef USE_EXTENDED_CMS_MENUS
    {"COPY PROF", OME_Submenu, cmsMenuChange,                 &cmsx_menuCopyProfile,                                         0},
#endif /* USE_EXTENDED_CMS_MENUS */

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuImu = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XIMU",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuImu_onEnter,
    .onExit = cmsx_menuImu_onExit,
    .entries = cmsx_menuImuEntries,
};

#endif // CMS
