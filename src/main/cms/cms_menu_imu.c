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
#include "build/build_config.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/utils.h"

#include "config/feature.h"
#include "config/simplified_tuning.h"

#include "drivers/pwm_output.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/pid_init.h"

#include "pg/pg.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "cli/settings.h"

//
// PID
//

#define PROFILE_INDEX_STRING_ADDITIONAL_SIZE 5 // Additional symbols for setProfileIndexString(): "2 (NAMENAME)\0"

static uint8_t tmpPidProfileIndex;
static uint8_t pidProfileIndex;
static char pidProfileIndexString[MAX_PROFILE_NAME_LENGTH + PROFILE_INDEX_STRING_ADDITIONAL_SIZE];
static uint8_t tempPid[3][3];
static uint16_t tempPidF[3];

static uint8_t tmpRateProfileIndex;
static uint8_t rateProfileIndex;
static char rateProfileIndexString[MAX_RATE_PROFILE_NAME_LENGTH + PROFILE_INDEX_STRING_ADDITIONAL_SIZE];
static controlRateConfig_t rateProfile;

static const char * const osdTableThrottleLimitType[] = {
    "OFF", "SCALE", "CLIP"
};

#ifdef USE_MULTI_GYRO
static const char * const osdTableGyroToUse[] = {
    "FIRST", "SECOND", "BOTH"
};
#endif

static void setProfileIndexString(char *profileString, int profileIndex, const char *profileName)
{
    int charIndex = 0;
    profileString[charIndex++] = '1' + profileIndex;

#ifdef USE_PROFILE_NAMES
    const int profileNameLen = strlen(profileName);

    if (profileNameLen > 0) {
        profileString[charIndex++] = ' ';
        profileString[charIndex++] = '(';
        for (int i = 0; i < profileNameLen; i++) {
            profileString[charIndex++] = toupper(profileName[i]);
        }
        profileString[charIndex++] = ')';
    }
#else
    UNUSED(profileName);
#endif

    profileString[charIndex] = '\0';
}

static char pidProfileNames[PID_PROFILE_COUNT][MAX_PROFILE_NAME_LENGTH + PROFILE_INDEX_STRING_ADDITIONAL_SIZE];
static const char *pidProfileNamePtrs[PID_PROFILE_COUNT];

static char rateProfileNames[CONTROL_RATE_PROFILE_COUNT][MAX_PROFILE_NAME_LENGTH + PROFILE_INDEX_STRING_ADDITIONAL_SIZE];
static const char *rateProfileNamePtrs[CONTROL_RATE_PROFILE_COUNT];

static const void *cmsx_menuImu_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        setProfileIndexString(pidProfileNames[i], i, pidProfiles(i)->profileName);
        pidProfileNamePtrs[i] = pidProfileNames[i];
    }

    pidProfileIndex = getCurrentPidProfileIndex();
    tmpPidProfileIndex = pidProfileIndex;

    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        setProfileIndexString(rateProfileNames[i], i, controlRateProfilesMutable(i)->profileName);
        rateProfileNamePtrs[i] = rateProfileNames[i];
    }

    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex = rateProfileIndex;

    return NULL;
}

static const void *cmsx_menuImu_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    changePidProfile(pidProfileIndex);
    changeControlRateProfile(rateProfileIndex);

    return NULL;
}

static const void *cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    pidProfileIndex = tmpPidProfileIndex;
    changePidProfile(pidProfileIndex);

    return NULL;
}

static const void *cmsx_rateProfileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    rateProfileIndex = tmpRateProfileIndex;
    changeControlRateProfile(rateProfileIndex);

    return NULL;
}

static const void *cmsx_PidRead(void)
{

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    for (uint8_t i = 0; i < 3; i++) {
        tempPid[i][0] = pidProfile->pid[i].P;
        tempPid[i][1] = pidProfile->pid[i].I;
        tempPid[i][2] = pidProfile->pid[i].D;
        tempPidF[i] = pidProfile->pid[i].F;
    }

    return NULL;
}

static const void *cmsx_PidOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, currentPidProfile->profileName);
    cmsx_PidRead();

    return NULL;
}

static const void *cmsx_PidWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;
    for (uint8_t i = 0; i < 3; i++) {
        pidProfile->pid[i].P = tempPid[i][0];
        pidProfile->pid[i].I = tempPid[i][1];
        pidProfile->pid[i].D = tempPid[i][2];
        pidProfile->pid[i].F = tempPidF[i];
    }
    pidInitConfig(currentPidProfile);

    return NULL;
}

static OSD_Entry cmsx_menuPidEntries[] =
{
    { "-- PID --", OME_Label, NULL, pidProfileIndexString},

    { "ROLL  P", OME_UINT8 | SLIDER_RP, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][0],  0, 200, 1 }},
    { "ROLL  I", OME_UINT8 | SLIDER_RP, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][1],  0, 200, 1 }},
    { "ROLL  D", OME_UINT8 | SLIDER_RP, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][2],  0, 200, 1 }},
    { "ROLL  F", OME_UINT16 | SLIDER_RP, NULL, &(OSD_UINT16_t){ &tempPidF[PID_ROLL],  0, 2000, 1 }},

    { "PITCH P", OME_UINT8 | SLIDER_RP, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][0], 0, 200, 1 }},
    { "PITCH I", OME_UINT8 | SLIDER_RP, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][1], 0, 200, 1 }},
    { "PITCH D", OME_UINT8 | SLIDER_RP, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][2], 0, 200, 1 }},
    { "PITCH F", OME_UINT16 | SLIDER_RP, NULL, &(OSD_UINT16_t){ &tempPidF[PID_PITCH], 0, 2000, 1 }},

    { "YAW   P", OME_UINT8 | SLIDER_RPY, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][0],   0, 200, 1 }},
    { "YAW   I", OME_UINT8 | SLIDER_RPY, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][1],   0, 200, 1 }},
    { "YAW   D", OME_UINT8 | SLIDER_RPY, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][2],   0, 200, 1 }},
    { "YAW   F", OME_UINT16 | SLIDER_RPY, NULL, &(OSD_UINT16_t){ &tempPidF[PID_YAW],   0, 2000, 1 }},

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuPid = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPID",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_PidOnEnter,
    .onExit = cmsx_PidWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuPidEntries
};

#ifdef USE_SIMPLIFIED_TUNING
static uint8_t cmsx_simplified_pids_mode;
static uint8_t cmsx_simplified_master_multiplier;
static uint8_t cmsx_simplified_roll_pitch_ratio;
static uint8_t cmsx_simplified_i_gain;
static uint8_t cmsx_simplified_d_gain;
static uint8_t cmsx_simplified_pi_gain;
#ifdef USE_D_MIN
static uint8_t cmsx_simplified_dmin_ratio;
#endif
static uint8_t cmsx_simplified_feedforward_gain;
static uint8_t cmsx_simplified_pitch_pi_gain;

static uint8_t cmsx_simplified_dterm_filter;
static uint8_t cmsx_simplified_dterm_filter_multiplier;
static uint8_t cmsx_simplified_gyro_filter;
static uint8_t cmsx_simplified_gyro_filter_multiplier;
static uint8_t cmsx_tpa_rate;
static uint16_t cmsx_tpa_breakpoint;

static const void *cmsx_simplifiedTuningOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_simplified_pids_mode = pidProfile->simplified_pids_mode;
    cmsx_simplified_master_multiplier = pidProfile->simplified_master_multiplier;
    cmsx_simplified_roll_pitch_ratio = pidProfile->simplified_roll_pitch_ratio;
    cmsx_simplified_i_gain = pidProfile->simplified_i_gain;
    cmsx_simplified_d_gain = pidProfile->simplified_d_gain;
    cmsx_simplified_pi_gain = pidProfile->simplified_pi_gain;
#ifdef USE_D_MIN
    cmsx_simplified_dmin_ratio = pidProfile->simplified_dmin_ratio;
#endif
    cmsx_simplified_feedforward_gain = pidProfile->simplified_feedforward_gain;
    cmsx_simplified_pitch_pi_gain = pidProfile->simplified_pitch_pi_gain;

    cmsx_simplified_dterm_filter = pidProfile->simplified_dterm_filter;
    cmsx_simplified_dterm_filter_multiplier = pidProfile->simplified_dterm_filter_multiplier;
    cmsx_simplified_gyro_filter = gyroConfig()->simplified_gyro_filter;
    cmsx_simplified_gyro_filter_multiplier = gyroConfig()->simplified_gyro_filter_multiplier;

    return 0;
}

static const void *cmsx_simplifiedTuningOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;

    const bool anySettingChanged = pidProfile->simplified_pids_mode != cmsx_simplified_pids_mode
        || pidProfile->simplified_master_multiplier != cmsx_simplified_master_multiplier
        || pidProfile->simplified_roll_pitch_ratio != cmsx_simplified_roll_pitch_ratio
        || pidProfile->simplified_i_gain != cmsx_simplified_i_gain
        || pidProfile->simplified_d_gain != cmsx_simplified_d_gain
        || pidProfile->simplified_pi_gain != cmsx_simplified_pi_gain
#ifdef USE_D_MIN
        || pidProfile->simplified_dmin_ratio != cmsx_simplified_dmin_ratio
#endif
        || pidProfile->simplified_feedforward_gain != cmsx_simplified_feedforward_gain
        || pidProfile->simplified_pitch_pi_gain != cmsx_simplified_pitch_pi_gain
        || pidProfile->simplified_dterm_filter != cmsx_simplified_dterm_filter
        || pidProfile->simplified_dterm_filter_multiplier != cmsx_simplified_dterm_filter_multiplier
        || gyroConfigMutable()->simplified_gyro_filter != cmsx_simplified_gyro_filter
        || gyroConfigMutable()->simplified_gyro_filter_multiplier != cmsx_simplified_gyro_filter_multiplier;

    if (anySettingChanged) {
        pidProfile->simplified_pids_mode = cmsx_simplified_pids_mode;
        pidProfile->simplified_master_multiplier = cmsx_simplified_master_multiplier;
        pidProfile->simplified_roll_pitch_ratio = cmsx_simplified_roll_pitch_ratio;
        pidProfile->simplified_i_gain = cmsx_simplified_i_gain;
        pidProfile->simplified_d_gain = cmsx_simplified_d_gain;
        pidProfile->simplified_pi_gain = cmsx_simplified_pi_gain;
#ifdef USE_D_MIN
        pidProfile->simplified_dmin_ratio = cmsx_simplified_dmin_ratio;
#endif
        pidProfile->simplified_feedforward_gain = cmsx_simplified_feedforward_gain;
        pidProfile->simplified_pitch_pi_gain = cmsx_simplified_pitch_pi_gain;

        pidProfile->simplified_dterm_filter = cmsx_simplified_dterm_filter;
        pidProfile->simplified_dterm_filter_multiplier = cmsx_simplified_dterm_filter_multiplier;
        gyroConfigMutable()->simplified_gyro_filter = cmsx_simplified_gyro_filter;
        gyroConfigMutable()->simplified_gyro_filter_multiplier = cmsx_simplified_gyro_filter_multiplier;

        applySimplifiedTuning(currentPidProfile, gyroConfigMutable());
    }

    return 0;
}

static const OSD_Entry cmsx_menuSimplifiedTuningEntries[] =
{
    { "-- SIMPLIFIED PID --", OME_Label, NULL, NULL},
    { "PID TUNING",        OME_TAB,   NULL, &(OSD_TAB_t)   { &cmsx_simplified_pids_mode, PID_SIMPLIFIED_TUNING_MODE_COUNT - 1, lookupTableSimplifiedTuningPidsMode } },

    { "-- BASIC --",       OME_Label, NULL, NULL},
    { "D GAINS",           OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_d_gain,            SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },
    { "P&I GAINS",         OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_pi_gain,           SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },
    { "FF GAINS",          OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_feedforward_gain,  SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },

    { "-- EXPERT --",      OME_Label, NULL, NULL},
#ifdef USE_D_MIN
    { "D MAX",             OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_dmin_ratio,        SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },
#endif
    { "I GAINS",           OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_i_gain,            SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },

    { "PITCH:ROLL D",      OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_roll_pitch_ratio,  SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },
    { "PITCH:ROLL P,I&FF", OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_pitch_pi_gain,     SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },
    { "MASTER MULT",       OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_master_multiplier, SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },

    { "-- SIMPLIFIED FILTER --", OME_Label, NULL, NULL},
    { "GYRO TUNING",       OME_TAB,   NULL, &(OSD_TAB_t)   { &cmsx_simplified_gyro_filter,  1, lookupTableOffOn } },
    { "GYRO MULT",         OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_gyro_filter_multiplier,  SIMPLIFIED_TUNING_FILTERS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },
    { "DTERM TUNING",      OME_TAB,   NULL, &(OSD_TAB_t)   { &cmsx_simplified_dterm_filter, 1, lookupTableOffOn } },
    { "DTERM MULT",        OME_FLOAT, NULL, &(OSD_FLOAT_t) { &cmsx_simplified_dterm_filter_multiplier, SIMPLIFIED_TUNING_FILTERS_MIN, SIMPLIFIED_TUNING_MAX, 5, 10 } },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuSimplifiedTuning = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XSIMPLIFIED",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_simplifiedTuningOnEnter,
    .onExit = cmsx_simplifiedTuningOnExit,
    .entries = cmsx_menuSimplifiedTuningEntries,
};
#endif // USE_SIMPLIFIED_TUNING

//
// Rate & Expo
//

static const void *cmsx_RateProfileRead(void)
{
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));

    return NULL;
}

static const void *cmsx_RateProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    memcpy(controlRateProfilesMutable(rateProfileIndex), &rateProfile, sizeof(controlRateConfig_t));

    return NULL;
}

static const void *cmsx_RateProfileOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(rateProfileIndexString, rateProfileIndex, controlRateProfilesMutable(rateProfileIndex)->profileName);
    cmsx_RateProfileRead();

    return NULL;
}

static const OSD_Entry cmsx_menuRateProfileEntries[] =
{
    { "-- RATE --", OME_Label, NULL, rateProfileIndexString },

    { "RC R RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_ROLL],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 } },
    { "RC P RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_PITCH],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 } },
    { "RC Y RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_YAW], 1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 } },

    { "ROLL SUPER",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_ROLL],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 } },
    { "PITCH SUPER", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_PITCH],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 } },
    { "YAW SUPER",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_YAW],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 } },

    { "RC R EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_ROLL],    0, 100, 1, 10 } },
    { "RC P EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_PITCH],    0, 100, 1, 10 } },
    { "RC Y EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_YAW], 0, 100, 1, 10 } },

    { "THR MID",     OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrMid8,           0,  100,  1} },
    { "THR EXPO",    OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrExpo8,          0,  100,  1} },

    { "THR LIM TYPE",OME_TAB,    NULL, &(OSD_TAB_t)   { &rateProfile.throttle_limit_type, THROTTLE_LIMIT_TYPE_COUNT - 1, osdTableThrottleLimitType} },
    { "THR LIM %",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.throttle_limit_percent, 25,  100,  1} },

    { "ROLL LVL EXPO",  OME_FLOAT, NULL, &(OSD_FLOAT_t) { &rateProfile.levelExpo[FD_ROLL],  0, 100, 1, 10 } },
    { "PITCH LVL EXPO", OME_FLOAT, NULL, &(OSD_FLOAT_t) { &rateProfile.levelExpo[FD_PITCH], 0, 100, 1, 10 } },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuRateProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENURATE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_RateProfileOnEnter,
    .onExit = cmsx_RateProfileWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuRateProfileEntries
};

#ifdef USE_LAUNCH_CONTROL
static uint8_t cmsx_launchControlMode;
static uint8_t cmsx_launchControlAllowTriggerReset;
static uint8_t cmsx_launchControlThrottlePercent;
static uint8_t cmsx_launchControlAngleLimit;
static uint8_t cmsx_launchControlGain;

static const void *cmsx_launchControlOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_launchControlMode  = pidProfile->launchControlMode;
    cmsx_launchControlAllowTriggerReset  = pidProfile->launchControlAllowTriggerReset;
    cmsx_launchControlThrottlePercent  = pidProfile->launchControlThrottlePercent;
    cmsx_launchControlAngleLimit  = pidProfile->launchControlAngleLimit;
    cmsx_launchControlGain  = pidProfile->launchControlGain;

    return NULL;
}

static const void *cmsx_launchControlOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

    pidProfile->launchControlMode = cmsx_launchControlMode;
    pidProfile->launchControlAllowTriggerReset = cmsx_launchControlAllowTriggerReset;
    pidProfile->launchControlThrottlePercent = cmsx_launchControlThrottlePercent;
    pidProfile->launchControlAngleLimit = cmsx_launchControlAngleLimit;
    pidProfile->launchControlGain = cmsx_launchControlGain;

    return NULL;
}

static const OSD_Entry cmsx_menuLaunchControlEntries[] = {
    { "-- LAUNCH CONTROL --", OME_Label, NULL, pidProfileIndexString },

    { "MODE",             OME_TAB,   NULL, &(OSD_TAB_t)   { &cmsx_launchControlMode, LAUNCH_CONTROL_MODE_COUNT - 1, osdLaunchControlModeNames} },
    { "ALLOW RESET",      OME_Bool,  NULL, &cmsx_launchControlAllowTriggerReset },
    { "TRIGGER THROTTLE", OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_launchControlThrottlePercent, 0,  LAUNCH_CONTROL_THROTTLE_TRIGGER_MAX, 1 }  },
    { "ANGLE LIMIT",      OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_launchControlAngleLimit,      0,  80, 1 }  },
    { "ITERM GAIN",       OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_launchControlGain,            0, 200, 1 }  },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuLaunchControl = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "LAUNCH",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_launchControlOnEnter,
    .onExit = cmsx_launchControlOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuLaunchControlEntries,
};
#endif

static uint8_t  cmsx_angleStrength;
static uint8_t  cmsx_horizonStrength;
static uint8_t  cmsx_horizonTransition;
static uint8_t  cmsx_levelAngleLimit;
static uint8_t  cmsx_throttleBoost;
static uint8_t  cmsx_thrustLinearization;
static uint8_t  cmsx_antiGravityGain;
static uint8_t  cmsx_motorOutputLimit;
static int8_t   cmsx_autoProfileCellCount;
#ifdef USE_D_MIN
static uint8_t  cmsx_d_min[XYZ_AXIS_COUNT];
static uint8_t  cmsx_d_min_gain;
static uint8_t  cmsx_d_min_advance;
#endif

#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
static uint8_t  cmsx_vbat_sag_compensation;
#endif

#ifdef USE_ITERM_RELAX
static uint8_t cmsx_iterm_relax;
static uint8_t cmsx_iterm_relax_type;
static uint8_t cmsx_iterm_relax_cutoff;
#endif

#ifdef USE_FEEDFORWARD
static uint8_t cmsx_feedforward_transition;
static uint8_t cmsx_feedforward_boost;
static uint8_t cmsx_feedforward_averaging;
static uint8_t cmsx_feedforward_smooth_factor;
static uint8_t cmsx_feedforward_jitter_factor;
#endif

static uint8_t cmsx_tpa_rate;
static uint16_t cmsx_tpa_breakpoint;

static const void *cmsx_profileOtherOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, currentPidProfile->profileName);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_angleStrength =     pidProfile->pid[PID_LEVEL].P;
    cmsx_horizonStrength =   pidProfile->pid[PID_LEVEL].I;
    cmsx_horizonTransition = pidProfile->pid[PID_LEVEL].D;
    cmsx_levelAngleLimit =   pidProfile->levelAngleLimit;

    cmsx_antiGravityGain   = pidProfile->anti_gravity_gain;

    cmsx_throttleBoost = pidProfile->throttle_boost;
    cmsx_thrustLinearization = pidProfile->thrustLinearization;
    cmsx_motorOutputLimit = pidProfile->motor_output_limit;
    cmsx_autoProfileCellCount = pidProfile->auto_profile_cell_count;

#ifdef USE_D_MIN
    for (unsigned i = 0; i < XYZ_AXIS_COUNT; i++) {
        cmsx_d_min[i]  = pidProfile->d_min[i];
    }
    cmsx_d_min_gain = pidProfile->d_min_gain;
    cmsx_d_min_advance = pidProfile->d_min_advance;
#endif

#ifdef USE_ITERM_RELAX
    cmsx_iterm_relax = pidProfile->iterm_relax;
    cmsx_iterm_relax_type = pidProfile->iterm_relax_type;
    cmsx_iterm_relax_cutoff = pidProfile->iterm_relax_cutoff;
#endif

#ifdef USE_FEEDFORWARD
    cmsx_feedforward_transition  = pidProfile->feedforward_transition;
    cmsx_feedforward_averaging = pidProfile->feedforward_averaging;
    cmsx_feedforward_boost = pidProfile->feedforward_boost;
    cmsx_feedforward_smooth_factor = pidProfile->feedforward_smooth_factor;
    cmsx_feedforward_jitter_factor = pidProfile->feedforward_jitter_factor;
#endif

#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
    cmsx_vbat_sag_compensation = pidProfile->vbat_sag_compensation;
#endif
    cmsx_tpa_rate = pidProfile->tpa_rate;
    cmsx_tpa_breakpoint = pidProfile->tpa_breakpoint;

    return NULL;
}

static const void *cmsx_profileOtherOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
    pidInitConfig(currentPidProfile);

    pidProfile->pid[PID_LEVEL].P = cmsx_angleStrength;
    pidProfile->pid[PID_LEVEL].I = cmsx_horizonStrength;
    pidProfile->pid[PID_LEVEL].D = cmsx_horizonTransition;
    pidProfile->levelAngleLimit  = cmsx_levelAngleLimit;

    pidProfile->anti_gravity_gain   = cmsx_antiGravityGain;

    pidProfile->throttle_boost = cmsx_throttleBoost;
    pidProfile->thrustLinearization = cmsx_thrustLinearization;
    pidProfile->motor_output_limit = cmsx_motorOutputLimit;
    pidProfile->auto_profile_cell_count = cmsx_autoProfileCellCount;

#ifdef USE_D_MIN
    for (unsigned i = 0; i < XYZ_AXIS_COUNT; i++) {
        pidProfile->d_min[i] = cmsx_d_min[i];
    }
    pidProfile->d_min_gain = cmsx_d_min_gain;
    pidProfile->d_min_advance = cmsx_d_min_advance;
#endif

#ifdef USE_ITERM_RELAX
    pidProfile->iterm_relax = cmsx_iterm_relax;
    pidProfile->iterm_relax_type = cmsx_iterm_relax_type;
    pidProfile->iterm_relax_cutoff = cmsx_iterm_relax_cutoff;
#endif

#ifdef USE_FEEDFORWARD
    pidProfile->feedforward_transition = cmsx_feedforward_transition;
    pidProfile->feedforward_averaging = cmsx_feedforward_averaging;
    pidProfile->feedforward_boost = cmsx_feedforward_boost;
    pidProfile->feedforward_smooth_factor = cmsx_feedforward_smooth_factor;
    pidProfile->feedforward_jitter_factor = cmsx_feedforward_jitter_factor;
#endif

#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
    pidProfile->vbat_sag_compensation = cmsx_vbat_sag_compensation;
#endif
    pidProfile->tpa_rate = cmsx_tpa_rate;
    pidProfile->tpa_breakpoint = cmsx_tpa_breakpoint;

    initEscEndpoints();
    return NULL;
}

static const OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, pidProfileIndexString },

#ifdef USE_FEEDFORWARD
    { "FF TRANSITION", OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_feedforward_transition,        0,    100,   1, 10 } },
    { "FF AVERAGING",  OME_TAB,    NULL, &(OSD_TAB_t)    { &cmsx_feedforward_averaging,         4, lookupTableFeedforwardAveraging} },
    { "FF SMOOTHNESS", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_feedforward_smooth_factor,     0,     75,   1  }    },
    { "FF JITTER",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_feedforward_jitter_factor,     0,     20,   1  }    },
    { "FF BOOST",      OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_feedforward_boost,             0,     50,   1  }    },
#endif
    { "ANGLE STR",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_angleStrength,          0,    200,   1  }    },
    { "HORZN STR",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonStrength,        0,    200,   1  }    },
    { "HORZN TRS",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonTransition,      0,    200,   1  }    },
    { "ANGLE LIMIT", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_levelAngleLimit,        10,    90,   1  }    },

    { "AG GAIN",     OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &cmsx_antiGravityGain,   ITERM_ACCELERATOR_GAIN_OFF, ITERM_ACCELERATOR_GAIN_MAX, 1, 100 }    },
#ifdef USE_THROTTLE_BOOST
    { "THR BOOST",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_throttleBoost,          0,    100,   1  }    },
#endif
#ifdef USE_THRUST_LINEARIZATION
    { "THR LINEAR",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_thrustLinearization,    0,    150,   1  }    },
#endif
#ifdef USE_ITERM_RELAX
    { "I_RELAX",         OME_TAB,    NULL, &(OSD_TAB_t)     { &cmsx_iterm_relax,        ITERM_RELAX_COUNT - 1,      lookupTableItermRelax       } },
    { "I_RELAX TYPE",    OME_TAB,    NULL, &(OSD_TAB_t)     { &cmsx_iterm_relax_type,   ITERM_RELAX_TYPE_COUNT - 1, lookupTableItermRelaxType   } },
    { "I_RELAX CUTOFF",  OME_UINT8,  NULL, &(OSD_UINT8_t)   { &cmsx_iterm_relax_cutoff, 1, 50, 1 } },
#endif
#ifdef USE_LAUNCH_CONTROL
    {"LAUNCH CONTROL", OME_Submenu, cmsMenuChange, &cmsx_menuLaunchControl },
#endif
    { "MTR OUT LIM %",OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_motorOutputLimit, MOTOR_OUTPUT_LIMIT_PERCENT_MIN,  MOTOR_OUTPUT_LIMIT_PERCENT_MAX,  1} },

    { "AUTO CELL CNT", OME_INT8, NULL, &(OSD_INT8_t) { &cmsx_autoProfileCellCount, AUTO_PROFILE_CELL_COUNT_CHANGE, MAX_AUTO_DETECT_CELL_COUNT, 1} },

#ifdef USE_D_MIN
    { "D_MIN ROLL",  OME_UINT8 | SLIDER_RP,  NULL, &(OSD_UINT8_t) { &cmsx_d_min[FD_ROLL],      0, 100, 1 } },
    { "D_MIN PITCH", OME_UINT8 | SLIDER_RP,  NULL, &(OSD_UINT8_t) { &cmsx_d_min[FD_PITCH],     0, 100, 1 } },
    { "D_MIN YAW",   OME_UINT8 | SLIDER_RPY,  NULL, &(OSD_UINT8_t) { &cmsx_d_min[FD_YAW],       0, 100, 1 } },
    { "D_MIN GAIN",  OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min_gain,          0, 100, 1 } },
    { "D_MIN ADV",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_d_min_advance,       0, 200, 1 } },
#endif

#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
    { "VBAT_SAG_COMP", OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_vbat_sag_compensation, 0, 150, 1 } },
#endif

    { "TPA RATE",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &cmsx_tpa_rate,          0,  100,  1, 10} },
    { "TPA BRKPT",   OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_tpa_breakpoint, 1000, 2000, 10} },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuProfileOther = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFOTHER",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileOtherOnEnter,
    .onExit = cmsx_profileOtherOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileOtherEntries,
};


static uint16_t gyroConfig_gyro_lpf1_static_hz;
static uint16_t gyroConfig_gyro_lpf2_static_hz;
static uint16_t gyroConfig_gyro_soft_notch_hz_1;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_1;
static uint16_t gyroConfig_gyro_soft_notch_hz_2;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_2;
static uint8_t  gyroConfig_gyro_to_use;

static const void *cmsx_menuGyro_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    gyroConfig_gyro_lpf1_static_hz =  gyroConfig()->gyro_lpf1_static_hz;
    gyroConfig_gyro_lpf2_static_hz =  gyroConfig()->gyro_lpf2_static_hz;
    gyroConfig_gyro_soft_notch_hz_1 = gyroConfig()->gyro_soft_notch_hz_1;
    gyroConfig_gyro_soft_notch_cutoff_1 = gyroConfig()->gyro_soft_notch_cutoff_1;
    gyroConfig_gyro_soft_notch_hz_2 = gyroConfig()->gyro_soft_notch_hz_2;
    gyroConfig_gyro_soft_notch_cutoff_2 = gyroConfig()->gyro_soft_notch_cutoff_2;
    gyroConfig_gyro_to_use = gyroConfig()->gyro_to_use;

    return NULL;
}

static const void *cmsx_menuGyro_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    gyroConfigMutable()->gyro_lpf1_static_hz =  gyroConfig_gyro_lpf1_static_hz;
    gyroConfigMutable()->gyro_lpf2_static_hz =  gyroConfig_gyro_lpf2_static_hz;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = gyroConfig_gyro_soft_notch_hz_1;
    gyroConfigMutable()->gyro_soft_notch_cutoff_1 = gyroConfig_gyro_soft_notch_cutoff_1;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = gyroConfig_gyro_soft_notch_hz_2;
    gyroConfigMutable()->gyro_soft_notch_cutoff_2 = gyroConfig_gyro_soft_notch_cutoff_2;
    gyroConfigMutable()->gyro_to_use = gyroConfig_gyro_to_use;

    return NULL;
}

static const OSD_Entry cmsx_menuFilterGlobalEntries[] =
{
    { "-- FILTER GLB  --", OME_Label, NULL, NULL },

    { "GYRO LPF1",  OME_UINT16 | SLIDER_GYRO, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lpf1_static_hz, 0, LPF_MAX_HZ, 1 } },
#ifdef USE_GYRO_LPF2
    { "GYRO LPF2",  OME_UINT16 | SLIDER_GYRO, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lpf2_static_hz,  0, LPF_MAX_HZ, 1 } },
#endif
    { "GYRO NF1",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_1,     0, 500, 1 } },
    { "GYRO NF1C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_1, 0, 500, 1 } },
    { "GYRO NF2",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_2,     0, 500, 1 } },
    { "GYRO NF2C",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_2, 0, 500, 1 } },
#ifdef USE_MULTI_GYRO
    { "GYRO TO USE",  OME_TAB | REBOOT_REQUIRED,  NULL, &(OSD_TAB_t)    { &gyroConfig_gyro_to_use,  2, osdTableGyroToUse} },
#endif

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuFilterGlobal = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTGLB",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuGyro_onEnter,
    .onExit = cmsx_menuGyro_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFilterGlobalEntries,
};

#if (defined(USE_DYN_NOTCH_FILTER) || defined(USE_DYN_LPF)) && defined(USE_EXTENDED_CMS_MENUS)

#ifdef USE_DYN_NOTCH_FILTER
static uint16_t dynFiltNotchMaxHz;
static uint8_t  dynFiltNotchCount;
static uint16_t dynFiltNotchQ;
static uint16_t dynFiltNotchMinHz;
#endif
#ifdef USE_DYN_LPF
static uint16_t gyroLpfDynMin;
static uint16_t gyroLpfDynMax;
static uint8_t gyroLpfDynExpo;
static uint16_t dtermLpfDynMin;
static uint16_t dtermLpfDynMax;
static uint8_t dtermLpfDynExpo;
#endif

static const void *cmsx_menuDynFilt_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

#ifdef USE_DYN_NOTCH_FILTER
    dynFiltNotchMaxHz   = dynNotchConfig()->dyn_notch_max_hz;
    dynFiltNotchCount   = dynNotchConfig()->dyn_notch_count;
    dynFiltNotchQ       = dynNotchConfig()->dyn_notch_q;
    dynFiltNotchMinHz   = dynNotchConfig()->dyn_notch_min_hz;
#endif
#ifdef USE_DYN_LPF
    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    gyroLpfDynMin   = gyroConfig()->gyro_lpf1_dyn_min_hz;
    gyroLpfDynMax   = gyroConfig()->gyro_lpf1_dyn_max_hz;
    gyroLpfDynExpo  = gyroConfig()->gyro_lpf1_dyn_expo;
    dtermLpfDynMin  = pidProfile->dterm_lpf1_dyn_min_hz;
    dtermLpfDynMax  = pidProfile->dterm_lpf1_dyn_max_hz;
    dtermLpfDynExpo = pidProfile->dterm_lpf1_dyn_expo;
#endif

    return NULL;
}

static const void *cmsx_menuDynFilt_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

#ifdef USE_DYN_NOTCH_FILTER
    dynNotchConfigMutable()->dyn_notch_max_hz        = dynFiltNotchMaxHz;
    dynNotchConfigMutable()->dyn_notch_count         = dynFiltNotchCount;
    dynNotchConfigMutable()->dyn_notch_q             = dynFiltNotchQ;
    dynNotchConfigMutable()->dyn_notch_min_hz        = dynFiltNotchMinHz;
#endif
#ifdef USE_DYN_LPF
    pidProfile_t *pidProfile = currentPidProfile;
    gyroConfigMutable()->gyro_lpf1_dyn_min_hz = gyroLpfDynMin;
    gyroConfigMutable()->gyro_lpf1_dyn_max_hz = gyroLpfDynMax;
    gyroConfigMutable()->gyro_lpf1_dyn_expo   = gyroLpfDynExpo;
    pidProfile->dterm_lpf1_dyn_min_hz         = dtermLpfDynMin;
    pidProfile->dterm_lpf1_dyn_max_hz         = dtermLpfDynMax;
    pidProfile->dterm_lpf1_dyn_expo           = dtermLpfDynExpo;
#endif

    return NULL;
}

static const OSD_Entry cmsx_menuDynFiltEntries[] =
{
    { "-- DYN FILT --", OME_Label, NULL, NULL },

#ifdef USE_DYN_NOTCH_FILTER
    { "NOTCH COUNT",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &dynFiltNotchCount,   0, DYN_NOTCH_COUNT_MAX, 1 } },
    { "NOTCH Q",        OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchQ,       1, 1000, 1 } },
    { "NOTCH MIN HZ",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchMinHz,   20, 250, 1 } },
    { "NOTCH MAX HZ",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchMaxHz,   200, 1000, 1 } },
#endif

#ifdef USE_DYN_LPF
    { "GYRO DLPF MIN",   OME_UINT16 | SLIDER_GYRO, NULL, &(OSD_UINT16_t) { &gyroLpfDynMin,  0, 1000, 1 } },
    { "GYRO DLPF MAX",   OME_UINT16 | SLIDER_GYRO, NULL, &(OSD_UINT16_t) { &gyroLpfDynMax,  0, 1000, 1 } },
    { "GYRO DLPF EXPO",  OME_UINT8, NULL, &(OSD_UINT8_t) { &gyroLpfDynExpo,   0, 10, 1 } },
    { "DTERM DLPF MIN",  OME_UINT16 | SLIDER_DTERM, NULL, &(OSD_UINT16_t) { &dtermLpfDynMin, 0, 1000, 1 } },
    { "DTERM DLPF MAX",  OME_UINT16 | SLIDER_DTERM, NULL, &(OSD_UINT16_t) { &dtermLpfDynMax, 0, 1000, 1 } },
    { "DTERM DLPF EXPO", OME_UINT8, NULL, &(OSD_UINT8_t) { &dtermLpfDynExpo,  0, 10, 1 } },
#endif

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuDynFilt = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XDYNFLT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuDynFilt_onEnter,
    .onExit = cmsx_menuDynFilt_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuDynFiltEntries,
};

#endif

static uint16_t cmsx_dterm_lpf1_static_hz;
static uint16_t cmsx_dterm_lpf2_static_hz;
static uint16_t cmsx_dterm_notch_hz;
static uint16_t cmsx_dterm_notch_cutoff;
static uint16_t cmsx_yaw_lowpass_hz;

static const void *cmsx_FilterPerProfileRead(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_dterm_lpf1_static_hz   = pidProfile->dterm_lpf1_static_hz;
    cmsx_dterm_lpf2_static_hz   = pidProfile->dterm_lpf2_static_hz;
    cmsx_dterm_notch_hz         = pidProfile->dterm_notch_hz;
    cmsx_dterm_notch_cutoff     = pidProfile->dterm_notch_cutoff;
    cmsx_yaw_lowpass_hz         = pidProfile->yaw_lowpass_hz;

    return NULL;
}

static const void *cmsx_FilterPerProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;

    pidProfile->dterm_lpf1_static_hz  = cmsx_dterm_lpf1_static_hz;
    pidProfile->dterm_lpf2_static_hz  = cmsx_dterm_lpf2_static_hz;
    pidProfile->dterm_notch_hz        = cmsx_dterm_notch_hz;
    pidProfile->dterm_notch_cutoff    = cmsx_dterm_notch_cutoff;
    pidProfile->yaw_lowpass_hz        = cmsx_yaw_lowpass_hz;

    return NULL;
}

static const OSD_Entry cmsx_menuFilterPerProfileEntries[] =
{
    { "-- FILTER PP  --", OME_Label, NULL, NULL },

    { "DTERM LPF1", OME_UINT16 | SLIDER_DTERM, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lpf1_static_hz, 0, LPF_MAX_HZ, 1 } },
    { "DTERM LPF2", OME_UINT16 | SLIDER_DTERM, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lpf2_static_hz, 0, LPF_MAX_HZ, 1 } },
    { "DTERM NF",   OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_notch_hz,       0, LPF_MAX_HZ, 1 } },
    { "DTERM NFCO", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_notch_cutoff,   0, LPF_MAX_HZ, 1 } },
    { "YAW LPF",    OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_yaw_lowpass_hz,       0, 500, 1 } },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuFilterPerProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTPP",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_FilterPerProfileRead,
    .onExit = cmsx_FilterPerProfileWriteback,
    .onDisplayUpdate = NULL,
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

static const void *cmsx_menuCopyProfile_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_dstPidProfile = 0;
    cmsx_dstControlRateProfile = 0;

    return NULL;
}

static const void *cmsx_CopyPidProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstPidProfile > 0) {
        pidCopyProfile(cmsx_dstPidProfile - 1, getCurrentPidProfileIndex());
    }

    return NULL;
}

static const void *cmsx_CopyControlRateProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstControlRateProfile > 0) {
        copyControlRateProfile(cmsx_dstControlRateProfile - 1, getCurrentControlRateProfileIndex());
    }

    return NULL;
}

static const OSD_Entry cmsx_menuCopyProfileEntries[] =
{
    { "-- COPY PROFILE --", OME_Label, NULL, NULL},

    { "CPY PID PROF TO",   OME_TAB,      NULL,                        &cmsx_PidProfileTable },
    { "COPY PP",           OME_Funcall,  cmsx_CopyPidProfile,         NULL },
    { "CPY RATE PROF TO",  OME_TAB,      NULL,                        &cmsx_ControlRateProfileTable },
    { "COPY RP",           OME_Funcall,  cmsx_CopyControlRateProfile, NULL },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuCopyProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XCPY",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuCopyProfile_onEnter,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuCopyProfileEntries,
};

#endif

static const OSD_Entry cmsx_menuImuEntries[] =
{
    { "-- PROFILE --", OME_Label, NULL, NULL},

    {"PID PROF",  OME_TAB,     cmsx_profileIndexOnChange,     &(OSD_TAB_t){&tmpPidProfileIndex, PID_PROFILE_COUNT-1, pidProfileNamePtrs}},
    {"PID",       OME_Submenu, cmsMenuChange,                 &cmsx_menuPid},
#ifdef USE_SIMPLIFIED_TUNING
    {"SIMPLIFIED TUNING",   OME_Submenu, cmsMenuChange,                 &cmsx_menuSimplifiedTuning},
#endif
    {"MISC PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther},
    {"FILT PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterPerProfile},

    {"RATE PROF", OME_TAB,   cmsx_rateProfileIndexOnChange, &(OSD_TAB_t){&tmpRateProfileIndex, CONTROL_RATE_PROFILE_COUNT-1, rateProfileNamePtrs}},
    {"RATE",      OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile},

    {"FILT GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal},
#if  (defined(USE_DYN_NOTCH_FILTER) || defined(USE_DYN_LPF)) && defined(USE_EXTENDED_CMS_MENUS)
    {"DYN FILT",  OME_Submenu, cmsMenuChange,                 &cmsx_menuDynFilt},
#endif

#ifdef USE_EXTENDED_CMS_MENUS
    {"COPY PROF", OME_Submenu, cmsMenuChange,                 &cmsx_menuCopyProfile},
#endif /* USE_EXTENDED_CMS_MENUS */

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuImu = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XIMU",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuImu_onEnter,
    .onExit = cmsx_menuImu_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuImuEntries,
};

#endif // CMS
