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

//#include "common/typeconversion.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

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
static char profileIndexString[] = " PROF n";
static uint8_t tempPid[4][3];

static long cmsx_menuImu_onEnter(void)
{
    profileIndex = masterConfig.current_profile_index;
    tmpProfileIndex = profileIndex + 1;
    profileIndexString[6] = '0' + tmpProfileIndex;

    return 0;
}

static long cmsx_menuImu_onExit(OSD_Entry *self)
{
    UNUSED(self);

    masterConfig.current_profile_index = tmpProfileIndex - 1;

    return 0;
}

static long cmsx_PidRead(void)
{

    for (uint8_t i = 0; i < 3; i++) {
        tempPid[i][0] = masterConfig.profile[profileIndex].pidProfile.P8[i];
        tempPid[i][1] = masterConfig.profile[profileIndex].pidProfile.I8[i];
        tempPid[i][2] = masterConfig.profile[profileIndex].pidProfile.D8[i];
    }
    tempPid[3][0] = masterConfig.profile[profileIndex].pidProfile.P8[PIDLEVEL];
    tempPid[3][1] = masterConfig.profile[profileIndex].pidProfile.I8[PIDLEVEL];
    tempPid[3][2] = masterConfig.profile[profileIndex].pidProfile.D8[PIDLEVEL];

    return 0;
}

static long cmsx_PidOnEnter(void)
{
    profileIndexString[6] = '0' + tmpProfileIndex;
    cmsx_PidRead();

    return 0;
}


static long cmsx_PidWriteback(OSD_Entry *self)
{
    UNUSED(self);

    for (uint8_t i = 0; i < 3; i++) {
        masterConfig.profile[profileIndex].pidProfile.P8[i] = tempPid[i][0];
        masterConfig.profile[profileIndex].pidProfile.I8[i] = tempPid[i][1];
        masterConfig.profile[profileIndex].pidProfile.D8[i] = tempPid[i][2];
    }

    masterConfig.profile[profileIndex].pidProfile.P8[PIDLEVEL] = tempPid[3][0];
    masterConfig.profile[profileIndex].pidProfile.I8[PIDLEVEL] = tempPid[3][1];
    masterConfig.profile[profileIndex].pidProfile.D8[PIDLEVEL] = tempPid[3][2];

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
    "XPID",
    OME_MENU,
    cmsx_PidOnEnter,
    cmsx_PidWriteback,
    NULL,
    cmsx_menuPidEntries,
};

//
// Rate & Expo
//
static controlRateConfig_t rateProfile;

static long cmsx_RateExpoRead(void)
{
    memcpy(&rateProfile, &masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], sizeof(controlRateConfig_t));

    return 0;
}

static long cmsx_RateExpoWriteback(OSD_Entry *self)
{
    UNUSED(self);

    memcpy(&masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], &rateProfile, sizeof(controlRateConfig_t));

    return 0;
}

static long cmsx_menuRcConfirmBack(OSD_Entry *self)
{
    if (self && self->type == OME_Back)
        return 0;
    else
        return -1;
}

static OSD_Entry cmsx_menuRateExpoEntries[] =
{
    { "-- RATE&EXPO --", OME_Label, NULL, NULL, 0 },

    { "RC RATE",     OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRate8,    0, 255, 1, 10 }, 0 },
    { "RC YAW RATE", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcYawRate8, 0, 255, 1, 10 }, 0 },

    { "ROLL SUPER",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[0],   0, 100, 1, 10 }, 0 },
    { "PITCH SUPER", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[1],   0, 100, 1, 10 }, 0 },
    { "YAW SUPER",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[2],   0, 100, 1, 10 }, 0 },

    { "RC EXPO",     OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo8,    0, 100, 1, 10 }, 0 },
    { "RC YAW EXP",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcYawExpo8, 0, 100, 1, 10 }, 0 },

    { "THRPID ATT",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.dynThrPID,  0, 100, 1, 10}, 0 },
    { "TPA BRKPT",   OME_UINT16, NULL, &(OSD_UINT16_t){ &rateProfile.tpa_breakpoint, 1000, 2000, 10}, 0 },
    { "D SETPT WT",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &masterConfig.profile[0].pidProfile.dtermSetpointWeight, 0, 255, 1, 10 }, 0 },
    { "SETPT RLX",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &masterConfig.profile[0].pidProfile.setpointRelaxRatio,  0, 100, 1, 10 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuRateExpo = {
    "MENURATE",
    OME_MENU,
    cmsx_RateExpoRead,
    cmsx_RateExpoWriteback,
    NULL,
    cmsx_menuRateExpoEntries,
};


//
// RC preview
//
static OSD_Entry cmsx_menuRcEntries[] =
{
    { "-- RC PREV --", OME_Label, NULL, NULL, 0},

    { "ROLL",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[ROLL],     1, 2500, 0 }, DYNAMIC },
    { "PITCH", OME_INT16, NULL, &(OSD_INT16_t){ &rcData[PITCH],    1, 2500, 0 }, DYNAMIC },
    { "THR",   OME_INT16, NULL, &(OSD_INT16_t){ &rcData[THROTTLE], 1, 2500, 0 }, DYNAMIC },
    { "YAW",   OME_INT16, NULL, &(OSD_INT16_t){ &rcData[YAW],      1, 2500, 0 }, DYNAMIC },

    { "AUX1",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX1],     1, 2500, 0 }, DYNAMIC },
    { "AUX2",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX2],     1, 2500, 0 }, DYNAMIC },
    { "AUX3",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX3],     1, 2500, 0 }, DYNAMIC },
    { "AUX4",  OME_INT16, NULL, &(OSD_INT16_t){ &rcData[AUX4],     1, 2500, 0 }, DYNAMIC },

    { "BACK",  OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuRcPreview = {
    "XRCPREV",
    OME_MENU,
    NULL,
    cmsx_menuRcConfirmBack,
    NULL,
    cmsx_menuRcEntries,
};


//
// Misc
//
static OSD_Entry menuImuMiscEntries[]=
{
    { "-- MISC --", OME_Label, NULL, NULL, 0 },

    { "GYRO LPF",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &masterConfig.gyro_soft_lpf_hz,                    0, 255, 1 }, 0 },
    { "DTERM LPF",  OME_UINT16, NULL, &(OSD_UINT16_t){ &masterConfig.profile[0].pidProfile.dterm_lpf_hz,  0, 500, 1 }, 0 },
    { "YAW LPF",    OME_UINT16, NULL, &(OSD_UINT16_t){ &masterConfig.profile[0].pidProfile.yaw_lpf_hz,    0, 500, 1 }, 0 },
    { "YAW P LIM",  OME_UINT16, NULL, &(OSD_UINT16_t){ &masterConfig.profile[0].pidProfile.yaw_p_limit, 100, 500, 1 }, 0 },
    { "MIN THR",    OME_UINT16, NULL, &(OSD_UINT16_t){ &masterConfig.motorConfig.minthrottle,         1000, 2000, 1 }, 0 },
    { "VBAT SCALE", OME_UINT8,  NULL, &(OSD_UINT8_t) { &masterConfig.batteryConfig.vbatscale,             1, 250, 1 }, 0 },
    { "VBAT CLMAX", OME_UINT8,  NULL, &(OSD_UINT8_t) { &masterConfig.batteryConfig.vbatmaxcellvoltage,   10,  50, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0},
    { NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu menuImuMisc = {
    "XIMUMISC",
    OME_MENU,
    NULL,
    NULL,
    NULL,
    menuImuMiscEntries,
};

static long onProfileChange(displayPort_t *pDisplay, void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    masterConfig.current_profile_index = tmpProfileIndex - 1;

    return 0;
}

static OSD_Entry cmsx_menuImuEntries[] =
{
    { "-- IMU --", OME_Label, NULL, NULL, 0},

    {"PID PROF",  OME_UINT8,   onProfileChange, &(OSD_UINT8_t){ &tmpProfileIndex, 1, MAX_PROFILE_COUNT, 1}, 0},
    {"PID",       OME_Submenu, cmsMenuChange,   &cmsx_menuPid,       0},
    {"RATE&RXPO", OME_Submenu, cmsMenuChange,   &cmsx_menuRateExpo,  0},
    {"RC PREV",   OME_Submenu, cmsMenuChange,   &cmsx_menuRcPreview, 0},
    {"MISC",      OME_Submenu, cmsMenuChange,   &menuImuMisc,        0},

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuImu = {
    "XIMU",
    OME_MENU,
    cmsx_menuImu_onEnter,
    cmsx_menuImu_onExit,
    NULL,
    cmsx_menuImuEntries,
};
#endif // CMS
