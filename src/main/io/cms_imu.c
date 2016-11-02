
// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "build/version.h"

#ifdef CMS

#include "drivers/system.h"

//#include "common/typeconversion.h"

#include "io/cms.h"
#include "io/cms_types.h"
#include "io/cms_imu.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

OSD_UINT8_t entryPidProfile = {&masterConfig.current_profile_index, 0, MAX_PROFILE_COUNT, 1};

uint8_t tempPid[4][3];

static OSD_UINT8_t entryRollP = {&tempPid[PIDROLL][0], 10, 150, 1};
static OSD_UINT8_t entryRollI = {&tempPid[PIDROLL][1], 1, 150, 1};
static OSD_UINT8_t entryRollD = {&tempPid[PIDROLL][2], 0, 150, 1};

static OSD_UINT8_t entryPitchP = {&tempPid[PIDPITCH][0], 10, 150, 1};
static OSD_UINT8_t entryPitchI = {&tempPid[PIDPITCH][1], 1, 150, 1};
static OSD_UINT8_t entryPitchD = {&tempPid[PIDPITCH][2], 0, 150, 1};

static OSD_UINT8_t entryYawP = {&tempPid[PIDYAW][0], 10, 150, 1};
static OSD_UINT8_t entryYawI = {&tempPid[PIDYAW][1], 1, 150, 1};
static OSD_UINT8_t entryYawD = {&tempPid[PIDYAW][2], 0, 150, 1};

void cmsx_PidRead(void)
{
    uint8_t i;

    for (i = 0; i < 3; i++) {
        tempPid[i][0] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.P8[i];
        tempPid[i][1] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.I8[i];
        tempPid[i][2] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.D8[i];
    }
    tempPid[3][0] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.P8[PIDLEVEL];
    tempPid[3][1] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.I8[PIDLEVEL];
    tempPid[3][2] = masterConfig.profile[masterConfig.current_profile_index].pidProfile.D8[PIDLEVEL];
}

void cmsx_PidWriteback(void)
{
    uint8_t i;

    for (i = 0; i < 3; i++) {
        masterConfig.profile[masterConfig.current_profile_index].pidProfile.P8[i] = tempPid[i][0];
        masterConfig.profile[masterConfig.current_profile_index].pidProfile.I8[i] = tempPid[i][1];
        masterConfig.profile[masterConfig.current_profile_index].pidProfile.D8[i] = tempPid[i][2];
    }

    masterConfig.profile[masterConfig.current_profile_index].pidProfile.P8[PIDLEVEL] = tempPid[3][0];
    masterConfig.profile[masterConfig.current_profile_index].pidProfile.I8[PIDLEVEL] = tempPid[3][1];
    masterConfig.profile[masterConfig.current_profile_index].pidProfile.D8[PIDLEVEL] = tempPid[3][2];
}

OSD_Entry cmsx_menuPid[] =
{
    {"--- PID ---", OME_Label, NULL, NULL, 0},
    {"ROLL P", OME_UINT8, NULL, &entryRollP, 0},
    {"ROLL I", OME_UINT8, NULL, &entryRollI, 0},
    {"ROLL D", OME_UINT8, NULL, &entryRollD, 0},

    {"PITCH P", OME_UINT8, NULL, &entryPitchP, 0},
    {"PITCH I", OME_UINT8, NULL, &entryPitchI, 0},
    {"PITCH D", OME_UINT8, NULL, &entryPitchD, 0},

    {"YAW P", OME_UINT8, NULL, &entryYawP, 0},
    {"YAW I", OME_UINT8, NULL, &entryYawI, 0},
    {"YAW D", OME_UINT8, NULL, &entryYawD, 0},

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

//
// Rate & Expo
//
controlRateConfig_t rateProfile;

static OSD_FLOAT_t entryRollRate = {&rateProfile.rates[0], 0, 250, 1, 10};
static OSD_FLOAT_t entryPitchRate = {&rateProfile.rates[1], 0, 250, 1, 10};
static OSD_FLOAT_t entryYawRate = {&rateProfile.rates[2], 0, 250, 1, 10};
static OSD_FLOAT_t entryRcRate = {&rateProfile.rcRate8, 0, 200, 1, 10};
static OSD_FLOAT_t entryRcYawRate = {&rateProfile.rcYawRate8, 0, 200, 1, 10};
static OSD_FLOAT_t entryRcExpo = {&rateProfile.rcExpo8, 0, 100, 1, 10};
static OSD_FLOAT_t entryRcExpoYaw = {&rateProfile.rcYawExpo8, 0, 100, 1, 10};
static OSD_FLOAT_t extryTpaEntry = {&rateProfile.dynThrPID, 0, 70, 1, 10};
static OSD_UINT16_t entryTpaBreak = {&rateProfile.tpa_breakpoint, 1100, 1800, 10};
static OSD_FLOAT_t entryPSetpoint = {&masterConfig.profile[0].pidProfile.setpointRelaxRatio, 0, 100, 1, 10};
static OSD_FLOAT_t entryDSetpoint = {&masterConfig.profile[0].pidProfile.dtermSetpointWeight, 0, 255, 1, 10};

void cmsx_RateExpoRead()
{
    memcpy(&rateProfile, &masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], sizeof(controlRateConfig_t));
}

void cmsx_RateExpoWriteback()
{
    memcpy(&masterConfig.profile[masterConfig.current_profile_index].controlRateProfile[masterConfig.profile[masterConfig.current_profile_index].activeRateProfile], &rateProfile, sizeof(controlRateConfig_t));
}

OSD_Entry cmsx_menuRateExpo[] =
{
    {"--- RATE&EXPO ---", OME_Label, NULL, NULL, 0},
    {"RC RATE", OME_FLOAT, NULL, &entryRcYawRate, 0},
    {"RC YAW RATE", OME_FLOAT, NULL, &entryRcRate, 0},
    {"ROLL SUPER", OME_FLOAT, NULL, &entryRollRate, 0},
    {"PITCH SUPER", OME_FLOAT, NULL, &entryPitchRate, 0},
    {"YAW SUPER", OME_FLOAT, NULL, &entryYawRate, 0},
    {"RC EXPO", OME_FLOAT, NULL, &entryRcExpo, 0},
    {"RC YAW EXPO", OME_FLOAT, NULL, &entryRcExpoYaw, 0},
    {"THR PID ATT", OME_FLOAT, NULL, &extryTpaEntry, 0},
    {"TPA BRKPT", OME_UINT16, NULL, &entryTpaBreak, 0},
    {"D SETPT", OME_FLOAT, NULL, &entryDSetpoint, 0},
    {"D SETPT TRN", OME_FLOAT, NULL, &entryPSetpoint, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

//
// RC preview
//
static OSD_INT16_t entryRcRoll = {&rcData[ROLL], 1, 2500, 0};
static OSD_INT16_t entryRcPitch = {&rcData[PITCH], 1, 2500, 0};
static OSD_INT16_t entryRcThr = {&rcData[THROTTLE], 1, 2500, 0};
static OSD_INT16_t entryRcYaw = {&rcData[YAW], 1, 2500, 0};
static OSD_INT16_t entryRcAux1 = {&rcData[AUX1], 1, 2500, 0};
static OSD_INT16_t entryRcAux2 = {&rcData[AUX2], 1, 2500, 0};
static OSD_INT16_t entryRcAux3 = {&rcData[AUX3], 1, 2500, 0};
static OSD_INT16_t entryRcAux4 = {&rcData[AUX4], 1, 2500, 0};

OSD_Entry cmsx_menuRc[] =
{
    {"--- RC PREV ---", OME_Label, NULL, NULL, 0},
    {"ROLL", OME_Poll_INT16, NULL, &entryRcRoll, 0},
    {"PITCH", OME_Poll_INT16, NULL, &entryRcPitch, 0},
    {"THR", OME_Poll_INT16, NULL, &entryRcThr, 0},
    {"YAW", OME_Poll_INT16, NULL, &entryRcYaw, 0},
    {"AUX1", OME_Poll_INT16, NULL, &entryRcAux1, 0},
    {"AUX2", OME_Poll_INT16, NULL, &entryRcAux2, 0},
    {"AUX3", OME_Poll_INT16, NULL, &entryRcAux3, 0},
    {"AUX4", OME_Poll_INT16, NULL, &entryRcAux4, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

//
// Misc
//
OSD_UINT16_t entryMinThrottle = {&masterConfig.motorConfig.minthrottle, 1020, 1300, 10};
OSD_UINT8_t entryGyroSoftLpfHz = {&masterConfig.gyro_soft_lpf_hz, 0, 255, 1};
OSD_UINT16_t entryDtermLpf = {&masterConfig.profile[0].pidProfile.dterm_lpf_hz, 0, 500, 5};
OSD_UINT16_t entryYawLpf = {&masterConfig.profile[0].pidProfile.yaw_lpf_hz, 0, 500, 5};
OSD_UINT16_t entryYawPLimit = {&masterConfig.profile[0].pidProfile.yaw_p_limit, 100, 500, 5};
OSD_UINT8_t entryVbatScale = {&masterConfig.batteryConfig.vbatscale, 1, 250, 1};
OSD_UINT8_t entryVbatMaxCell = {&masterConfig.batteryConfig.vbatmaxcellvoltage, 10, 50, 1};

OSD_Entry menuImuMisc[]=
{
    {"--- MISC ---", OME_Label, NULL, NULL, 0},
    {"GYRO LPF", OME_UINT8, NULL, &entryGyroSoftLpfHz, 0},
    {"DTERM LPF", OME_UINT16, NULL, &entryDtermLpf, 0},
    {"YAW LPF", OME_UINT16, NULL, &entryYawLpf, 0},
    {"YAW P LIM", OME_UINT16, NULL, &entryYawPLimit, 0},
    {"MIN THR", OME_UINT16, NULL, &entryMinThrottle, 0},
    {"VBAT SCALE", OME_UINT8, NULL, &entryVbatScale, 0},
    {"VBAT CLMAX", OME_UINT8, NULL, &entryVbatMaxCell, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

OSD_Entry cmsx_menuImu[] =
{
    {"--- CFG.IMU ---", OME_Label, NULL, NULL, 0},
    {"PID PROF", OME_UINT8, NULL, &entryPidProfile, 0},
    {"PID", OME_Submenu, cmsMenuChange, cmsx_menuPid, 0},
    {"RATE&RXPO", OME_Submenu, cmsMenuChange, cmsx_menuRateExpo, 0},
    {"RC PREV", OME_Submenu, cmsMenuChange, cmsx_menuRc, 0},
    {"MISC", OME_Submenu, cmsMenuChange, menuImuMisc, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};
#endif // CMS
