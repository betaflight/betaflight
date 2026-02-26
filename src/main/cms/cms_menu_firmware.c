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

//
// Firmware related menu contents and support functions
//

#include <ctype.h>

#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "common/printf.h"

#include "config/config.h"

#include "drivers/system.h"

#include "fc/board_info.h"
#include "fc/runtime_config.h"

#include "pg/board.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"

#ifdef USE_HOVER_CALIBRATION
#include "flight/hover_calibration.h"
#include "pg/autopilot.h"
#endif

#include "cms_menu_firmware.h"

// Calibration

#define CALIBRATION_STATUS_MAX_LENGTH 6

#define CALIBRATION_STATUS_OFF " --- "
#define CALIBRATION_STATUS_NOK " NOK "
#define CALIBRATION_STATUS_WAIT "WAIT "
#define CALIBRATION_STATUS_OK "  OK "

static char gyroCalibrationStatus[CALIBRATION_STATUS_MAX_LENGTH];
#if defined(USE_ACC)
static char accCalibrationStatus[CALIBRATION_STATUS_MAX_LENGTH];
#endif
#if defined(USE_BARO)
static char baroCalibrationStatus[CALIBRATION_STATUS_MAX_LENGTH];
#endif
#if defined(USE_HOVER_CALIBRATION)
#define HOVER_CAL_STATUS_MAX_LENGTH 12
static char hoverCalibrationStatus[HOVER_CAL_STATUS_MAX_LENGTH];
#endif

static const void *cmsx_CalibrationOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

    tfp_sprintf(gyroCalibrationStatus, sensors(SENSOR_GYRO) ? gyroIsCalibrationComplete() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
#if defined(USE_ACC)
    tfp_sprintf(accCalibrationStatus, sensors(SENSOR_ACC) ? accIsCalibrationComplete() ? accHasBeenCalibrated() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_NOK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
#endif
#if defined(USE_BARO)
    tfp_sprintf(baroCalibrationStatus, sensors(SENSOR_BARO) ? baroIsCalibrated() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
#endif

    return NULL;
}

static const void *cmsCalibrateGyro(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (sensors(SENSOR_GYRO)) {
        gyroStartCalibration(false);
    }

    return NULL;
}

#if defined(USE_ACC)
static const void *cmsCalibrateAcc(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (sensors(SENSOR_ACC)) {
        accStartCalibration();
    }

    return MENU_CHAIN_BACK;
}
#endif

#if defined(USE_BARO)
static const void *cmsCalibrateBaro(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (sensors(SENSOR_BARO)) {
        baroStartCalibration();
    }

    return NULL;
}
#endif

#if defined(USE_HOVER_CALIBRATION)
static uint16_t hoverCalibration_hoverThrottle;

static const void *cmsx_HoverCalibrationOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    hoverCalibration_hoverThrottle = autopilotConfig()->hoverThrottle;

    return NULL;
}

static const char* getFailReasonString(hoverCalibrationFailReason_e reason)
{
    switch (reason) {
        case HOVER_CAL_FAIL_DISARMED:    return "DISARMED";
        case HOVER_CAL_FAIL_NO_ALTITUDE: return "NO ALT";
        case HOVER_CAL_FAIL_TOO_LOW:     return "TOO LOW";
        case HOVER_CAL_FAIL_NOT_LEVEL:   return "NOT LEVEL";
        case HOVER_CAL_FAIL_MOVING:      return "MOVING";
        case HOVER_CAL_FAIL_ALTHOLD_MODE:return "ALTHOLD ON";
        case HOVER_CAL_FAIL_RESULT_RANGE:return "BAD RESULT";
        default:                         return "";
    }
}

static const void *cmsx_HoverCalibrationOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

    // Update the display value from config (in case calibration changed it)
    hoverCalibration_hoverThrottle = autopilotConfig()->hoverThrottle;

    const hoverCalibrationStatus_e status = getHoverCalibrationStatus();
    const hoverCalibrationFailReason_e failReason = getHoverCalibrationFailReason();

    switch (status) {
        case HOVER_CAL_STATUS_IDLE:
            tfp_sprintf(hoverCalibrationStatus, "READY");
            break;
        case HOVER_CAL_STATUS_WAITING_STABLE:
            // Show why we're waiting if there's a reason
            if (failReason != HOVER_CAL_FAIL_NONE) {
                tfp_sprintf(hoverCalibrationStatus, "%s", getFailReasonString(failReason));
            } else {
                tfp_sprintf(hoverCalibrationStatus, "STABILIZE...");
            }
            break;
        case HOVER_CAL_STATUS_SAMPLING:
            tfp_sprintf(hoverCalibrationStatus, "SAMPLE %3d%%", getHoverCalibrationProgress());
            break;
        case HOVER_CAL_STATUS_COMPLETE:
            tfp_sprintf(hoverCalibrationStatus, "DONE: %4d", getHoverCalibrationResult());
            break;
        case HOVER_CAL_STATUS_FAILED:
            tfp_sprintf(hoverCalibrationStatus, "FAIL:%s", getFailReasonString(failReason));
            break;
        default:
            tfp_sprintf(hoverCalibrationStatus, "---");
            break;
    }

    return NULL;
}

static const void *cmsStartHoverCalibration(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    hoverCalibrationStart();

    return NULL;
}

static const OSD_Entry menuCalibrateHoverEntries[] = {
    { "-- HOVER CALIB --", OME_Label, NULL, NULL },
    { "ARM AND HOVER", OME_Label, NULL, NULL },
    { "STEADILY >50CM", OME_Label, NULL, NULL },
    { " ", OME_Label, NULL, NULL },
    { "STATUS", OME_String | DYNAMIC, NULL, hoverCalibrationStatus },
    { "START", OME_Funcall, cmsStartHoverCalibration, NULL },
    { "CURRENT", OME_UINT16 | DYNAMIC, NULL, &(OSD_UINT16_t){ &hoverCalibration_hoverThrottle, 1100, 1700, 1 } },
    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL }
};

static CMS_Menu cmsx_menuCalibrateHover = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "HOVERCALIBRATION",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_HoverCalibrationOnEnter,
    .onExit = NULL,
    .onDisplayUpdate = cmsx_HoverCalibrationOnDisplayUpdate,
    .entries = menuCalibrateHoverEntries
};

static const void *cmsCalibrateHoverMenu(displayPort_t *pDisp, const void *self)
{
    UNUSED(self);

    cmsMenuChange(pDisp, &cmsx_menuCalibrateHover);

    return NULL;
}
#endif

#if defined(USE_ACC)
static const OSD_Entry menuCalibrateAccEntries[] = {
    { "--- CALIBRATE ACC ---", OME_Label, NULL, NULL },
    { "PLACE ON A LEVEL SURFACE", OME_Label, NULL, NULL},
    { "MAKE SURE CRAFT IS STILL", OME_Label, NULL, NULL},
    { " ", OME_Label, NULL, NULL},
    { "START CALIBRATION",  OME_Funcall, cmsCalibrateAcc, NULL },
    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuCalibrateAcc = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "ACCCALIBRATION",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = menuCalibrateAccEntries
};

const void *cmsCalibrateAccMenu(displayPort_t *pDisp, const void *self)
{
    UNUSED(self);

    if (sensors(SENSOR_ACC)) {
        cmsMenuChange(pDisp, &cmsx_menuCalibrateAcc);
    }

    return NULL;
}

#endif

static const OSD_Entry menuCalibrationEntries[] = {
    { "--- CALIBRATE ---", OME_Label, NULL, NULL },
    { "GYRO", OME_Funcall | DYNAMIC, cmsCalibrateGyro, gyroCalibrationStatus },
#if defined(USE_ACC)
    { "ACC",  OME_Funcall | DYNAMIC, cmsCalibrateAccMenu, accCalibrationStatus },
#endif
#if defined(USE_BARO)
    { "BARO", OME_Funcall | DYNAMIC, cmsCalibrateBaro, baroCalibrationStatus },
#endif
#if defined(USE_HOVER_CALIBRATION)
    { "HOVER", OME_Funcall, cmsCalibrateHoverMenu, NULL },
#endif
    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

static CMS_Menu cmsx_menuCalibration = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUCALIBRATION",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = cmsx_CalibrationOnDisplayUpdate,
    .entries = menuCalibrationEntries
};

// Info

#if defined(USE_BOARD_INFO)
static char manufacturerId[MAX_MANUFACTURER_ID_LENGTH + 1];
static char boardName[MAX_BOARD_NAME_LENGTH + 1];

static const void *cmsx_FirmwareInit(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    strncpy(manufacturerId, getManufacturerId(), MAX_MANUFACTURER_ID_LENGTH);
    manufacturerId[MAX_MANUFACTURER_ID_LENGTH] = 0;
    strncpy(boardName, getBoardName(), MAX_BOARD_NAME_LENGTH);
    boardName[MAX_BOARD_NAME_LENGTH] = 0;

    return NULL;
}
#endif

static const OSD_Entry menuFirmwareEntries[] = {
    { "--- INFO ---", OME_Label, NULL, NULL },
    { "FWID", OME_String, NULL, FC_FIRMWARE_IDENTIFIER },
    { "FWVER", OME_String, NULL, FC_VERSION_STRING },
    { "GITREV", OME_String, NULL, __REVISION__ },
    { "TARGET", OME_String, NULL, __TARGET__ },
#if defined(USE_BOARD_INFO)
    { "MFR", OME_String, NULL, manufacturerId },
    { "BOARD", OME_String, NULL, boardName },
#endif
    { "--- SETUP ---", OME_Label, NULL, NULL },
    { "CALIBRATE",     OME_Submenu, cmsMenuChange, &cmsx_menuCalibration},
    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuFirmware = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUFIRMWARE",
    .GUARD_type = OME_MENU,
#endif
#if defined(USE_BOARD_INFO)
    .onEnter = cmsx_FirmwareInit,
#else
    .onEnter = NULL,
#endif
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = menuFirmwareEntries
};
#endif
