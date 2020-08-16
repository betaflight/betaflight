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

static const void *cmsx_CalibrationOnDisplayUpdate(displayPort_t *pDisp, const OSD_Entry *selected)
{
    UNUSED(pDisp);
    UNUSED(selected);

    tfp_sprintf(gyroCalibrationStatus, sensors(SENSOR_GYRO) ? gyroIsCalibrationComplete() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
#if defined(USE_ACC)
    tfp_sprintf(accCalibrationStatus, sensors(SENSOR_ACC) ? accIsCalibrationComplete() ? accHasBeenCalibrated() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_NOK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
#endif
#if defined(USE_BARO)
    tfp_sprintf(baroCalibrationStatus, sensors(SENSOR_BARO) ? baroIsCalibrationComplete() ? CALIBRATION_STATUS_OK : CALIBRATION_STATUS_WAIT: CALIBRATION_STATUS_OFF);
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

#if defined(USE_ACC)
static const OSD_Entry menuCalibrateAccEntries[] = {
    { "--- CALIBRATE ACC ---", OME_Label, NULL, NULL, 0 },
    { "PLACE ON A LEVEL SURFACE", OME_Label, NULL, NULL, 0},
    { "MAKE SURE CRAFT IS STILL", OME_Label, NULL, NULL, 0},
    { " ", OME_Label, NULL, NULL, 0},
    { "START CALIBRATION",  OME_Funcall, cmsCalibrateAcc, NULL, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
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
    { "--- CALIBRATE ---", OME_Label, NULL, NULL, 0 },
    { "GYRO", OME_Funcall, cmsCalibrateGyro, gyroCalibrationStatus, DYNAMIC },
#if defined(USE_ACC)
    { "ACC",  OME_Funcall, cmsCalibrateAccMenu, accCalibrationStatus, DYNAMIC },
#endif
#if defined(USE_BARO)
    { "BARO", OME_Funcall, cmsCalibrateBaro, baroCalibrationStatus, DYNAMIC },
#endif
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
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

    strncpy(manufacturerId, getManufacturerId(), MAX_MANUFACTURER_ID_LENGTH + 1);
    strncpy(boardName, getBoardName(), MAX_BOARD_NAME_LENGTH + 1);

    return NULL;
}
#endif

static const OSD_Entry menuFirmwareEntries[] = {
    { "--- INFO ---", OME_Label, NULL, NULL, 0 },
    { "FWID", OME_String, NULL, FC_FIRMWARE_IDENTIFIER, 0 },
    { "FWVER", OME_String, NULL, FC_VERSION_STRING, 0 },
    { "GITREV", OME_String, NULL, __REVISION__, 0 },
    { "TARGET", OME_String, NULL, __TARGET__, 0 },
#if defined(USE_BOARD_INFO)
    { "MFR", OME_String, NULL, manufacturerId, 0 },
    { "BOARD", OME_String, NULL, boardName, 0 },
#endif
    { "--- SETUP ---", OME_Label, NULL, NULL, 0 },
    { "CALIBRATE",     OME_Submenu, cmsMenuChange, &cmsx_menuCalibration, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
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
