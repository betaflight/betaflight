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

#pragma once

#include "drivers/serial.h"

//
// The protocol for Runcam Device definition
//
#define RCDEVICE_PROTOCOL_HEADER                                    0xCC

#define RCDEVICE_PROTOCOL_MAX_PACKET_SIZE                           64
#define RCDEVICE_PROTOCOL_MAX_DATA_SIZE                             62
#define RCDEVICE_PROTOCOL_MAX_DATA_SIZE_WITH_CRC_FIELD              63
#define RCDEVICE_PROTOCOL_MAX_MENUITEM_PER_PAGE                     32
#define RCDEVICE_PROTOCOL_MAX_SETTING_NAME_LENGTH                   20
#define RCDEVICE_PROTOCOL_MAX_SETTING_VALUE_LENGTH                  20
#define RCDEVICE_PROTOCOL_MAX_CHUNK_PER_RESPONSE                    12
#define RCDEVICE_PROTOCOL_MAX_TEXT_SELECTIONS                       30
#define RCDEVICE_PROTOCOL_MAX_STRING_LENGTH                         58

// Commands
#define RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO                   0x00
// camera control
#define RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL                    0x01
// 5 key osd cable simulation
#define RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS             0x02
#define RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE           0x03
#define RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION                   0x04
// device setting access
#define RCDEVICE_PROTOCOL_COMMAND_GET_SETTINGS                      0x10
#define RCDEVICE_PROTOCOL_COMMAND_READ_SETTING_DETAIL               0x11
#define RCDEVICE_PROTOCOL_COMMAND_READ_SETTING                      0x12
#define RCDEVICE_PROTOCOL_COMMAND_WRITE_SETTING                     0x13
// display port support
#define RCDEVICE_PROTOCOL_COMMAND_DISP_FILL_REGION                  0x20
#define RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_CHAR                   0x21
#define RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_HORIZONTAL_STRING      0x22
#define RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_VERTICAL_STRING        0x23
#define RCDEVICE_PROTOCOL_COMMAND_DISP_WRITE_CHARS                  0x24

// Feature Flag sets, it's a uint16_t flag
typedef enum {
    RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON    = (1 << 0),
    RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON     = (1 << 1),
    RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE              = (1 << 2),
    RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE = (1 << 3),
    RCDEVICE_PROTOCOL_FEATURE_DEVICE_SETTINGS_ACCESS   = (1 << 4),
    RCDEVICE_PROTOCOL_FEATURE_DISPLAYP_PORT            = (1 << 5),
    RCDEVICE_PROTOCOL_FEATURE_START_RECORDING          = (1 << 6),
    RCDEVICE_PROTOCOL_FEATURE_STOP_RECORDING           = (1 << 7),
    RCDEVICE_PROTOCOL_FEATURE_CMS_MENU                 = (1 << 8),
} rcdevice_features_e;

// Operation of Camera Button Simulation
typedef enum {
    RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_WIFI_BTN        = 0x00,
    RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_POWER_BTN       = 0x01,
    RCDEVICE_PROTOCOL_CAM_CTRL_CHANGE_MODE              = 0x02,
    RCDEVICE_PROTOCOL_CAM_CTRL_START_RECORDING          = 0x03,
    RCDEVICE_PROTOCOL_CAM_CTRL_STOP_RECORDING           = 0x04,
    RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION = 0xFF
} rcdevice_camera_control_opeation_e;

// Operation Of 5 Key OSD Cable Simulation
typedef enum {
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE  = 0x00,
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET   = 0x01,
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT  = 0x02,
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT = 0x03,
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP    = 0x04,
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN  = 0x05
} rcdevice_5key_simulation_operation_e;

// Operation of RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION
typedef enum {
    RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN = 0x01,
    RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE = 0x02
} RCDEVICE_5key_connection_event_e;

typedef enum {
    RCDEVICE_CAM_KEY_NONE,
    RCDEVICE_CAM_KEY_ENTER,
    RCDEVICE_CAM_KEY_LEFT,
    RCDEVICE_CAM_KEY_UP,
    RCDEVICE_CAM_KEY_RIGHT,
    RCDEVICE_CAM_KEY_DOWN,
    RCDEVICE_CAM_KEY_CONNECTION_CLOSE,
    RCDEVICE_CAM_KEY_CONNECTION_OPEN,
    RCDEVICE_CAM_KEY_RELEASE,
} rcdeviceCamSimulationKeyEvent_e;

typedef enum {
    RCDEVICE_PROTOCOL_RCSPLIT_VERSION = 0x00, // this is used to indicate the
                                              // device that using rcsplit
                                              // firmware version that <= 1.1.0
    RCDEVICE_PROTOCOL_VERSION_1_0 = 0x01,
    RCDEVICE_PROTOCOL_UNKNOWN
} rcdevice_protocol_version_e;

// Reserved setting ids
typedef enum {
    RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET                = 0, // type: text_selection, read&write, 0: use charset with betaflight logo, 1 use
                                                                 // charset with cleanflight logo, other id are not used
    RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS                = 1, // type: uint8_t, read only, the column count of the OSD layer
    RCDEVICE_PROTOCOL_SETTINGID_DISP_TV_MODE                = 2, // type: text_selection, read&write, 0:NTSC, 1:PAL
    RCDEVICE_PROTOCOL_SETTINGID_SDCARD_CAPACITY             = 3, // type: info, read only, return sd card capacity
    RCDEVICE_PROTOCOL_SETTINGID_REMAINING_RECORDING_TIME    = 4, // type: info, read only, return remaining recording time
    RCDEVICE_PROTOCOL_SETTINGID_RESOLUTION                  = 5, // type: text selection, read&write, return the current resolution and all available resolutions
    RCDEVICE_PROTOCOL_SETTINGID_CAMERA_TIME                 = 6, // type: string, read&write, update the camera time, the time attribute of  medias file in camera will use this time.
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED7                   = 7,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED8                   = 8,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED9                   = 9,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED10                  = 10,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED11                  = 11,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED12                  = 12,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED13                  = 13,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED14                  = 14,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED15                  = 15,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED16                  = 16,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED17                  = 17,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED18                  = 18,
    RCDEVICE_PROTOCOL_SETTINGID_RESERVED19                  = 19,
} rcdeviceReservedSettingID_e;

typedef enum {
    RCDEVICE_PROTOCOL_SETTINGTYPE_UINT8          = 0,
    RCDEVICE_PROTOCOL_SETTINGTYPE_INT8           = 1,
    RCDEVICE_PROTOCOL_SETTINGTYPE_UINT16         = 2,
    RCDEVICE_PROTOCOL_SETTINGTYPE_INT16          = 3,
    RCDEVICE_PROTOCOL_SETTINGTYPE_FLOAT          = 8,
    RCDEVICE_PROTOCOL_SETTINGTYPE_TEXT_SELECTION = 9,
    RCDEVICE_PROTOCOL_SETTINGTYPE_STRING         = 10,
    RCDEVICE_PROTOCOL_SETTINGTYPE_FOLDER         = 11,
    RCDEVICE_PROTOCOL_SETTINGTYPE_INFO           = 12,
    RCDEVICE_PROTOCOL_SETTINGTYPE_UNKNOWN
} rcdeviceSettingType_e;

typedef enum {
    RCDEVICE_SUCCEED            = 0,
    RCDEVICE_INVALID            = 1,
    RCDEVICE_NODEV              = 2,
    RCDEVICE_DEVBUSY            = 3,
} rcdeviceErrorCode_e;

// end of Runcam Device definition

// Old version defination(RCSplit firmware v1.0.0 and v1.1.0)
// packet header and tail
#define RCSPLIT_PACKET_HEADER       0x55
#define RCSPLIT_PACKET_CMD_CTRL     0x01
#define RCSPLIT_PACKET_TAIL         0xaa



typedef enum {
    RCSPLIT_CTRL_ARGU_INVALID     = 0x0,
    RCSPLIT_CTRL_ARGU_WIFI_BTN    = 0x1,
    RCSPLIT_CTRL_ARGU_POWER_BTN   = 0x2,
    RCSPLIT_CTRL_ARGU_CHANGE_MODE = 0x3,
    RCSPLIT_CTRL_ARGU_WHO_ARE_YOU = 0xFF,
} rcsplit_ctrl_argument_e;
// end of old version protocol definition

typedef struct runcamDeviceInfo_s {
    rcdevice_protocol_version_e protocolVersion;
    uint16_t features;
} runcamDeviceInfo_t;

typedef struct runcamDeviceSetting_s {
    uint8_t id;
    char name[RCDEVICE_PROTOCOL_MAX_SETTING_NAME_LENGTH];
    char value[RCDEVICE_PROTOCOL_MAX_SETTING_VALUE_LENGTH];
} runcamDeviceSetting_t;

typedef struct runcamDeviceSettingTextSelection_s {
    char text[RCDEVICE_PROTOCOL_MAX_SETTING_VALUE_LENGTH];
} runcamDeviceSettingTextSelection_t;

typedef struct runcamDeviceSettingDetail_s {
    uint8_t type;
    uint32_t value;
    uint32_t minValue;
    uint32_t maxValue;
    uint8_t decimalPoint;
    uint32_t stepSize;
    uint8_t maxStringSize;
    char stringValue[RCDEVICE_PROTOCOL_MAX_STRING_LENGTH]; // when settingType is RCDEVICE_PROTOCOL_SETTINGTYPE_INFO or RCDEVICE_PROTOCOL_SETTINGTYPE_STRING, this field store the string/info value;
    runcamDeviceSettingTextSelection_t textSelections[RCDEVICE_PROTOCOL_MAX_TEXT_SELECTIONS];
} runcamDeviceSettingDetail_t;

typedef struct runcamDeviceWriteSettingResponse_s {
    uint8_t resultCode;
    uint8_t needUpdateMenuItems;
} runcamDeviceWriteSettingResponse_t;

typedef struct runcamDevice_s {
    serialPort_t *serialPort;
    uint8_t buffer[RCDEVICE_PROTOCOL_MAX_PACKET_SIZE];
    runcamDeviceInfo_t info;
} runcamDevice_t;

bool runcamDeviceInit(runcamDevice_t *device);

// camera button simulation
bool runcamDeviceSimulateCameraButton(runcamDevice_t *device, uint8_t operation);

// 5 key osd cable simulation
bool runcamDeviceOpen5KeyOSDCableConnection(runcamDevice_t *device);
bool runcamDeviceClose5KeyOSDCableConnection(runcamDevice_t *device, uint8_t *resultCode);
bool runcamDeviceSimulate5KeyOSDCableButtonPress(runcamDevice_t *device, uint8_t operation);
bool runcamDeviceSimulate5KeyOSDCableButtonRelease(runcamDevice_t *device);

// DisplayPort feature support
void runcamDeviceDispFillRegion(runcamDevice_t *device, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t c);
void runcamDeviceDispWriteChar(runcamDevice_t *device, uint8_t x, uint8_t y, uint8_t c);
void runcamDeviceDispWriteHorizontalString(runcamDevice_t *device, uint8_t x, uint8_t y, const char *text);
void runcamDeviceDispWriteVerticalString(runcamDevice_t *device, uint8_t x, uint8_t y, const char *text);
void runcamDeviceDispWriteChars(runcamDevice_t *device, uint8_t *data, uint8_t datalen);

// Device Setting Access
bool runcamDeviceGetSettings(runcamDevice_t *device, uint8_t parentSettingID, runcamDeviceSetting_t *outSettingList, int maxSettingItemCount);
bool runcamDeviceGetSettingDetail(runcamDevice_t *device, uint8_t settingID, runcamDeviceSettingDetail_t *outSettingDetail);
bool runcamDeviceWriteSetting(runcamDevice_t *device, uint8_t settingID, uint8_t *data, uint8_t dataLen, runcamDeviceWriteSettingResponse_t *response);
