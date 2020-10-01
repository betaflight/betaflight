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

#pragma once

#include "drivers/serial.h"

//
// The protocol for Runcam Device definition
//
#define RCDEVICE_PROTOCOL_HEADER                                    0xCC

#define RCDEVICE_PROTOCOL_MAX_PACKET_SIZE                           64
#define RCDEVICE_PROTOCOL_MAX_DATA_SIZE                             62

// Commands
#define RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO                   0x00
// camera control
#define RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL                    0x01
// 5 key osd cable simulation
#define RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS             0x02
#define RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE           0x03
#define RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION                   0x04
#define RCDEVICE_PROTOCOL_COMMAND_REQUEST_FC_ATTITUDE               0x50

// Old protocol defines
#define RCSPLIT_PACKET_HEADER           0x55
#define RCSPLIT_PACKET_CMD_CTRL  0x01
#define RCSPLIT_PACKET_TAIL     0xaa

// Feature Flag sets, it's a uint16_t flag
typedef enum {
    RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON    = (1 << 0),
    RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON     = (1 << 1),
    RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE              = (1 << 2),
    RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE = (1 << 3),
    RCDEVICE_PROTOCOL_FEATURE_START_RECORDING          = (1 << 6),
    RCDEVICE_PROTOCOL_FEATURE_STOP_RECORDING           = (1 << 7),
    RCDEVICE_PROTOCOL_FEATURE_CMS_MENU                 = (1 << 8),
    RCDEVICE_PROTOCOL_FEATURE_FC_ATTITUDE              = (1 << 9)
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

// end of Runcam Device definition

typedef struct runcamDeviceInfo_s {
    rcdevice_protocol_version_e protocolVersion;
    uint16_t features;
} runcamDeviceInfo_t;

typedef struct runcamDevice_s {
    serialPort_t *serialPort;
    uint8_t buffer[RCDEVICE_PROTOCOL_MAX_PACKET_SIZE];
    runcamDeviceInfo_t info;
    bool isReady;
} runcamDevice_t;

#define MAX_WAITING_RESPONSES 1

typedef enum {
    RCDEVICE_RESP_SUCCESS = 0,
    RCDEVICE_RESP_INCORRECT_CRC = 1,
    RCDEVICE_RESP_TIMEOUT = 2
} rcdeviceResponseStatus_e;

typedef struct rcdeviceResponseParseContext_s rcdeviceResponseParseContext_t;
typedef void(*rcdeviceRespParseFunc)(rcdeviceResponseParseContext_t*);
struct rcdeviceResponseParseContext_s {
    uint8_t command;
    uint8_t expectedRespLen; // total length of response data
    uint8_t recvRespLen; // length of the data received
    uint8_t *recvBuf; // response data buffer
    timeMs_t timeout;
    timeMs_t timeoutTimestamp; // if zero, it's means keep waiting for the response
    rcdeviceRespParseFunc parserFunc;
    runcamDevice_t *device;
    uint8_t paramData[RCDEVICE_PROTOCOL_MAX_DATA_SIZE];
    uint8_t paramDataLen;
    uint8_t protocolVersion;
    int maxRetryTimes;
    void *userInfo;
    rcdeviceResponseStatus_e result;
};

typedef struct {
    uint8_t headPos; // current head position of the queue
    uint8_t tailPos;
    uint8_t itemCount; // the item count in the queue
    rcdeviceResponseParseContext_t buffer[MAX_WAITING_RESPONSES];
    rcdeviceRespParseFunc parseFunc;
} rcdeviceWaitingResponseQueue;

typedef struct {
    uint8_t command;
    uint8_t data[RCDEVICE_PROTOCOL_MAX_DATA_SIZE - 1];
    uint8_t dataLength;
} runcamDeviceRequest_t;

void runcamDeviceInit(runcamDevice_t *device);
void rcdeviceReceive(timeUs_t currentTimeUs);

// camera button simulation
bool runcamDeviceSimulateCameraButton(runcamDevice_t *device, uint8_t operation);

// 5 key osd cable simulation
void runcamDeviceOpen5KeyOSDCableConnection(runcamDevice_t *device, rcdeviceRespParseFunc parseFunc);
void runcamDeviceClose5KeyOSDCableConnection(runcamDevice_t *device, rcdeviceRespParseFunc parseFunc);
void runcamDeviceSimulate5KeyOSDCableButtonPress(runcamDevice_t *device, uint8_t operation, rcdeviceRespParseFunc parseFunc);
void runcamDeviceSimulate5KeyOSDCableButtonRelease(runcamDevice_t *device, rcdeviceRespParseFunc parseFunc);

void runcamDeviceSendAttitude(runcamDevice_t *device);

runcamDeviceRequest_t* rcdeviceGetRequest();
