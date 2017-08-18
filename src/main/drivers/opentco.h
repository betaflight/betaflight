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

#include "common/streambuf.h"
#include "io/serial.h"

#include <stdint.h>

typedef enum {
    OSD_WRITE_MODE_VERTICAL = 0x0,
    OSD_WRITE_MODE_HORIZONTAL
} openTCOCommandOSDWriteMode_e;

#define OPENTCO_PROTOCOL_HEADER 0x80

#define OPENTCO_MAX_DATA_LENGTH       60
#define OPENTCO_MAX_FRAME_LENGTH     (OPENTCO_MAX_DATA_LENGTH + 4)

// 0x01..0x07 = valid device ids
#define OPENTCO_DEVICE_OSD                           0x00
#define OPENTCO_DEVICE_VTX                           0x01
#define OPENTCO_DEVICE_CAM                           0x02
//
#define OPENTCO_DEVICE_MAX                           0x07

// 0x08..0x0F = valid device response ids
#define OPENTCO_DEVICE_RESPONSE                      0x08
#define OPENTCO_DEVICE_OSD_RESPONSE                  (OPENTCO_DEVICE_RESPONSE | OPENTCO_DEVICE_OSD)
#define OPENTCO_DEVICE_VTX_RESPONSE                  (OPENTCO_DEVICE_RESPONSE | OPENTCO_DEVICE_VTX)
#define OPENTCO_DEVICE_CAM_RESPONSE                  (OPENTCO_DEVICE_RESPONSE | OPENTCO_DEVICE_CAM)


#define OPENTCO_OSD_COMMAND_REGISTER_ACCESS          0x00
#define OPENTCO_OSD_COMMAND_FILL_REGION              0x01
#define OPENTCO_OSD_COMMAND_WRITE                    0x02
#define OPENTCO_OSD_COMMAND_WRITE_BUFFER_H           0x08
#define OPENTCO_OSD_COMMAND_WRITE_BUFFER_V           0x09
#define OPENTCO_OSD_COMMAND_SPECIAL                  0x0F

#define OPENTCO_REGISTER_ACCESS_MODE_READ            0x80
#define OPENTCO_REGISTER_ACCESS_MODE_WRITE           0x00

#define OPENTCO_OSD_REGISTER_STATUS                  0x00  // R/W
//
#define OPENTCO_OSD_REGISTER_VIDEO_FORMAT            0x01  // R/W
#define OPENTCO_OSD_REGISTER_INVERT                  0x02  // R/W
#define OPENTCO_OSD_REGISTER_BRIGHTNESS_BLACK        0x03  // R/W
#define OPENTCO_OSD_REGISTER_BRIGHTNESS_WHITE        0x04  // R/W
#define OPENTCO_MAX_REGISTER                         0x0F


#define OPENTCO_OSD_COMMAND_SPECIAL_SUB_STICKSTATUS  0x00
#define OPENTCO_OSD_COMMAND_SPECIAL_SUB_SPECTRUM     0x01

typedef enum {
    OPENTCO_OSD_ENABLE                = (1 << 0),
    OPENTCO_OSD_SHOW_LOGO             = (1 << 1),
    OPENTCO_OSD_SHOW_STICKOVERLAY     = (1 << 2),
    OPENTCO_OSD_SHOW_SPECTRUMOVERLAY  = (1 << 3),
    OPENTCO_OSD_SHOW_CROSSHAIR        = (1 << 4)
} opentcoOSDRegisterStatus_e;


typedef struct {
    serialPort_t *serialPort;

    uint8_t buffer[OPENTCO_MAX_FRAME_LENGTH];

    sbuf_t streamBuffer;
    sbuf_t *sbuf;

    uint8_t id;

    bool locked;
} opentcoDevice_t;

bool opentcoInit(opentcoDevice_t *device);

void opentcoInitializeFrame(opentcoDevice_t *device, uint8_t command);
void opentcoSendFrame(opentcoDevice_t *device);

bool opentcoReadRegister(opentcoDevice_t *device, uint8_t reg, uint16_t *val);
bool opentcoWriteRegister(opentcoDevice_t *device, uint8_t reg, uint16_t val);
