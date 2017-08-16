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

#define OPENTCO_DEVICE_OSD                           0x00
#define OPENTCO_DEVICE_VTX                           0x01
#define OPENTCO_DEVICE_CAM                           0x02

#define OPENTCO_OSD_COMMAND_SET_REGISTER             0x00
#define OPENTCO_OSD_COMMAND_FILL_REGION              0x01
#define OPENTCO_OSD_COMMAND_WRITE                    0x02
#define OPENTCO_OSD_COMMAND_WRITE_BUFFER_H           0x08
#define OPENTCO_OSD_COMMAND_WRITE_BUFFER_V           0x09
#define OPENTCO_OSD_COMMAND_SPECIAL                  0x0F

#define OPENTCO_OSD_REGISTER_STATUS                  0x00
#define OPENTCO_OSD_REGISTER_VIDEO_FORMAT            0x01
#define OPENTCO_OSD_REGISTER_INVERT                  0x02
#define OPENTCO_OSD_REGISTER_BRIGHTNESS_BLACK        0x03
#define OPENTCO_OSD_REGISTER_BRIGHTNESS_WHITE        0x04

#define OPENTCO_OSD_COMMAND_SPECIAL_SUB_STICKSTATUS  0x00
#define OPENTCO_OSD_COMMAND_SPECIAL_SUB_SPECTRUM     0x01

bool opentcoInit(serialPortFunction_e function);
void opentcoInitializeFrame(sbuf_t *dst, uint8_t device, uint8_t command);
void opentcoSendFrame(sbuf_t *dst);
