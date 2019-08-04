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

#include "drivers/pwm_output.h"

#define ESCSERIAL_BUFFER_SIZE 1024

typedef enum {
    ESCSERIAL1 = 0,
    ESCSERIAL2
} escSerialPortIndex_e;

typedef enum {
    PROTOCOL_SIMONK = 0,
    PROTOCOL_BLHELI = 1,
    PROTOCOL_KISS = 2,
    PROTOCOL_KISSALL = 3,
    PROTOCOL_CASTLE = 4,
    PROTOCOL_COUNT
} escProtocol_e;

// serialPort API
struct motorDevConfig_s;
bool escEnablePassthrough(serialPort_t *escPassthroughPort, const struct motorDevConfig_s *motorConfig, uint16_t escIndex, uint8_t mode);

typedef struct escSerialConfig_s {
    ioTag_t ioTag;
} escSerialConfig_t;

PG_DECLARE(escSerialConfig_t, escSerialConfig);
