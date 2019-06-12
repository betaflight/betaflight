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

#include "io_types.h"
#include "pg/pg.h"

typedef enum {
    CAMERA_CONTROL_KEY_ENTER,
    CAMERA_CONTROL_KEY_LEFT,
    CAMERA_CONTROL_KEY_UP,
    CAMERA_CONTROL_KEY_RIGHT,
    CAMERA_CONTROL_KEY_DOWN,
    CAMERA_CONTROL_KEYS_COUNT
} cameraControlKey_e;

typedef enum {
    CAMERA_CONTROL_MODE_HARDWARE_PWM,
    CAMERA_CONTROL_MODE_SOFTWARE_PWM,
    CAMERA_CONTROL_MODE_DAC,
    CAMERA_CONTROL_MODES_COUNT
} cameraControlMode_e;

typedef struct cameraControlConfig_s {
    cameraControlMode_e mode;
    // measured in 10 mV steps
    uint16_t refVoltage;
    uint16_t keyDelayMs;
    // measured 100 Ohm steps
    uint16_t internalResistance;

    ioTag_t ioTag;
    uint8_t inverted;
} cameraControlConfig_t;

PG_DECLARE(cameraControlConfig_t, cameraControlConfig);

void cameraControlInit(void);

void cameraControlProcess(uint32_t currentTimeUs);
void cameraControlKeyPress(cameraControlKey_e key, uint32_t holdDurationMs);
