/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "io_types.h"
#include "pg/pg.h"
#include "drivers/time.h"

#define CAMERA_CONTROL_PWM_RESOLUTION      128
#define CAMERA_CONTROL_SOFT_PWM_RESOLUTION 448

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

typedef struct cameraControlRuntime_s{
    bool enabled;
    IO_t io;
    uint32_t period;
    uint8_t inverted;
    timeMs_t endTimeMillis;
} cameraControlRuntime_t;

typedef struct cameraControlConfig_s {
    cameraControlMode_e mode;
    // measured in 10 mV steps
    uint16_t refVoltage;
    uint16_t keyDelayMs;
    // measured 100 Ohm steps
    uint16_t internalResistance;

    ioTag_t ioTag;
    uint8_t inverted;
    uint16_t buttonResistanceValues[CAMERA_CONTROL_KEYS_COUNT]; // resistance in 100ohm steps
} cameraControlConfig_t;

PG_DECLARE(cameraControlConfig_t, cameraControlConfig);

void cameraControlInit(void);
void cameraControlProcess(timeUs_t currentTimeUs);
void cameraControlKeyPress(cameraControlKey_e key, timeMs_t holdDurationMs);
