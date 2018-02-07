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

#include "drivers/io_types.h"
#include "drivers/camera_control.h"
#include "pg/pg.h"

typedef struct cameraControlConfig_s {
    cameraControlMode_e mode;
    // measured in 10 mV steps
    uint16_t refVoltage;
    uint16_t keyDelayMs;
    // measured 100 Ohm steps
    uint16_t internalResistance;

    ioTag_t ioTag;
} cameraControlConfig_t;

PG_DECLARE(cameraControlConfig_t, cameraControlConfig);
