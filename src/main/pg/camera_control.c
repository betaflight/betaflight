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

#include <platform.h>

#ifdef USE_CAMERA_CONTROL

#include "pg/pg_ids.h"
#include "pg/camera_control.h"
#include "drivers/camera_control.h"
#include "drivers/io.h"

//#include "math.h"
//#include "nvic.h"
//#include "pwm_output.h"
//#include "time.h"

#ifndef CAMERA_CONTROL_PIN
#define CAMERA_CONTROL_PIN NONE
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(cameraControlConfig_t, cameraControlConfig, PG_CAMERA_CONTROL_CONFIG, 0);

PG_RESET_TEMPLATE(cameraControlConfig_t, cameraControlConfig,
    .mode = CAMERA_CONTROL_MODE_HARDWARE_PWM,
    .refVoltage = 330,
    .keyDelayMs = 180,
    .internalResistance = 470,
    .ioTag = IO_TAG(CAMERA_CONTROL_PIN)
);

#endif
