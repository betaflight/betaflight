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

#include "platform.h"

#if ENABLE_DRONECAN_ESC

#include <stdint.h>

#include "common/time.h"

#include "pg/motor.h"

#include "drivers/motor_types.h"

// Install the esc.Status subscriber (telemetry in) and reset the RawCommand
// throttle buffer (command out). Called once from dronecanInit().
void dronecanEscInit(void);

// Per-tick service from the dronecan task: broadcasts esc.RawCommand from the
// latest motor throttles and ages telemetry slots that have gone quiet.
void dronecanEscUpdate(timeUs_t currentTimeUs);

// Called from the motor driver vtable (PID-loop context). Stores one motor's
// normalised command [0,1] into the shared throttle buffer; the dronecan task
// reads it and broadcasts RawCommand at its own rate. updateComplete() commits.
void dronecanEscWrite(uint8_t motorIndex, float value);
void dronecanEscUpdateComplete(void);

// Motor-driver entry points (called from drivers/motor.c). Populate the motor
// vtable and define the output endpoints for the DRONECAN protocol family.
bool dronecanMotorDevInit(motorDevice_t *device, uint8_t motorCount);
void dronecanMotorInitEndpoints(const motorConfig_t *motorConfig, float outputLimit,
                                float *outputLow, float *outputHigh, float *disarm,
                                float *deadbandMotor3dHigh, float *deadbandMotor3dLow);

#endif // ENABLE_DRONECAN_ESC
