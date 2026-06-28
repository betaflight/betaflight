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

// Per-tick service from the dronecan task: ages telemetry slots that have gone
// quiet. RawCommand emission lives in updateComplete() (PID-loop context).
void dronecanEscUpdate(timeUs_t currentTimeUs);

// Called from the motor driver vtable (PID-loop context). write() stages one
// motor's RawCommand-unit throttle; updateComplete() encodes the staged values
// and, rate-gated to esc_rate_hz, broadcasts RawCommand and flushes it to the
// CAN driver immediately — no cross-task buffer needed (cooperative scheduler).
void dronecanEscWrite(uint8_t motorIndex, float value);
void dronecanEscUpdateComplete(void);

// Motor-driver entry points (called from drivers/motor.c). Populate the motor
// vtable and define the output endpoints for the DRONECAN protocol family.
bool dronecanMotorDevInit(motorDevice_t *device, uint8_t motorCount);
void dronecanMotorInitEndpoints(const motorConfig_t *motorConfig, float outputLimit,
                                float *outputLow, float *outputHigh, float *disarm,
                                float *deadbandMotor3dHigh, float *deadbandMotor3dLow);

#endif // ENABLE_DRONECAN_ESC
