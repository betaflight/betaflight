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

#include "build/debug.h"
#include "build/debug_pin.h"

#include "drivers/io.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/motor_types.h"
#include "drivers/time.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "dshot_pio_programs.h"

#define PIO_SHIFT_LEFT (false)
#define PIO_SHIFT_RIGHT (true)
#define PIO_NO_AUTO_PUSHPULL (false)

typedef struct motorOutput_s {
    int offset;              // NB current code => offset same for all motors
    dshotProtocolControl_t protocolControl;
    int pinIndex;            // pinIndex of this motor output
    IO_t io;                 // IO_t for this output
    bool configured;
    bool enabled;
    PIO pio;
    uint16_t pio_sm;
} motorOutput_t;

extern const PIO dshotPio; // currently only single pio supported => 4 motors.

extern motorProtocolTypes_e dshotMotorProtocol;

extern motorOutput_t dshotMotors[MAX_SUPPORTED_MOTORS];

float dshotGetPeriodTiming(void);

bool dshot_program_bidir_init(PIO pio, uint sm, int offset, uint pin);
bool dshotTelemetryWait(void);
bool dshotDecodeTelemetry(void);
