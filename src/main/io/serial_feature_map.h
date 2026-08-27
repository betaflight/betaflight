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

#include <stdbool.h>
#include <stdint.h>

#include "io/serial.h"

// Walk every feature PG that claims a UART and build a functionMask
// for the given port.  Drives the back-compat view exposed over MSP
// (MSP_CF_SERIAL_CONFIG) and the CLI `serial` command's output.
uint32_t serialSynthesizeFunctionMask(serialPortIdentifier_e identifier);

// Clear the claims on just those ports whose functions cannot coexist, sparing
// MSP so the board stays reachable.  First recovery step for a stored assignment
// that isSerialConfigValid() rejects, so one bad assignment costs the user that
// port rather than every port on the board.
void serialDropConflictingAssignments(void);

// Clear every feature's port claim and restore MSP on the first port.  Last
// resort for a configuration still invalid once the conflicting claims are gone,
// such as one that has lost MSP on the VCP port.
void serialResetFeatureAssignments(void);

// A rangefinder or optical flow module on an MSP transport is heard by pushing
// MSP frames, so its declared UART needs an MSP port even though the user never
// assigned MSP there.  Fills `ports` with the distinct UARTs that need one, one
// module answering as both sensors counting once, and returns how many.
unsigned serialImpliedMspPorts(serialPortIdentifier_e *ports, unsigned maxPorts);

// The four baud rates every port carries in the legacy serialPortConfig_t.
// Each is owned by one feature class rather than by the port, so the pair
// below translates between the legacy per-port layout and the feature PGs.
typedef enum {
    SERIAL_BAUD_MSP = 0,
    SERIAL_BAUD_GPS,
    SERIAL_BAUD_TELEMETRY,
    SERIAL_BAUD_BLACKBOX,
    SERIAL_BAUD_CLASS_COUNT,
} serialBaudClass_e;

// Baud a port reports for a class when no feature claims it.  `diff` compares
// against this to decide whether a port's baud is still at its default.
uint8_t serialDefaultPortBaud(serialBaudClass_e baudClass);

// Read the baud a port would carry for the given class in the legacy view.
// Ports no feature claims report the class default, so an untouched config
// still round-trips through `dump` and MSP unchanged.
uint8_t serialSynthesizePortBaud(serialPortIdentifier_e identifier, serialBaudClass_e baudClass);

