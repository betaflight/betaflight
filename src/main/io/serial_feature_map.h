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

// Apply a legacy bitmask to the feature PGs.  Used by MSP_SET_CF_SERIAL_CONFIG
// and the CLI `serial` command's write path, and by the one-shot EEPROM
// migration on config load.  Returns false when the requested combination
// cannot be represented (e.g. more MSP ports requested than slots available).
bool serialApplyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask);

// Walk serialConfig.portConfigs[] and populate the per-feature PG fields
// from each port's legacy functionMask.  Idempotent.  Called once after
// EEPROM load so pre-existing configs carry their function assignments
// into the new per-feature view while the legacy mask is still the
// runtime source of truth.
void serialBackfillFeatureFields(void);

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

// Write a legacy per-port baud back onto whichever feature PGs claim the
// port for that class.  Telemetry writes every provider slot on the port,
// preserving the legacy one-baud-per-port semantics.
void serialApplyPortBaud(serialPortIdentifier_e identifier, serialBaudClass_e baudClass, uint8_t baudRateIndex);
