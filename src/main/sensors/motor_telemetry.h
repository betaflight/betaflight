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

// Merged per-motor telemetry from all sources (DShot telemetry, ESC sensor).
//
// Consumers read one value per field and do not select a source. The merge
// rules are in motor_telemetry.c:
// - rpm: DShot telemetry first, then ESC sensor.
// - temperature, voltage, current, consumption: ESC sensor first (better
//   resolution), then DShot extended telemetry.
//
// Hot paths (RPM filter, dynamic idle, blackbox) continue to read DShot
// telemetry directly. This module is for the slow consumers (OSD, MSP,
// RX telemetry, battery meters).

typedef enum {
    MOTOR_TELEMETRY_SOURCE_DSHOT      = 1 << 0,
    MOTOR_TELEMETRY_SOURCE_ESC_SENSOR = 1 << 1, // serial ESC sensor or DroneCAN
} motorTelemetrySource_e;

typedef enum {
    MOTOR_TELEMETRY_RPM         = 1 << 0,
    MOTOR_TELEMETRY_TEMPERATURE = 1 << 1,
    MOTOR_TELEMETRY_VOLTAGE     = 1 << 2,
    MOTOR_TELEMETRY_CURRENT     = 1 << 3,
    MOTOR_TELEMETRY_CONSUMPTION = 1 << 4,
} motorTelemetryField_e;

typedef struct motorTelemetry_s {
    uint32_t rpm;           // mechanical RPM
    int32_t current;        // 0.01A
    int32_t consumption;    // mAh
    uint16_t voltage;       // 0.01V
    uint8_t temperature;    // degrees Celsius
    uint8_t dataAge;        // 0 = fresh, ESC_DATA_INVALID = no data
    uint8_t valid;          // motorTelemetryField_e bits
    uint8_t sources;        // motorTelemetrySource_e bits
} motorTelemetry_t;

// True if at least one telemetry source is enabled
bool motorTelemetryIsEnabled(void);

// Telemetry of one motor. Returns NULL if the index is not valid or no source is enabled.
const motorTelemetry_t *motorTelemetryGet(unsigned motorIndex);

// Telemetry of all motors:
// - rpm and voltage: average of the motors with a valid value
// - temperature: maximum
// - current and consumption: sum
// - dataAge: maximum over all motors
// Returns NULL if no source is enabled.
const motorTelemetry_t *motorTelemetryGetCombined(void);

static inline bool motorTelemetryHas(const motorTelemetry_t *telemetry, motorTelemetryField_e field)
{
    return telemetry && (telemetry->valid & field);
}
