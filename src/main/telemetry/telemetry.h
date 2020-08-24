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

/*
 * telemetry.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */

#pragma once

#include "common/unit.h"

#include "io/serial.h"

#include "pg/pg.h"

#include "rx/rx.h"

#include "telemetry/ibus_shared.h"

typedef enum {
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;

typedef enum {
    SENSOR_VOLTAGE         = 1 << 0,
    SENSOR_CURRENT         = 1 << 1,
    SENSOR_FUEL            = 1 << 2,
    SENSOR_MODE            = 1 << 3,
    SENSOR_ACC_X           = 1 << 4,
    SENSOR_ACC_Y           = 1 << 5,
    SENSOR_ACC_Z           = 1 << 6,
    SENSOR_PITCH           = 1 << 7,
    SENSOR_ROLL            = 1 << 8,
    SENSOR_HEADING         = 1 << 9,
    SENSOR_ALTITUDE        = 1 << 10,
    SENSOR_VARIO           = 1 << 11,
    SENSOR_LAT_LONG        = 1 << 12,
    SENSOR_GROUND_SPEED    = 1 << 13,
    SENSOR_DISTANCE        = 1 << 14,
    ESC_SENSOR_CURRENT     = 1 << 15,
    ESC_SENSOR_VOLTAGE     = 1 << 16,
    ESC_SENSOR_RPM         = 1 << 17,
    ESC_SENSOR_TEMPERATURE = 1 << 18,
    ESC_SENSOR_ALL         = ESC_SENSOR_CURRENT \
                            | ESC_SENSOR_VOLTAGE \
                            | ESC_SENSOR_RPM \
                            | ESC_SENSOR_TEMPERATURE,
    SENSOR_TEMPERATURE     = 1 << 19,
    SENSOR_ALL             = (1 << 20) - 1,
} sensor_e;

typedef struct telemetryConfig_s {
    int16_t gpsNoFixLatitude;
    int16_t gpsNoFixLongitude;
    uint8_t telemetry_inverted;
    uint8_t halfDuplex;
    uint8_t frsky_coordinate_format;
    uint8_t frsky_unit;
    uint8_t frsky_vfas_precision;
    uint8_t hottAlarmSoundInterval;
    uint8_t pidValuesAsTelemetry;
    uint8_t report_cell_voltage;
    uint8_t flysky_sensors[IBUS_SENSOR_COUNT];
    uint16_t mavlink_mah_as_heading_divisor;
    uint32_t disabledSensors; // bit flags
} telemetryConfig_t;

PG_DECLARE(telemetryConfig_t, telemetryConfig);

extern serialPort_t *telemetrySharedPort;

void telemetryInit(void);
bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig, const SerialRXType serialrxProvider);

void telemetryCheckState(void);
void telemetryProcess(uint32_t currentTime);

bool telemetryDetermineEnabledState(portSharing_e portSharing);

bool telemetryIsSensorEnabled(sensor_e sensor);
