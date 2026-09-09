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
    TELEMETRY_PROTOCOL_NONE = 0,
    TELEMETRY_PROTOCOL_FRSKY_HUB,
    TELEMETRY_PROTOCOL_HOTT,
    TELEMETRY_PROTOCOL_LTM,
    TELEMETRY_PROTOCOL_SMARTPORT,
    TELEMETRY_PROTOCOL_MAVLINK,
    TELEMETRY_PROTOCOL_IBUS,
    TELEMETRY_PROTOCOL_COUNT
} telemetryProtocol_e;

typedef struct telemetryProvider_s {
    uint8_t protocol;  // telemetryProtocol_e
    int8_t uart;       // serialPortIdentifier_e; SERIAL_PORT_NONE = unused slot
    uint8_t baud;      // baudRate_e index; BAUD_AUTO lets the protocol pick its own default
} telemetryProvider_t;

#ifdef USE_TELEMETRY_FRSKY_HUB
#define TELEMETRY_HAS_FRSKY_HUB 1
#else
#define TELEMETRY_HAS_FRSKY_HUB 0
#endif
#ifdef USE_TELEMETRY_HOTT
#define TELEMETRY_HAS_HOTT 1
#else
#define TELEMETRY_HAS_HOTT 0
#endif
#ifdef USE_TELEMETRY_LTM
#define TELEMETRY_HAS_LTM 1
#else
#define TELEMETRY_HAS_LTM 0
#endif
#ifdef USE_TELEMETRY_SMARTPORT
#define TELEMETRY_HAS_SMARTPORT 1
#else
#define TELEMETRY_HAS_SMARTPORT 0
#endif
#ifdef USE_TELEMETRY_MAVLINK
#define TELEMETRY_HAS_MAVLINK 1
#else
#define TELEMETRY_HAS_MAVLINK 0
#endif
#ifdef USE_TELEMETRY_IBUS
#define TELEMETRY_HAS_IBUS 1
#else
#define TELEMETRY_HAS_IBUS 0
#endif

#define TELEMETRY_PROVIDERS_AVAILABLE (TELEMETRY_HAS_FRSKY_HUB + TELEMETRY_HAS_HOTT + TELEMETRY_HAS_LTM \
                                     + TELEMETRY_HAS_SMARTPORT + TELEMETRY_HAS_MAVLINK + TELEMETRY_HAS_IBUS)

// Concurrent telemetry protocols. Well short of one slot per protocol, which no
// craft needs and which costs a settings entry per slot. A target may override
// the default, but never past what it actually builds in.
#ifndef MAX_TELEMETRY_PROVIDERS_DEFAULT
#define MAX_TELEMETRY_PROVIDERS_DEFAULT 3
#endif

#if TELEMETRY_PROVIDERS_AVAILABLE < MAX_TELEMETRY_PROVIDERS_DEFAULT
#define MAX_TELEMETRY_PROVIDERS TELEMETRY_PROVIDERS_AVAILABLE
#else
#define MAX_TELEMETRY_PROVIDERS MAX_TELEMETRY_PROVIDERS_DEFAULT
#endif

// A build can have telemetry without any slot-based protocol, since CRSF and
// GHST ride the receiver's own port and never claim one.
#if MAX_TELEMETRY_PROVIDERS > 0
#define USE_TELEMETRY_PROVIDERS
#endif

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
    SENSOR_CAP_USED        = 1 << 20,
    SENSOR_ALL             = (1 << 21) - 1,
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
    uint8_t mavlink_min_txbuff; // The min TX buffer space value to send mavlink telemetry data 1...100%
    uint8_t mavlink_extended_status_rate;
    uint8_t mavlink_rc_channels_rate;
    uint8_t mavlink_position_rate;
    uint8_t mavlink_extra1_rate;
    uint8_t mavlink_extra2_rate;
    uint8_t mavlink_extra3_rate;
    uint8_t crsf_tlm_accgyro;
    telemetryProvider_t providers[MAX_TELEMETRY_PROVIDERS];
} telemetryConfig_t;

PG_DECLARE(telemetryConfig_t, telemetryConfig);

extern serialPort_t *telemetrySharedPort;

// Port assigned to a telemetry protocol, or SERIAL_PORT_NONE if unassigned.
serialPortIdentifier_e telemetryProviderPort(uint8_t protocol);
void telemetryValidateProviders(void);

// Baud configured for a given protocol on a given port, or BAUD_AUTO when no
// slot matches.  Lets a consumer that shares a port with a telemetry provider
// (serial MAVLink RX) pick up that provider's rate without reading the mask.
uint8_t telemetryProviderBaud(uint8_t protocol, serialPortIdentifier_e identifier);

void telemetryInit(void);
bool telemetryCheckRxPortShared(serialPortIdentifier_e identifier, const SerialRXType serialrxProvider);

void telemetryCheckState(void);
void telemetryProcess(uint32_t currentTime);

bool telemetryDetermineEnabledState(portSharing_e portSharing);

bool telemetryIsSensorEnabled(sensor_e sensor);
