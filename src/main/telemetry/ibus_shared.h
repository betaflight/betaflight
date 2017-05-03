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

#include "io/serial.h"

#define IBUS_CHECKSUM_SIZE (2)
#define IBUS_TASK_PERIOD_US (500)
#define IBUS_BAUDRATE      (115200)
#define IBUS_CYCLE_TIME_MS (8)
#define IBUS_MIN_LEN       (2 + IBUS_CHECKSUM_SIZE)
#define IBUS_MAX_TX_LEN    (6)
#define IBUS_MAX_RX_LEN    (4)
#define IBUS_RX_BUF_LEN    (IBUS_MAX_RX_LEN)

#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)

typedef enum {
    IBUS_MEAS_TYPE_INTERNAL_VOLTAGE = 0x00, // Internal Voltage
    IBUS_MEAS_TYPE_TEMPERATURE      = 0x01, // Temperature -##0.0 C, 0=-40.0 C, 400=0.0 C, 65535=6513.5 C
    IBUS_MEAS_TYPE_RPM              = 0x02, // Rotation RPM, ####0RPM, 0=0RPM, 65535=65535RPM
    IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE = 0x03, // External Voltage, -##0.00V, 0=0.00V, 32767=327.67V, 32768=na, 32769=-327.67V, 65535=-0.01V
    IBUS_MEAS_TYPE_PRES             = 0x41, // Pressure, not work
    IBUS_MEAS_TYPE_ODO1             = 0x7c, // Odometer1, 0.0km, 0.0 only
    IBUS_MEAS_TYPE_ODO2             = 0x7d, // Odometer2, 0.0km, 0.0 only
    IBUS_MEAS_TYPE_SPE              = 0x7e, // Speed km/h, ###0km/h, 0=0Km/h, 1000=100Km/h
    IBUS_MEAS_TYPE_ALT              = 0xf9, // Altitude m, not work
    IBUS_MEAS_TYPE_SNR              = 0xfa, // SNR, not work
    IBUS_MEAS_TYPE_NOISE            = 0xfb, // Noise, not work
    IBUS_MEAS_TYPE_RSSI             = 0xfc, // RSSI, not work
    IBUS_MEAS_TYPE_ERR              = 0xfe  // Error rate, #0%
} ibusSensorType_e;

uint8_t respondToIbusRequest(uint8_t ibusPacket[static IBUS_RX_BUF_LEN]);
void initSharedIbusTelemetry(serialPort_t *port);
void changeTypeIbusTelemetry(uint8_t id, uint8_t type);

#endif //defined(TELEMETRY) && defined(TELEMETRY_IBUS)

bool ibusIsChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length);
uint16_t ibusCalculateChecksum(const uint8_t *ibusPacket, size_t packetLength);
