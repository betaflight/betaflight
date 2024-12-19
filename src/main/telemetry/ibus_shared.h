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
 * The ibus_shared module implements the ibus telemetry packet handling
 * which is shared between the ibus serial rx and the ibus telemetry.
 *
 * There is only one 'user' active at any time, serial rx will open the
 * serial port if both functions are enabled at the same time
 */

#pragma once

#include "platform.h"
#include "drivers/serial.h"

#define IBUS_CHECKSUM_SIZE (2)
#define IBUS_SENSOR_COUNT 15

typedef enum {
    IBUS_SENSOR_TYPE_NONE             = 0x00,
    IBUS_SENSOR_TYPE_TEMPERATURE      = 0x01,
    IBUS_SENSOR_TYPE_RPM_FLYSKY       = 0x02,
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE = 0x03,
    IBUS_SENSOR_TYPE_CELL             = 0x04, // Avg Cell voltage
    IBUS_SENSOR_TYPE_BAT_CURR         = 0x05, // battery current A * 100
    IBUS_SENSOR_TYPE_FUEL             = 0x06, // remaining battery percentage / mah drawn otherwise or fuel level no unit!
    IBUS_SENSOR_TYPE_RPM              = 0x07, // throttle value / battery capacity
    IBUS_SENSOR_TYPE_CMP_HEAD         = 0x08, //Heading  0..360 deg, 0=north 2bytes
    IBUS_SENSOR_TYPE_CLIMB_RATE       = 0x09, //2 bytes m/s *100
    IBUS_SENSOR_TYPE_COG              = 0x0a, //2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
    IBUS_SENSOR_TYPE_GPS_STATUS       = 0x0b, //2 bytes
    IBUS_SENSOR_TYPE_ACC_X            = 0x0c, //2 bytes m/s *100 signed
    IBUS_SENSOR_TYPE_ACC_Y            = 0x0d, //2 bytes m/s *100 signed
    IBUS_SENSOR_TYPE_ACC_Z            = 0x0e, //2 bytes m/s *100 signed
    IBUS_SENSOR_TYPE_ROLL             = 0x0f, //2 bytes deg *100 signed
    IBUS_SENSOR_TYPE_PITCH            = 0x10, //2 bytes deg *100 signed
    IBUS_SENSOR_TYPE_YAW              = 0x11, //2 bytes deg *100 signed
    IBUS_SENSOR_TYPE_VERTICAL_SPEED   = 0x12, //2 bytes m/s *100
    IBUS_SENSOR_TYPE_GROUND_SPEED     = 0x13, //2 bytes m/s *100 different unit than build-in sensor
    IBUS_SENSOR_TYPE_GPS_DIST         = 0x14, //2 bytes dist from home m unsigned
    IBUS_SENSOR_TYPE_ARMED            = 0x15, //2 bytes
    IBUS_SENSOR_TYPE_FLIGHT_MODE      = 0x16, //2 bytes
    IBUS_SENSOR_TYPE_PRES             = 0x41, // Pressure
    IBUS_SENSOR_TYPE_ODO1             = 0x7c, // Odometer1
    IBUS_SENSOR_TYPE_ODO2             = 0x7d, // Odometer2
    IBUS_SENSOR_TYPE_SPE              = 0x7e, // Speed 2bytes km/h

    IBUS_SENSOR_TYPE_GPS_LAT          = 0x80, //4bytes signed WGS84 in degrees * 1E7
    IBUS_SENSOR_TYPE_GPS_LON          = 0x81, //4bytes signed WGS84 in degrees * 1E7
    IBUS_SENSOR_TYPE_GPS_ALT          = 0x82, //4bytes signed!!! GPS alt m*100
    IBUS_SENSOR_TYPE_ALT              = 0x83, //4bytes signed!!! Alt m*100
    IBUS_SENSOR_TYPE_ALT_MAX          = 0x84, //4bytes signed MaxAlt m*100

    IBUS_SENSOR_TYPE_ALT_FLYSKY       = 0xf9, // Altitude 2 bytes signed in m
#if defined(USE_TELEMETRY_IBUS_EXTENDED)
    IBUS_SENSOR_TYPE_GPS_FULL         = 0xfd,
    IBUS_SENSOR_TYPE_VOLT_FULL        = 0xf0,
    IBUS_SENSOR_TYPE_ACC_FULL         = 0xef,
#endif //defined(TELEMETRY_IBUS_EXTENDED)
    IBUS_SENSOR_TYPE_UNKNOWN          = 0xff
} ibusSensorType_e;

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)

uint8_t respondToIbusRequest(uint8_t const * const ibusPacket);
void initSharedIbusTelemetry(serialPort_t * port);

#endif //defined(TELEMETRY) && defined(TELEMETRY_IBUS)

bool isChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length);
