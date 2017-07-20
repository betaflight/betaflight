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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "platform.h"
#include "telemetry/ibus_shared.h"

#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/serial.h"

#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "scheduler/scheduler.h"

#include "io/serial.h"

#include "sensors/barometer.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "flight/imu.h"
#include "flight/failsafe.h"

#include "navigation/navigation.h"

#include "telemetry/ibus.h"
#include "telemetry/telemetry.h"
#include "fc/config.h"
#include "config/feature.h"
#include "io/gps.h"
#define IBUS_TEMPERATURE_OFFSET (0x0190)

typedef uint8_t ibusAddress_t;

typedef enum {
    IBUS_COMMAND_DISCOVER_SENSOR      = 0x80,
    IBUS_COMMAND_SENSOR_TYPE          = 0x90,
    IBUS_COMMAND_MEASUREMENT          = 0xA0
} ibusCommand_e;

static uint8_t SENSOR_ADDRESS_TYPE_LOOKUP[] = {
    IBUS_MEAS_TYPE_INTERNAL_VOLTAGE,  // Address 0, sensor 1, not usable since it is reserved for internal voltage
    IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE,  // Address 1 ,sensor 2, VBAT
    IBUS_MEAS_TYPE_TEMPERATURE,       // Address 2, sensor 3, Baro/Gyro Temp
    IBUS_MEAS_TYPE_RPM,               // Address 3, sensor 4, Status AS RPM
    IBUS_MEAS_TYPE_RPM,               // Address 4, sensor 5, MAG_COURSE in deg AS RPM
    IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE,  // Address 5, sensor 6, Current in A AS ExtV
    IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE   // Address 6, sensor 7, Baro Alt in cm AS ExtV
#if defined(GPS)
   ,IBUS_MEAS_TYPE_RPM,               // Address 7, sensor 8, HOME_DIR in deg AS RPM
    IBUS_MEAS_TYPE_RPM,               // Address 8, sensor 9, HOME_DIST in m AS RPM
    IBUS_MEAS_TYPE_RPM,               // Address 9, sensor 10,GPS_COURSE in deg AS RPM
    IBUS_MEAS_TYPE_RPM,               // Address 10,sensor 11,GPS_ALT in m AS RPM
    IBUS_MEAS_TYPE_RPM,               // Address 11,sensor 12,GPS_LAT2 AS RPM 5678
    IBUS_MEAS_TYPE_RPM,               // Address 12,sensor 13,GPS_LON2 AS RPM 6789
    IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE,  // Address 13,sensor 14,GPS_LAT1 AS ExtV -12.45 (-12.3456789 N)
    IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE,  // Address 14,sensor 15,GPS_LON1 AS ExtV -123.45 (-123.4567890 E)
    IBUS_MEAS_TYPE_RPM                // Address 15,sensor 16,GPS_SPEED in km/h AS RPM
#endif
    //IBUS_MEAS_TYPE_TX_VOLTAGE,
    //IBUS_MEAS_TYPE_ERR
};

static serialPort_t *ibusSerialPort = NULL;

static uint8_t transmitIbusPacket(uint8_t ibusPacket[static IBUS_MIN_LEN], size_t packetLength) {
    uint16_t checksum = ibusCalculateChecksum(ibusPacket, packetLength);
    ibusPacket[packetLength - IBUS_CHECKSUM_SIZE] = (checksum & 0xFF);
    ibusPacket[packetLength - IBUS_CHECKSUM_SIZE + 1] = (checksum >> 8);
    for (size_t i = 0; i < packetLength; i++) {
        serialWrite(ibusSerialPort, ibusPacket[i]);
    }
    return packetLength;
}

static uint8_t sendIbusCommand(ibusAddress_t address) {
    uint8_t sendBuffer[] = { 0x04, IBUS_COMMAND_DISCOVER_SENSOR | address, 0x00, 0x00 };
    return transmitIbusPacket(sendBuffer, sizeof sendBuffer);
}

static uint8_t sendIbusSensorType(ibusAddress_t address) {
    uint8_t sendBuffer[] = { 0x06, IBUS_COMMAND_SENSOR_TYPE | address, SENSOR_ADDRESS_TYPE_LOOKUP[address], 0x02, 0x0, 0x0 };
    return transmitIbusPacket(sendBuffer, sizeof sendBuffer);
}

static uint8_t sendIbusMeasurement(ibusAddress_t address, uint16_t measurement) {
    uint8_t sendBuffer[] = { 0x06, IBUS_COMMAND_MEASUREMENT | address, measurement & 0xFF, measurement >> 8, 0x0, 0x0 };
    return transmitIbusPacket(sendBuffer, sizeof sendBuffer);
}

static bool isCommand(ibusCommand_e expected, uint8_t ibusPacket[static IBUS_MIN_LEN]) {
    return (ibusPacket[1] & 0xF0) == expected;
}

static ibusAddress_t getAddress(uint8_t ibusPacket[static IBUS_MIN_LEN]) {
    return (ibusPacket[1] & 0x0F);
}

// MANUAL, ACRO, ANGLE, HRZN, ALTHOLD, POSHOLD, RTH, WP, LAUNCH, FAILSAFE
static uint8_t flightModeToIBusTelemetryMode[FLM_COUNT] = { 0, 1, 3, 2, 5, 6, 7, 4, 8, 9 };

static uint8_t dispatchMeasurementRequest(ibusAddress_t address) {
    if (address == 1) { //2. VBAT
        return sendIbusMeasurement(address, vbat * 10);
    } else if (address == 2) { //3. BARO_TEMP\GYRO_TEMP
        if (sensors(SENSOR_BARO)) {
            return sendIbusMeasurement(address, (uint16_t) ((baro.baroTemperature + 50) / 10  + IBUS_TEMPERATURE_OFFSET)); //int32_t
        } else {
            /*
             * There is no temperature data
             * assuming (baro.baroTemperature + 50) / 10
             * 0 degrees (no sensor) equals 50 / 10 = 5
             */
            return sendIbusMeasurement(address, (uint16_t) (5 + IBUS_TEMPERATURE_OFFSET)); //int16_t
        }
    } else if (address == 3) { //4. STATUS (sat num AS #0, FIX AS 0, HDOP AS 0, Mode AS 0)
        int16_t status = flightModeToIBusTelemetryMode[getFlightModeForTelemetry()];
#if defined(GPS)
        if (sensors(SENSOR_GPS)) {
            status += gpsSol.numSat * 1000;
            if (gpsSol.fixType == GPS_NO_FIX) status += 100;
            else if (gpsSol.fixType == GPS_FIX_2D) status += 200;
            else if (gpsSol.fixType == GPS_FIX_3D) status += 300;
            if (STATE(GPS_FIX_HOME)) status += 500; {
                status += constrain(gpsSol.hdop / 1000, 0, 9) * 10;
            }
        }
#endif
        return sendIbusMeasurement(address, (uint16_t) status);
    } else if (address == 4) { //5. MAG_COURSE (0-360*, 0=north) //In ddeg ==> deg, 10ddeg = 1deg
        return sendIbusMeasurement(address, (uint16_t) (attitude.values.yaw / 10));
    } else if (address == 5) { //6. CURR //In 10*mA, 1 = 10 mA
        if (feature(FEATURE_CURRENT_METER)) return sendIbusMeasurement(address, (uint16_t) amperage); //int32_t
        else return sendIbusMeasurement(address, 0);
    } else if (address == 6) { //7. BARO_ALT //In cm => m
        if (sensors(SENSOR_BARO)) return sendIbusMeasurement(address, (uint16_t) baro.BaroAlt); //int32_t
        else return sendIbusMeasurement(address, 0);
    }
#if defined(GPS)
      else if (address == 7) { //8. HOME_DIR (0-360deg, 0=head)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement(address, (uint16_t) GPS_directionToHome); //int16_t
        else return sendIbusMeasurement(address, 0);
    } else if (address == 8) { //9. HOME_DIST //In m
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement(address, (uint16_t) GPS_distanceToHome); //uint16_t
        else return sendIbusMeasurement(address, 0);
    } else if (address == 9) { //10.GPS_COURSE (0-360deg, 0=north)
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement(address, (uint16_t) (gpsSol.groundCourse / 10)); //int16_t
        else return sendIbusMeasurement(address, 0);
    } else if (address == 10) { //11. GPS_ALT //In cm => m
        if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) return sendIbusMeasurement(address, (uint16_t) (gpsSol.llh.alt / 100));
        else return sendIbusMeasurement(address, 0);
    } else if (address == 11) { //12. GPS_LAT2 //Lattitude * 1e+7
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement(address, (uint16_t) ((gpsSol.llh.lat % 100000)/10));
        else return sendIbusMeasurement(address, 0);
    } else if (address == 12) { //13. GPS_LON2 //Longitude * 1e+7
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement(address, (uint16_t) ((gpsSol.llh.lon % 100000)/10));
        else return sendIbusMeasurement(address, 0);
    } else if (address == 13) { //14. GPS_LAT1 //Lattitude * 1e+7
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement(address, (uint16_t) (gpsSol.llh.lat / 100000));
        else return sendIbusMeasurement(address, 0);
    } else if (address == 14) { //15. GPS_LON1 //Longitude * 1e+7
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement(address, (uint16_t) (gpsSol.llh.lon / 100000));
        else return sendIbusMeasurement(address, 0);
    } else if (address == 15) { //16. GPS_SPEED //In cm/s => km/h, 1cm/s = 0.0194384449 knots
        if (sensors(SENSOR_GPS)) return sendIbusMeasurement(address, (uint16_t) gpsSol.groundSpeed * 1944 / 10000); //int16_t
        else return sendIbusMeasurement(address, 0);
    }
#endif
    else return 0;
}

uint8_t respondToIbusRequest(uint8_t ibusPacket[static IBUS_RX_BUF_LEN]) {
    ibusAddress_t returnAddress = getAddress(ibusPacket);
    if (returnAddress < sizeof SENSOR_ADDRESS_TYPE_LOOKUP) {
        if (isCommand(IBUS_COMMAND_DISCOVER_SENSOR, ibusPacket)) {
            return sendIbusCommand(returnAddress);
        } else if (isCommand(IBUS_COMMAND_SENSOR_TYPE, ibusPacket)) {
            return sendIbusSensorType(returnAddress);
        } else if (isCommand(IBUS_COMMAND_MEASUREMENT, ibusPacket)) {
            return dispatchMeasurementRequest(returnAddress);
        }
    }
    return 0;
}

void initSharedIbusTelemetry(serialPort_t *port) {
    ibusSerialPort = port;
}

void changeTypeIbusTelemetry(uint8_t id, uint8_t type) {
    SENSOR_ADDRESS_TYPE_LOOKUP[id] = type;
}

#endif //defined(TELEMETRY) && defined(TELEMETRY_IBUS)
