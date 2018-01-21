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
 *
 * FlySky iBus telemetry implementation by CraigJPerry.
 * Unit tests and some additions by Unitware
 *
 * Many thanks to Dave Borthwick's iBus telemetry dongle converter for
 * PIC 12F1572 (also distributed under GPLv3) which was referenced to
 * clarify the protocol.
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
// #include <string.h>

#include "platform.h"
//#include "common/utils.h"
#include "telemetry/telemetry.h"
#include "telemetry/ibus_shared.h"

static uint16_t calculateChecksum(const uint8_t *ibusPacket);

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "sensors/battery.h"
#include "fc/rc_controls.h"
#include "fc/config.h"
#include "sensors/gyro.h"
#include "drivers/accgyro/accgyro.h"
#include "fc/runtime_config.h"
#include "sensors/acceleration.h"
#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "flight/imu.h"
#include "flight/altitude.h"
#include "flight/navigation.h"
#include "io/gps.h"


#define IBUS_TEMPERATURE_OFFSET     400
#define INVALID_IBUS_ADDRESS        0
#define IBUS_BUFFSIZE               33 // biggest iBus message seen so far + 1
#define IBUS_HEADER_FOOTER_SIZE     4
#define IBUS_2BYTE_SESNSOR          2
#define IBUS_4BYTE_SESNSOR          4

typedef uint8_t ibusAddress_t;

typedef enum {
    IBUS_COMMAND_DISCOVER_SENSOR      = 0x80,
    IBUS_COMMAND_SENSOR_TYPE          = 0x90,
    IBUS_COMMAND_MEASUREMENT          = 0xA0
} ibusCommand_e;

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

typedef union ibusTelemetry {
    uint16_t uint16;
    uint32_t uint32;
    int16_t int16;
    int32_t int32;
    uint8_t byte[4];
} ibusTelemetry_s;

#if defined(USE_GPS)

const uint8_t GPS_IDS[] = {
    IBUS_SENSOR_TYPE_GPS_STATUS,
    IBUS_SENSOR_TYPE_SPE,
    IBUS_SENSOR_TYPE_GPS_LAT,
    IBUS_SENSOR_TYPE_GPS_LON,
    IBUS_SENSOR_TYPE_GPS_ALT,
    IBUS_SENSOR_TYPE_GROUND_SPEED,
    IBUS_SENSOR_TYPE_ODO1,
    IBUS_SENSOR_TYPE_ODO2,
    IBUS_SENSOR_TYPE_GPS_DIST,
    IBUS_SENSOR_TYPE_COG,
};
#endif

#if defined(USE_TELEMETRY_IBUS_EXTENDED)

const uint8_t FULL_GPS_IDS[] = {
    IBUS_SENSOR_TYPE_GPS_STATUS,
    IBUS_SENSOR_TYPE_GPS_LAT,
    IBUS_SENSOR_TYPE_GPS_LON,
    IBUS_SENSOR_TYPE_GPS_ALT,
};

const uint8_t FULL_VOLT_IDS[] = {
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,
    IBUS_SENSOR_TYPE_CELL,
    IBUS_SENSOR_TYPE_BAT_CURR,
    IBUS_SENSOR_TYPE_FUEL,
    IBUS_SENSOR_TYPE_RPM,
};

const uint8_t FULL_ACC_IDS[] = {
    IBUS_SENSOR_TYPE_ACC_X,
    IBUS_SENSOR_TYPE_ACC_Y,
    IBUS_SENSOR_TYPE_ACC_Z,
    IBUS_SENSOR_TYPE_ROLL,
    IBUS_SENSOR_TYPE_PITCH,
    IBUS_SENSOR_TYPE_YAW,
};

#endif //defined(USE_TELEMETRY_IBUS_EXTENDED)

static serialPort_t *ibusSerialPort = NULL;
static ibusAddress_t ibusBaseAddress = INVALID_IBUS_ADDRESS;
static uint8_t sendBuffer[IBUS_BUFFSIZE];


static void setValue(uint8_t* bufferPtr, uint8_t sensorType, uint8_t length);

static uint8_t getSensorID(ibusAddress_t address)
{
    //all checks are done in theAddressIsWithinOurRange
    uint32_t index = address - ibusBaseAddress;
    return telemetryConfig()->flysky_sensors[index];
}

static uint8_t getSensorLength(uint8_t sensorID)
{
    if (sensorID == IBUS_SENSOR_TYPE_PRES || (sensorID >= IBUS_SENSOR_TYPE_GPS_LAT && sensorID <= IBUS_SENSOR_TYPE_ALT_MAX)) {
        return IBUS_4BYTE_SESNSOR;
    }
#if defined(USE_TELEMETRY_IBUS_EXTENDED)
    if (sensorID == IBUS_SENSOR_TYPE_GPS_FULL) {
        return 14;
    }
    if (sensorID == IBUS_SENSOR_TYPE_VOLT_FULL) {
        return 10;
    }
    if (sensorID == IBUS_SENSOR_TYPE_VOLT_FULL) {
        return 12;
    }
#endif
    return IBUS_2BYTE_SESNSOR;
}

static uint8_t transmitIbusPacket()
{
    unsigned frameLength = sendBuffer[0];
    if (frameLength == INVALID_IBUS_ADDRESS) {
        return 0;
    }
    unsigned payloadLength = frameLength - IBUS_CHECKSUM_SIZE;
    uint16_t checksum = calculateChecksum(sendBuffer);
    for (unsigned i = 0; i < payloadLength; i++) {
        serialWrite(ibusSerialPort, sendBuffer[i]);
    }
    serialWrite(ibusSerialPort, checksum & 0xFF);
    serialWrite(ibusSerialPort, checksum >> 8);
    return frameLength;
}

static void setIbusDiscoverSensorReply(ibusAddress_t address)
{
    sendBuffer[0] = IBUS_HEADER_FOOTER_SIZE;
    sendBuffer[1] = IBUS_COMMAND_DISCOVER_SENSOR | address;
}

static void setIbusSensorType(ibusAddress_t address)
{
    uint8_t sensorID = getSensorID(address);
    uint8_t sensorLength = getSensorLength(sensorID);
    sendBuffer[0] = IBUS_HEADER_FOOTER_SIZE + 2;
    sendBuffer[1] = IBUS_COMMAND_SENSOR_TYPE | address;
    sendBuffer[2] = sensorID;
    sendBuffer[3] = sensorLength;
}

static uint16_t getVoltage()
{
    uint16_t voltage = getBatteryVoltage() *10;
    if (telemetryConfig()->report_cell_voltage) {
        voltage /= getBatteryCellCount();
    }
    return voltage;
}

static uint16_t getTemperature()
{
    uint16_t temperature = gyroGetTemperature() * 10;
#if defined(USE_BARO)
    if (sensors(SENSOR_BARO)) {
        temperature = (uint16_t) ((baro.baroTemperature + 50) / 10);
    }
#endif
    return temperature + IBUS_TEMPERATURE_OFFSET;
}


static uint16_t getFuel()
{
    uint16_t fuel = 0;
    if (batteryConfig()->batteryCapacity > 0) {
        fuel = (uint16_t)calculateBatteryPercentageRemaining();
    } else {
        fuel = (uint16_t)constrain(getMAhDrawn(), 0, 0xFFFF);
    }
    return fuel;
}

static uint16_t getRPM()
{
    uint16_t rpm = 0;
    if (ARMING_FLAG(ARMED)) {
        const throttleStatus_e throttleStatus = calculateThrottleStatus();
        rpm = rcCommand[THROTTLE];  // / BLADE_NUMBER_DIVIDER;
        if (throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP)) rpm = 0;
    } else {
        rpm = (uint16_t)(batteryConfig()->batteryCapacity); //  / BLADE_NUMBER_DIVIDER
    }
    return rpm;
}

static uint16_t getMode()
{
    uint16_t flightMode = 1; //Acro
    if (FLIGHT_MODE(ANGLE_MODE)) {
         flightMode = 0; //Stab
    }
    if (FLIGHT_MODE(BARO_MODE)) {
         flightMode = 2; //AltHold
    }
    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        flightMode = 3; //Auto
    }
    if (FLIGHT_MODE(HEADFREE_MODE) || FLIGHT_MODE(MAG_MODE)) {
        flightMode = 4; //Guided! (there in no HEAD, MAG so use Guided)
    }
    if (FLIGHT_MODE(GPS_HOLD_MODE) && FLIGHT_MODE(BARO_MODE)) {
        flightMode = 5; //Loiter
    }
    if (FLIGHT_MODE(GPS_HOME_MODE)) {
        flightMode = 6; //RTL
    }
    if (FLIGHT_MODE(HORIZON_MODE)) {
        flightMode = 7; //Circle! (there in no horizon so use Circle)
    }
    if (FLIGHT_MODE(GPS_HOLD_MODE)) {
        flightMode = 8; //PosHold
    }
    if (FLIGHT_MODE(FAILSAFE_MODE)) {
        flightMode = 9; //Land
    }
    return flightMode;
}

static int16_t getACC(uint8_t index)
{
    return (int16_t)(((float)acc.accADC[index] / acc.dev.acc_1G) * 1000);
}

#if defined(USE_TELEMETRY_IBUS_EXTENDED)
static void setCombinedFrame(uint8_t* bufferPtr, const uint8_t* structure, uint8_t itemCount)
{
    uint8_t offset = 0;
    uint8_t size = 0;
    for (unsigned i = 0; i < itemCount; i++) {
        size = getSensorLength(structure[i]);
        setValue(bufferPtr + offset, structure[i], size);
        offset += size;
    }
}
#endif



#if defined(USE_GPS)
static bool setGPS(uint8_t sensorType, ibusTelemetry_s* value)
{
    bool result = false;
    for (unsigned i = 0; i < sizeof(GPS_IDS); i++) {
        if (sensorType == GPS_IDS[i]) {
            result = true;
            break;
        }
    }
    if (!result) return result;

    uint16_t gpsFixType = 0;
    uint16_t sats = 0;
    if (sensors(SENSOR_GPS)) {
        gpsFixType = !STATE(GPS_FIX) ? 1 : (gpsSol.numSat < 5 ? 2 : 3);
        sats = gpsSol.numSat;
        if (STATE(GPS_FIX) || sensorType == IBUS_SENSOR_TYPE_GPS_STATUS) {
            result = true;
            switch (sensorType) {
            case IBUS_SENSOR_TYPE_SPE:
                value->uint16 = gpsSol.groundSpeed * 36 / 100;
                break;
            case IBUS_SENSOR_TYPE_GPS_LAT:
                value->int32 = gpsSol.llh.lat;
                break;
            case IBUS_SENSOR_TYPE_GPS_LON:
                value->int32 = gpsSol.llh.lon;
                break;
            case IBUS_SENSOR_TYPE_GPS_ALT:
                value->int32 = (int32_t)gpsSol.llh.alt;
                break;
            case IBUS_SENSOR_TYPE_GROUND_SPEED:
                value->uint16 = gpsSol.groundSpeed;
                break;
            case IBUS_SENSOR_TYPE_ODO1:
            case IBUS_SENSOR_TYPE_ODO2:
            case IBUS_SENSOR_TYPE_GPS_DIST:
                value->uint16 = GPS_distanceToHome;
                break;
            case IBUS_SENSOR_TYPE_COG:
                value->uint16 = gpsSol.groundCourse * 100;
                break;
            case IBUS_SENSOR_TYPE_GPS_STATUS:
                value->byte[0] = gpsFixType;
                value->byte[1] = sats;
                break;
            }
        }
    }
    return result;
}
#endif //defined(USE_GPS)

static void setValue(uint8_t* bufferPtr, uint8_t sensorType, uint8_t length)
{
    ibusTelemetry_s value;

#if defined(USE_TELEMETRY_IBUS_EXTENDED)
    const uint8_t* structure = 0;
    uint8_t itemCount;
    if (sensorType == IBUS_SENSOR_TYPE_GPS_FULL) {
        structure = FULL_GPS_IDS;
        itemCount = sizeof(FULL_GPS_IDS);
    }
    if (sensorType == IBUS_SENSOR_TYPE_VOLT_FULL) {
        structure = FULL_VOLT_IDS;
        itemCount = sizeof(FULL_VOLT_IDS);
    }
    if (sensorType == IBUS_SENSOR_TYPE_ACC_FULL) {
        structure = FULL_ACC_IDS;
        itemCount = sizeof(FULL_ACC_IDS);
    }
    if (structure != 0) {
        setCombinedFrame(bufferPtr, structure, sizeof(itemCount));
        return;
    }
#endif //defined(USE_TELEMETRY_IBUS_EXTENDED)

#if defined(USE_GPS)
    if (setGPS(sensorType, &value)) {
        return;
    }
#endif //defined(USE_TELEMETRY_IBUS_EXTENDED)

    for (unsigned i = 0; i < length; i++) {
        bufferPtr[i] = value.byte[i] = 0;
    }
    switch (sensorType) {
        case IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE:
            value.uint16 = getVoltage();
            break;
        case IBUS_SENSOR_TYPE_TEMPERATURE:
            value.uint16 = getTemperature();
            break;
        case IBUS_SENSOR_TYPE_RPM_FLYSKY:
            value.int16 = (int16_t)rcCommand[THROTTLE];
            break;
        case IBUS_SENSOR_TYPE_FUEL:
            value.uint16 = getFuel();
            break;
        case IBUS_SENSOR_TYPE_RPM:
            value.uint16 = getRPM();
            break;
        case IBUS_SENSOR_TYPE_FLIGHT_MODE:
            value.uint16 = getMode();
            break;
        case IBUS_SENSOR_TYPE_CELL:
            value.uint16 = (uint16_t)(getBatteryAverageCellVoltage() *10);
            break;
        case IBUS_SENSOR_TYPE_BAT_CURR:
            value.uint16 = (uint16_t)getAmperage();
            break;
        case IBUS_SENSOR_TYPE_ACC_X:
        case IBUS_SENSOR_TYPE_ACC_Y:
        case IBUS_SENSOR_TYPE_ACC_Z:
            value.int16 = getACC(sensorType - IBUS_SENSOR_TYPE_ACC_X);
            break;
        case IBUS_SENSOR_TYPE_ROLL:
        case IBUS_SENSOR_TYPE_PITCH:
        case IBUS_SENSOR_TYPE_YAW:
            value.int16 = attitude.raw[sensorType - IBUS_SENSOR_TYPE_ROLL] *10;
            break;
        case IBUS_SENSOR_TYPE_ARMED:
            value.uint16 = ARMING_FLAG(ARMED) ? 0 : 1;
            break;
#if defined(USE_TELEMETRY_IBUS_EXTENDED)
        case IBUS_SENSOR_TYPE_CMP_HEAD:
            value.uint16 = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            break;
        case IBUS_SENSOR_TYPE_VERTICAL_SPEED:
        case IBUS_SENSOR_TYPE_CLIMB_RATE:
            if(sensors(SENSOR_SONAR) || sensors(SENSOR_BARO)) {
                value.int16 = (int16_t)getEstimatedVario();
            }
            break;
        case IBUS_SENSOR_TYPE_ALT:
        case IBUS_SENSOR_TYPE_ALT_MAX:
            value.int32 = baro.BaroAlt;
            break;
        case IBUS_SENSOR_TYPE_PRES:
            value.uint32 = baro.baroPressure | (((uint32_t)getTemperature()) << 19);
            break;
#endif //defined(TELEMETRY_IBUS_EXTENDED)
    }
    for (unsigned i = 0; i < length; i++) {
        bufferPtr[i] = value.byte[i];
    }
}
static void setIbusMeasurement(ibusAddress_t address)
{
    uint8_t sensorID = getSensorID(address);
    uint8_t sensorLength = getSensorLength(sensorID);
    sendBuffer[0] = IBUS_HEADER_FOOTER_SIZE + sensorLength;
    sendBuffer[1] = IBUS_COMMAND_MEASUREMENT | address;
    setValue(sendBuffer + 2, sensorID, sensorLength);
}

static bool isCommand(ibusCommand_e expected, const uint8_t *ibusPacket)
{
    return (ibusPacket[1] & 0xF0) == expected;
}

static ibusAddress_t getAddress(const uint8_t *ibusPacket)
{
    return (ibusPacket[1] & 0x0F);
}

static void autodetectFirstReceivedAddressAsBaseAddress(ibusAddress_t returnAddress)
{
    if ((INVALID_IBUS_ADDRESS == ibusBaseAddress) &&
    (INVALID_IBUS_ADDRESS != returnAddress)) {
        ibusBaseAddress = returnAddress;
    }
}

static bool theAddressIsWithinOurRange(ibusAddress_t returnAddress)
{
    return (returnAddress >= ibusBaseAddress) &&
    (ibusAddress_t)(returnAddress - ibusBaseAddress) < ARRAYLEN(telemetryConfig()->flysky_sensors) &&
    telemetryConfig()->flysky_sensors[(returnAddress - ibusBaseAddress)] != IBUS_SENSOR_TYPE_NONE;
}

uint8_t respondToIbusRequest(uint8_t const * const ibusPacket)
{
    ibusAddress_t returnAddress = getAddress(ibusPacket);
    autodetectFirstReceivedAddressAsBaseAddress(returnAddress);
    //set buffer to invalid
    sendBuffer[0] = INVALID_IBUS_ADDRESS;

    if (theAddressIsWithinOurRange(returnAddress)) {
        if (isCommand(IBUS_COMMAND_DISCOVER_SENSOR, ibusPacket)) {
            setIbusDiscoverSensorReply(returnAddress);
        } else if (isCommand(IBUS_COMMAND_SENSOR_TYPE, ibusPacket)) {
            setIbusSensorType(returnAddress);
        } else if (isCommand(IBUS_COMMAND_MEASUREMENT, ibusPacket)) {
            setIbusMeasurement(returnAddress);
        }
    }
    //transmit if content was set
    return transmitIbusPacket();
}


void initSharedIbusTelemetry(serialPort_t *port)
{
    ibusSerialPort = port;
    ibusBaseAddress = INVALID_IBUS_ADDRESS;
}


#endif //defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)

static uint16_t calculateChecksum(const uint8_t *ibusPacket)
{
    uint16_t checksum = 0xFFFF;
    uint8_t dataSize = ibusPacket[0] - IBUS_CHECKSUM_SIZE;
    for (unsigned i = 0; i < dataSize; i++) {
        checksum -= ibusPacket[i];
    }

    return checksum;
}

bool isChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length)
{
    uint16_t calculatedChecksum = calculateChecksum(ibusPacket);

    // Note that there's a byte order swap to little endian here
    return (calculatedChecksum >> 8) == ibusPacket[length - 1]
           && (calculatedChecksum & 0xFF) == ibusPacket[length - 2];
}
