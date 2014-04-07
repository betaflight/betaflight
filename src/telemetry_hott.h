/*
 * telemetry_hott.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */

#ifndef TELEMETRY_HOTT_H_
#define TELEMETRY_HOTT_H_

/* HoTT module specifications */

#define HOTTV4_GENERAL_AIR_SENSOR_ID          0xD0

#define HOTTV4_ELECTRICAL_AIR_SENSOR_ID       0x8E // Electric Air Sensor ID
#define HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID  0xE0 // Electric Air Module ID

#define HOTTV4_GPS_SENSOR_ID                  0x8A // GPS Sensor ID
#define HOTTV4_GPS_SENSOR_TEXT_ID             0xA0 // GPS Module ID

#define HOTTV4_RXTX 4
#define HOTTV4_TX_DELAY 1000

#define HOTTV4_BUTTON_DEC    0xEB
#define HOTTV4_BUTTON_INC    0xED
#define HOTTV4_BUTTON_SET    0xE9
#define HOTTV4_BUTTON_NIL    0x0F
#define HOTTV4_BUTTON_NEXT   0xEE
#define HOTTV4_BUTTON_PREV   0xE7

#define OFFSET_HEIGHT       500
#define OFFSET_M2S           72
#define OFFSET_M3S          120

typedef enum {
    HoTTv4NotificationErrorCalibration     = 0x01,
    HoTTv4NotificationErrorReceiver        = 0x02,
    HoTTv4NotificationErrorDataBus         = 0x03,
    HoTTv4NotificationErrorNavigation      = 0x04,
    HoTTv4NotificationErrorError           = 0x05,
    HoTTv4NotificationErrorCompass         = 0x06,
    HoTTv4NotificationErrorSensor          = 0x07,
    HoTTv4NotificationErrorGPS             = 0x08,
    HoTTv4NotificationErrorMotor           = 0x09,

    HoTTv4NotificationMaxTemperature       = 0x0A,
    HoTTv4NotificationAltitudeReached      = 0x0B,
    HoTTv4NotificationWaypointReached      = 0x0C,
    HoTTv4NotificationNextWaypoint         = 0x0D,
    HoTTv4NotificationLanding              = 0x0E,
    HoTTv4NotificationGPSFix               = 0x0F,
    HoTTv4NotificationUndervoltage         = 0x10,
    HoTTv4NotificationGPSHold              = 0x11,
    HoTTv4NotificationGPSHome              = 0x12,
    HoTTv4NotificationGPSOff               = 0x13,
    HoTTv4NotificationBeep                 = 0x14,
    HoTTv4NotificationMicrocopter          = 0x15,
    HoTTv4NotificationCapacity             = 0x16,
    HoTTv4NotificationCareFreeOff          = 0x17,
    HoTTv4NotificationCalibrating          = 0x18,
    HoTTv4NotificationMaxRange             = 0x19,
    HoTTv4NotificationMaxAltitude          = 0x1A,

    HoTTv4Notification20Meter              = 0x25,
    HoTTv4NotificationMicrocopterOff       = 0x26,
    HoTTv4NotificationAltitudeOn           = 0x27,
    HoTTv4NotificationAltitudeOff          = 0x28,
    HoTTv4Notification100Meter             = 0x29,
    HoTTv4NotificationCareFreeOn           = 0x2E,
    HoTTv4NotificationDown                 = 0x2F,
    HoTTv4NotificationUp                   = 0x30,
    HoTTv4NotificationHold                 = 0x31,
    HoTTv4NotificationGPSOn                = 0x32,
    HoTTv4NotificationFollowing            = 0x33,
    HoTTv4NotificationStarting             = 0x34,
    HoTTv4NotificationReceiver             = 0x35,
} HoTTv4Notification;

/**
 * GPS
 * Receiver -> GPS Sensor (Flightcontrol)
 * Byte 1: 0x80 = Receiver byte
 * Byte 2: 0x8A = GPS Sensor byte (witch data Transmitter wants to get)
 * 5ms Idle Line!
 * 5ms delay
 */

typedef struct HoTTV4GPSModule_t {
    uint8_t startByte;               // Byte  1: 0x7C = Start byte data
    uint8_t sensorID;                // Byte  2: 0x8A = GPS Sensor
    uint8_t alarmTone;               // Byte  3: 0…= warning beeps
    uint8_t sensorTextID;            // Byte  4: 160 0xA0 Sensor ID Neu!
    uint8_t alarmInverse1;           // Byte  5: 01 inverse status
    uint8_t alarmInverse2;           // Byte  6: 00 inverse status status 1 = no GPS signal
    uint8_t flightDirection;         // Byte  7: 119 = fly direction. 1 = 2°; 0° (North), 9 0° (East), 180° (South), 270° (West)
    uint8_t GPSSpeedLow;             // Byte  8: 8 = GPS speed low byte 8km/h
    uint8_t GPSSpeedHigh;            // Byte  9: 0 = GPS speed high byte

    // Example: N 48°39'988"
    uint8_t LatitudeNS;              // Byte 10: 000 = N; 001 = S
    //          0x12E7 = 4839 (48°30')
    uint8_t LatitudeMinLow;          // Byte 11: 231 = 0xE7
    uint8_t LatitudeMinHigh;         // Byte 12: 018 = 0x12
    //          0x03DC = 0988 (988")
    uint8_t LatitudeSecLow;          // Byte 13: 220 = 0xDC
    uint8_t LatitudeSecHigh;         // Byte 14: 003 = 0x03

    // Example: E 9°25'9360"
    uint8_t longitudeEW;             // Byte 15: 000 = E; 001 = W;
    //          0x039D = 0925 (09°25')
    uint8_t longitudeMinLow;         // Byte 16: 157 = 0x9D
    uint8_t longitudeMinHigh;        // Byte 17: 003 = 0x03
    //          0x2490 = 9360 (9360")
    uint8_t longitudeSecLow;         // Byte 18: 144 = 0x90
    uint8_t longitudeSecHigh;        // Byte 19: 036 = 0x24

    uint8_t distanceLow;             // Byte 20: distance low byte (meter)
    uint8_t distanceHigh;            // Byte 21: distance high byte
    uint8_t altitudeLow;             // Byte 22: Altitude low byte (meter)
    uint8_t altitudeHigh;            // Byte 23: Altitude high byte
    uint8_t resolutionLow;           // Byte 24: Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s)
    uint8_t resolutionHigh;          // Byte 25: High Byte m/s resolution 0.01m

    uint8_t m3s;                     // Byte 26: 120 = 0m/3s
    uint8_t GPSNumSat;               // Byte 27: number of satelites (1 byte)
    uint8_t GPSFixChar;              // Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte)
    uint8_t HomeDirection;           // Byte 29: HomeDirection (direction from starting point to Model position) (1 byte)
    uint8_t angleXdirection;         // Byte 30: angle x-direction (1 byte)
    uint8_t angleYdirection;         // Byte 31: angle y-direction (1 byte)
    uint8_t angleZdirection;         // Byte 32: angle z-direction (1 byte)

    uint8_t gyroXLow;                // Byte 33: gyro x low byte (2 bytes)
    uint8_t gyroXHigh;               // Byte 34: gyro x high byte
    uint8_t gyroYLow;                // Byte 35: gyro y low byte (2 bytes)
    uint8_t gyroYHigh;               // Byte 36: gyro y high byte
    uint8_t gyroZLow;                // Byte 37: gyro z low byte (2 bytes)
    uint8_t gyroZHigh;               // Byte 38: gyro z high byte

    uint8_t vibration;               // Byte 39: vibration (1 bytes)
    uint8_t Ascii4;                  // Byte 40: 00 ASCII Free Character [4]
    uint8_t Ascii5;                  // Byte 41: 00 ASCII Free Character [5]
    uint8_t GPS_fix;                 // Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX
    uint8_t version;                 // Byte 43: 00 version number
    uint8_t endByte;                 // Byte 44: 0x7D End byte
} HoTTV4GPSModule_t;

/**
 * EAM (Electric Air Module) 33620
 * EmpfängerElectric Sensor
 * Byte 1: 80 = Receiver byte
 * Byte 2: 8E = Electric Sensor byte
 * 5ms Idle Line!
*/

typedef struct HoTTV4ElectricAirModule_t {
    uint8_t startByte;
    uint8_t sensorID;
    uint8_t alarmTone; // Alarm */
    uint8_t sensorTextID;
    uint8_t alarmInverse1;
    uint8_t alarmInverse2;

    uint8_t cell1L;              // Low Voltage Cell 1-7 in 2mV steps */
    uint8_t cell2L;
    uint8_t cell3L;
    uint8_t cell4L;
    uint8_t cell5L;
    uint8_t cell6L;
    uint8_t cell7L;
    uint8_t cell1H;              // High Voltage Cell 1-7 in 2mV steps */
    uint8_t cell2H;
    uint8_t cell3H;
    uint8_t cell4H;
    uint8_t cell5H;
    uint8_t cell6H;
    uint8_t cell7H;

    uint8_t battery1Low;         // Battetry 1 LSB/MSB in 100mv steps; 50 == 5V */
    uint8_t battery1High;        // Battetry 1 LSB/MSB in 100mv steps; 50 == 5V */
    uint8_t battery2Low;         // Battetry 2 LSB/MSB in 100mv steps; 50 == 5V */
    uint8_t battery2High;        // Battetry 2 LSB/MSB in 100mv steps; 50 == 5V */

    uint8_t temp1;               // Temp 1; Offset of 20. 20 == 0C */
    uint8_t temp2;               // Temp 2; Offset of 20. 20 == 0C */

    uint16_t height;             // Height. Offset -500. 500 == 0 */
    uint16_t current;            // 1 = 0.1A */
    uint8_t driveVoltageLow;
    uint8_t driveVoltageHigh;
    uint16_t capacity;           // mAh */
    uint16_t m2s;                // m2s; 0x48 == 0 */
    uint8_t m3s;                 // m3s; 0x78 == 0 */

    uint16_t rpm;                // RPM. 10er steps; 300 == 3000rpm */
    uint8_t minutes;
    uint8_t seconds;
    uint8_t speed;

    uint8_t version;
    uint8_t endByte;
} HoTTV4ElectricAirModule_t;

void handleHoTTTelemetry(void);
void checkTelemetryState(void);

void configureHoTTTelemetryPort(void);
void freeHoTTTelemetryPort(void);

#endif /* TELEMETRY_HOTT_H_ */
