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
#include <string.h>

#include <limits.h>

extern "C" {
    #include "debug.h"

    #include "platform.h"

    #include "common/axis.h"

    #include "drivers/system.h"
    #include "drivers/serial.h"

    #include "sensors/sensors.h"
    #include "sensors/battery.h"

    #include "io/serial.h"
    #include "io/gps.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/hott.h"

    #include "flight/pid.h"
    #include "flight/gps_conversion.h"

    #include "config/runtime_config.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    void addGPSCoordinates(HOTT_GPS_MSG_t *hottGPSMessage, int32_t latitude, int32_t longitude);
}
// See http://en.wikipedia.org/wiki/Geographic_coordinate_conversion

HOTT_GPS_MSG_t hottGPSMessage;

HOTT_GPS_MSG_t *getGPSMessageForTest(void)
{
    memset(&hottGPSMessage, 0, sizeof(hottGPSMessage));
    return &hottGPSMessage;
}

TEST(TelemetryHottTest, UpdateGPSCoordinates1)
{
    // given
    HOTT_GPS_MSG_t *hottGPSMessage = getGPSMessageForTest();

    // Mayrhofen, Austria
    uint32_t longitude = GPS_coord_to_degrees("4710.5186");
    uint32_t latitude = GPS_coord_to_degrees("1151.4252");

    // when
    addGPSCoordinates(hottGPSMessage, latitude, longitude);

    // then
    EXPECT_EQ(hottGPSMessage->pos_NS, 0);
    EXPECT_EQ(hottGPSMessage->pos_NS_dm_H << 8 | hottGPSMessage->pos_NS_dm_L, 1151);
    EXPECT_EQ((int16_t)(hottGPSMessage->pos_NS_sec_H << 8 | hottGPSMessage->pos_NS_sec_L), 4251);

    EXPECT_EQ(hottGPSMessage->pos_EW, 0);
    EXPECT_EQ(hottGPSMessage->pos_EW_dm_H << 8 | hottGPSMessage->pos_EW_dm_L, 4710);
    EXPECT_EQ((int16_t)(hottGPSMessage->pos_EW_sec_H << 8 | hottGPSMessage->pos_EW_sec_L), 5186);
}

TEST(TelemetryHottTest, UpdateGPSCoordinates2)
{
    // given
    HOTT_GPS_MSG_t *hottGPSMessage = getGPSMessageForTest();

    // Hampstead Heath, London
    // 51.563886, -0.159960
    uint32_t longitude = GPS_coord_to_degrees("5156.3886");
    uint32_t latitude  = -GPS_coord_to_degrees("015.9960");

    // when
    addGPSCoordinates(hottGPSMessage, longitude, latitude);

    // then
    EXPECT_EQ(hottGPSMessage->pos_NS, 0);
    EXPECT_EQ(hottGPSMessage->pos_NS_dm_H << 8 | hottGPSMessage->pos_NS_dm_L, 5156);
    EXPECT_EQ(hottGPSMessage->pos_NS_sec_H << 8 | hottGPSMessage->pos_NS_sec_L, 3886);

    EXPECT_EQ(hottGPSMessage->pos_EW, 1);
    EXPECT_EQ((int16_t)(hottGPSMessage->pos_EW_dm_H << 8 | hottGPSMessage->pos_EW_dm_L), -15);
    EXPECT_EQ((int16_t)(hottGPSMessage->pos_EW_sec_H << 8 | hottGPSMessage->pos_EW_sec_L), -9960);
}


TEST(TelemetryHottTest, UpdateGPSCoordinates3)
{
    // given
    HOTT_GPS_MSG_t *hottGPSMessage = getGPSMessageForTest();

    int32_t longitude = -GPS_coord_to_degrees("17999.9999");
    int32_t latitude = GPS_coord_to_degrees("8999.9999");

    // when
    addGPSCoordinates(hottGPSMessage, longitude, latitude);

    // then
    EXPECT_EQ(hottGPSMessage->pos_NS, 1);
    EXPECT_EQ((int16_t)(hottGPSMessage->pos_NS_dm_H << 8 | hottGPSMessage->pos_NS_dm_L), -18039);
    EXPECT_EQ((int16_t)(hottGPSMessage->pos_NS_sec_H << 8 | hottGPSMessage->pos_NS_sec_L), -9999);

    EXPECT_EQ(hottGPSMessage->pos_EW, 0);
    EXPECT_EQ((int16_t)(hottGPSMessage->pos_EW_dm_H << 8 | hottGPSMessage->pos_EW_dm_L), 9039);
    EXPECT_EQ((int16_t)(hottGPSMessage->pos_EW_sec_H << 8 | hottGPSMessage->pos_EW_sec_L), 9999);
}

TEST(TelemetryHottTest, PrepareGPSMessage_Altitude1m)
{
    // given
    HOTT_GPS_MSG_t *hottGPSMessage = getGPSMessageForTest();

    stateFlags = GPS_FIX;
    uint16_t altitudeInMeters = 1;
    GPS_altitude = altitudeInMeters * (1 / 0.1f); // 1 = 0.1m

    // when
    hottPrepareGPSResponse(hottGPSMessage);

    // then
    EXPECT_EQ((int16_t)(hottGPSMessage->altitude_H << 8 | hottGPSMessage->altitude_L), 1 + HOTT_GPS_ALTITUDE_OFFSET);
}


// STUBS

extern "C" {

int16_t debug[DEBUG16_VALUE_COUNT];

uint8_t stateFlags;

uint16_t batteryWarningVoltage;
uint8_t useHottAlarmSoundPeriod (void) { return 0; }


uint8_t GPS_numSat;
int32_t GPS_coord[2];
uint16_t GPS_speed;                 // speed in 0.1m/s
uint16_t GPS_distanceToHome;        // distance to home point in meters
uint16_t GPS_altitude;              // altitude in 0.1m
uint16_t vbat;
int16_t GPS_directionToHome;        // direction to home or hol point in degrees

int32_t amperage;
int32_t mAhDrawn;

uint32_t fixedMillis = 0;

uint32_t millis(void) {
    return fixedMillis;
}

uint32_t micros(void) { return 0; }

uint8_t serialRxBytesWaiting(serialPort_t *instance) {
    UNUSED(instance);
    return 0;
}

uint8_t serialTxBytesFree(serialPort_t *instance) {
    UNUSED(instance);
    return 0;
}

uint8_t serialRead(serialPort_t *instance) {
    UNUSED(instance);
    return 0;
}

void serialWrite(serialPort_t *instance, uint8_t ch) {
    UNUSED(instance);
    UNUSED(ch);
}

void serialSetMode(serialPort_t *instance, portMode_t mode) {
    UNUSED(instance);
    UNUSED(mode);
}


serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e functionMask, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options) {
    UNUSED(identifier);
    UNUSED(functionMask);
    UNUSED(baudRate);
    UNUSED(callback);
    UNUSED(mode);
    UNUSED(options);

    return NULL;
}

void closeSerialPort(serialPort_t *serialPort) {
    UNUSED(serialPort);
}

serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function) {
    UNUSED(function);

    return NULL;
}

bool sensors(uint32_t mask) {
    UNUSED(mask);
    return false;
}

bool telemetryDetermineEnabledState(portSharing_e) {
    return true;
}

portSharing_e determinePortSharing(serialPortConfig_t *, serialPortFunction_e) {
    return PORTSHARING_NOT_SHARED;
}

batteryState_e getBatteryState(void) {
	return BATTERY_OK;
}

}

