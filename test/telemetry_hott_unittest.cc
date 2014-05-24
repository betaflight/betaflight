#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <limits.h>

#include "platform.h"

#include "common/axis.h"

#include "drivers/system_common.h"

#include "drivers/serial_common.h"
#include "serial_common.h"

#include "runtime_config.h"

#include "sensors_common.h"

#include "flight_common.h"
#include "gps_common.h"
#include "battery.h"

#include "telemetry_common.h"
#include "telemetry_hott.h"

#include "gps_conversion.h"


#include "unittest_macros.h"
#include "gtest/gtest.h"

void addGPSCoordinates(HOTT_GPS_MSG_t *hottGPSMessage, int32_t latitude, int32_t longitude);

// See http://en.wikipedia.org/wiki/Geographic_coordinate_conversion

HOTT_GPS_MSG_t hottGPSMessage;

HOTT_GPS_MSG_t *getGPSMessageForTest(void)
{
    memset(&hottGPSMessage, 0, sizeof(hottGPSMessage));
    return &hottGPSMessage;
}

TEST(TelemetryHottTest, UpdateGPSCoordinates)
{
    // given
    HOTT_GPS_MSG_t *hottGPSMessage = getGPSMessageForTest();

    // Mayrhofen, Austria
    uint32_t latitude = GPS_coord_to_degrees("4710.5186");
    uint32_t longitude = GPS_coord_to_degrees("1151.4252");


    // when
    addGPSCoordinates(hottGPSMessage, longitude, latitude);

    // then
    EXPECT_EQ(hottGPSMessage->pos_EW, 0);
    EXPECT_EQ(hottGPSMessage->pos_EW_dm_H << 8 | hottGPSMessage->pos_EW_dm_L, 4710);
    EXPECT_EQ(hottGPSMessage->pos_EW_sec_H << 8 | hottGPSMessage->pos_EW_sec_L, 5186);

    EXPECT_EQ(hottGPSMessage->pos_NS, 0);
    EXPECT_EQ(hottGPSMessage->pos_NS_dm_H << 8 | hottGPSMessage->pos_NS_dm_L, 1151);
    EXPECT_EQ(hottGPSMessage->pos_NS_sec_H << 8 | hottGPSMessage->pos_NS_sec_L, 4251);
}


// STUBS

int16_t debug[4];

uint8_t GPS_numSat;
flags_t f;
int32_t GPS_coord[2];
uint16_t GPS_speed;                 // speed in 0.1m/s
uint16_t GPS_distanceToHome;        // distance to home point in meters
uint16_t GPS_altitude;              // altitude in 0.1m
uint8_t vbat;
int16_t GPS_directionToHome;        // direction to home or hol point in degrees

uint32_t micros(void) { return 0; }

uint8_t serialTotalBytesWaiting(serialPort_t *instance) {
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

void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate) {
    UNUSED(instance);
    UNUSED(baudRate);
}

void beginSerialPortFunction(serialPort_t *port, serialPortFunction_e function) {
    UNUSED(port);
    UNUSED(function);
}

void endSerialPortFunction(serialPort_t *port, serialPortFunction_e function) {
    UNUSED(port);
    UNUSED(function);
}

serialPort_t *openSerialPort(serialPortFunction_e functionMask, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, serialInversion_e inversion) {
    UNUSED(functionMask);
    UNUSED(baudRate);
    UNUSED(callback);
    UNUSED(mode);
    UNUSED(inversion);
}

serialPort_t *findOpenSerialPort(uint16_t functionMask) {
    UNUSED(functionMask);
    return NULL;
}

bool sensors(uint32_t mask) {
    UNUSED(mask);
    return false;
}






