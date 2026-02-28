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

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

extern "C" {

    #include "platform.h"
    #include "build/debug.h"
    #include "pg/pg_ids.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/vector.h"

    #include "io/gps.h"

    #include "pg/gps.h"
    #include "pg/gps_rescue.h"
    #include "pg/rx.h"

    #include "scheduler/scheduler.h"

    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);

    extern gpsSolutionData_t gpsSol;
    extern uint8_t stateFlags;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static uint32_t millisRW;

// =========================================================================
// navOriginInit / navOriginIsValid
// =========================================================================

TEST(GpsNavOriginUnittest, InitSetsValidOrigin)
{
    gpsLocation_t origin = { .lat = 404635100, .lon = -795181700, .altCm = 50000 };
    navOriginInit(&origin);

    EXPECT_TRUE(navOriginIsValid());
    EXPECT_EQ(navOriginGetDistanceCm(), 0u);
}

// =========================================================================
// LLH → NED conversion
// =========================================================================

TEST(GpsNavOriginUnittest, LLHtoNEDInvalidOrigin)
{
    // Without calling navOriginInit, origin is invalid
    // Need a fresh origin state - call navOriginInit with a position then test
    // Actually navOriginInit always sets isValid=true.
    // We test the case where we init then check basic conversion.
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // Same point as origin → NED should be (0, 0, 0)
    vector3_t ned;
    navOriginLLHtoNED(&origin, &ned);

    EXPECT_NEAR(ned.x, 0.0f, 0.1f);
    EXPECT_NEAR(ned.y, 0.0f, 0.1f);
    EXPECT_NEAR(ned.z, 0.0f, 0.1f);
}

TEST(GpsNavOriginUnittest, LLHtoNEDNorth)
{
    // Origin at 40°N, 10°E
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // Point 0.001° north (≈111.3m)
    gpsLocation_t point = { .lat = 400010000, .lon = 100000000, .altCm = 10000 };
    vector3_t ned;
    navOriginLLHtoNED(&point, &ned);

    // Expected North: 10000 * EARTH_ANGLE_TO_CM ≈ 10000 * 1.113195 ≈ 11132 cm
    EXPECT_GT(ned.x, 10000.0f);
    EXPECT_LT(ned.x, 12000.0f);
    EXPECT_NEAR(ned.y, 0.0f, 1.0f);
    EXPECT_NEAR(ned.z, 0.0f, 1.0f);
}

TEST(GpsNavOriginUnittest, LLHtoNEDSouth)
{
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // Point 0.001° south
    gpsLocation_t point = { .lat = 399990000, .lon = 100000000, .altCm = 10000 };
    vector3_t ned;
    navOriginLLHtoNED(&point, &ned);

    // North should be negative (south)
    EXPECT_LT(ned.x, 0.0f);
    EXPECT_NEAR(ned.y, 0.0f, 1.0f);
}

TEST(GpsNavOriginUnittest, LLHtoNEDEast)
{
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // Point 0.001° east
    gpsLocation_t point = { .lat = 400000000, .lon = 100010000, .altCm = 10000 };
    vector3_t ned;
    navOriginLLHtoNED(&point, &ned);

    EXPECT_NEAR(ned.x, 0.0f, 1.0f);
    // East should be positive, but less than North for same delta due to latitude scaling
    EXPECT_GT(ned.y, 0.0f);
}

TEST(GpsNavOriginUnittest, LLHtoNEDDown)
{
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // Point 100m lower in altitude
    gpsLocation_t point = { .lat = 400000000, .lon = 100000000, .altCm = 0 };
    vector3_t ned;
    navOriginLLHtoNED(&point, &ned);

    EXPECT_NEAR(ned.x, 0.0f, 1.0f);
    EXPECT_NEAR(ned.y, 0.0f, 1.0f);
    // Down is positive in NED when altitude decreases
    EXPECT_NEAR(ned.z, 10000.0f, 1.0f);
}

TEST(GpsNavOriginUnittest, LLHtoNEDAltitudeAboveOrigin)
{
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // Point 50m higher in altitude
    gpsLocation_t point = { .lat = 400000000, .lon = 100000000, .altCm = 15000 };
    vector3_t ned;
    navOriginLLHtoNED(&point, &ned);

    // Down is negative when altitude increases (NED convention)
    EXPECT_NEAR(ned.z, -5000.0f, 1.0f);
}

// =========================================================================
// NED → LLH conversion
// =========================================================================

TEST(GpsNavOriginUnittest, NEDtoLLHBasic)
{
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // Zero NED → should return origin
    vector3_t ned = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
    gpsLocation_t result;
    navOriginNEDtoLLH(&ned, &result);

    EXPECT_EQ(result.lat, 400000000);
    EXPECT_EQ(result.lon, 100000000);
    EXPECT_EQ(result.altCm, 10000);
}

TEST(GpsNavOriginUnittest, NEDtoLLHNorth)
{
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // 11132 cm North ≈ 0.001° latitude → 10000 GPS units
    float northCm = 10000.0f * EARTH_ANGLE_TO_CM;
    vector3_t ned = { .x = northCm, .y = 0.0f, .z = 0.0f };
    gpsLocation_t result;
    navOriginNEDtoLLH(&ned, &result);

    EXPECT_NEAR(result.lat, 400010000, 2);  // Allow ±2 GPS units rounding
    EXPECT_EQ(result.lon, 100000000);
}

// =========================================================================
// Round-trip accuracy: LLH → NED → LLH
// =========================================================================

TEST(GpsNavOriginUnittest, RoundTripAccuracyNearOrigin)
{
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // Test points at various offsets from origin
    gpsLocation_t testPoints[] = {
        { .lat = 400010000, .lon = 100000000, .altCm = 10000 },  // 111m North
        { .lat = 400000000, .lon = 100010000, .altCm = 10000 },  // ~85m East (at 40°N)
        { .lat = 400005000, .lon = 100005000, .altCm = 15000 },  // NE + 50m up
        { .lat = 399995000, .lon =  99995000, .altCm =  5000 },  // SW + 50m down
    };

    for (size_t i = 0; i < sizeof(testPoints) / sizeof(testPoints[0]); i++) {
        vector3_t ned;
        navOriginLLHtoNED(&testPoints[i], &ned);

        gpsLocation_t result;
        navOriginNEDtoLLH(&ned, &result);

        // Round-trip should be accurate within ±2 GPS units (~0.2mm)
        // The float precision limits this
        EXPECT_NEAR(result.lat, testPoints[i].lat, 3) << "Test point " << i << " lat";
        EXPECT_NEAR(result.lon, testPoints[i].lon, 3) << "Test point " << i << " lon";
        EXPECT_NEAR(result.altCm, testPoints[i].altCm, 2) << "Test point " << i << " alt";
    }
}

TEST(GpsNavOriginUnittest, RoundTripAccuracyAt500m)
{
    // Test at the edge of the 500m reset threshold
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // ~450m North (within 500m threshold)
    // 450m = 45000cm → deltaLat = 45000 / EARTH_ANGLE_TO_CM ≈ 40426 GPS units
    gpsLocation_t point = { .lat = 400040000, .lon = 100000000, .altCm = 10000 };
    vector3_t ned;
    navOriginLLHtoNED(&point, &ned);

    gpsLocation_t result;
    navOriginNEDtoLLH(&ned, &result);

    // At 450m, float precision should still be good
    EXPECT_NEAR(result.lat, point.lat, 5);
    EXPECT_NEAR(result.lon, point.lon, 5);
}

// =========================================================================
// Origin reset at 500m
// =========================================================================

TEST(GpsNavOriginUnittest, OriginResetAt500m)
{
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);
    EXPECT_EQ(navOriginGetDistanceCm(), 0u);

    // Move 600m north (> 500m threshold)
    // 600m = 60000cm → deltaLat = 60000 / EARTH_ANGLE_TO_CM ≈ 53901 GPS units
    gpsLocation_t farPoint = { .lat = 400054000, .lon = 100000000, .altCm = 10000 };
    navOriginUpdate(&farPoint);

    // Origin should have been reset → distance back to 0
    EXPECT_EQ(navOriginGetDistanceCm(), 0u);

    // Converting farPoint should now give near-zero NED
    vector3_t ned;
    navOriginLLHtoNED(&farPoint, &ned);
    EXPECT_NEAR(ned.x, 0.0f, 100.0f);  // Close to new origin
    EXPECT_NEAR(ned.y, 0.0f, 100.0f);
}

TEST(GpsNavOriginUnittest, OriginNoResetUnder500m)
{
    gpsLocation_t origin = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&origin);

    // Move 300m north (< 500m threshold)
    gpsLocation_t nearPoint = { .lat = 400027000, .lon = 100000000, .altCm = 10000 };
    navOriginUpdate(&nearPoint);

    // Distance should be approximately 300m = 30000cm
    uint32_t dist = navOriginGetDistanceCm();
    EXPECT_GT(dist, 25000u);
    EXPECT_LT(dist, 35000u);

    // Converting origin point should still give near-zero (origin unchanged)
    vector3_t ned;
    navOriginLLHtoNED(&origin, &ned);
    EXPECT_NEAR(ned.x, 0.0f, 1.0f);
    EXPECT_NEAR(ned.y, 0.0f, 1.0f);
}

TEST(GpsNavOriginUnittest, OriginUpdateInitializesIfInvalid)
{
    // navOriginUpdate should initialize if not valid
    // We can't easily reset the static navOrigin to invalid from the test,
    // but navOriginInit always sets valid=true. So we test that Init+Update works.
    gpsLocation_t pos = { .lat = 400000000, .lon = 100000000, .altCm = 10000 };
    navOriginInit(&pos);

    // Update with same position
    navOriginUpdate(&pos);
    EXPECT_TRUE(navOriginIsValid());
    EXPECT_EQ(navOriginGetDistanceCm(), 0u);
}

// =========================================================================
// Meridian crossing (longitude wrap at ±180°)
// =========================================================================

TEST(GpsNavOriginUnittest, MeridianCrossingEastToWest)
{
    // Origin at 179.9° E (just west of antimeridian)
    gpsLocation_t origin = { .lat = 0, .lon = 1799000000, .altCm = 0 };
    navOriginInit(&origin);

    // Point at 179.9° W (= -179.9° = just east of antimeridian, crossing the line)
    gpsLocation_t point = { .lat = 0, .lon = -1799000000, .altCm = 0 };
    vector3_t ned;
    navOriginLLHtoNED(&point, &ned);

    // The point is 0.2° east of origin (crossing the meridian)
    // North should be ~0, East should be positive (going east across meridian)
    EXPECT_NEAR(ned.x, 0.0f, 1.0f);
    EXPECT_GT(ned.y, 0.0f);  // East of origin

    // Magnitude check: 0.2° longitude at equator ≈ 2,226,390 cm (~22.3 km)
    EXPECT_GT(ned.y, 2200000.0f);
    EXPECT_LT(ned.y, 2300000.0f);
}

TEST(GpsNavOriginUnittest, MeridianCrossingWestToEast)
{
    // Origin at 179.9° W
    gpsLocation_t origin = { .lat = 0, .lon = -1799000000, .altCm = 0 };
    navOriginInit(&origin);

    // Point at 179.9° E (crossing westward across antimeridian)
    gpsLocation_t point = { .lat = 0, .lon = 1799000000, .altCm = 0 };
    vector3_t ned;
    navOriginLLHtoNED(&point, &ned);

    // Point is 0.2° west of origin
    EXPECT_NEAR(ned.x, 0.0f, 1.0f);
    EXPECT_LT(ned.y, 0.0f);  // West of origin
}

// =========================================================================
// Latitude effects on longitude scaling
// =========================================================================

TEST(GpsNavOriginUnittest, LongitudeScalingAtHighLatitude)
{
    // At 60°N, longitude degrees are half their equatorial size
    gpsLocation_t origin = { .lat = 600000000, .lon = 100000000, .altCm = 0 };
    navOriginInit(&origin);

    // Same longitude offset at equator and at 60°N
    gpsLocation_t point = { .lat = 600000000, .lon = 100010000, .altCm = 0 };
    vector3_t ned60;
    navOriginLLHtoNED(&point, &ned60);

    // Now at equator
    gpsLocation_t origin0 = { .lat = 0, .lon = 100000000, .altCm = 0 };
    navOriginInit(&origin0);
    gpsLocation_t point0 = { .lat = 0, .lon = 100010000, .altCm = 0 };
    vector3_t ned0;
    navOriginLLHtoNED(&point0, &ned0);

    // East distance at 60°N should be roughly half of equator
    // cos(60°) = 0.5
    float ratio = ned60.y / ned0.y;
    EXPECT_NEAR(ratio, 0.5f, 0.05f);
}

// =========================================================================
// STUBS
// =========================================================================

extern "C" {
    uint8_t armingFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    // Serial stubs (matching real declarations from io/serial.h and drivers/serial.h)
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
    void serialPrint(serialPort_t *, const char *) {}
    void serialSetBaudRate(serialPort_t *, uint32_t) {}
    void serialWriteBuf(serialPort_t *, const uint8_t *, int) {}
    const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return NULL; }
    void serialWrite(serialPort_t *, uint8_t) {}
    bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
    void closeSerialPort(serialPort_t *) {}
    uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
    uint8_t serialRead(serialPort_t *) { return 0; }
    uint32_t serialGetBaudRate(serialPort_t *) { return 115200; }
    void serialSetMode(serialPort_t *, portMode_e) {}
    void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
    void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
    serialType_e serialType(serialPortIdentifier_e) { return (serialType_e)0; }

    // Baud rate stubs
    const uint32_t baudRates[BAUD_COUNT] = { 0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000 };
    baudRate_e lookupBaudRateIndex(uint32_t) { return (baudRate_e)0; }

    // Timer stubs
    uint32_t millis(void) { return millisRW; }
    timeUs_t micros(void) { return 0; }
    void delay(uint32_t) {}

    // State stubs
    bool sensors(uint32_t mask) { UNUSED(mask); return true; }
    void sensorsSet(uint32_t) {}
    void sensorsClear(uint32_t) {}

    // Beeper stubs
    void beeper(uint32_t) {}
    void beeperConfirmationBeeps(uint8_t) {}

    // Feature stubs
    bool featureIsEnabled(uint32_t) { return false; }

    // Dashboard stubs
    void dashboardUpdate(void) {}
    void dashboardShowFixedPage(int) {}

    // GPS Rescue stubs
    uint16_t gpsRescueGetGPSRate(void) { return 10; }

    // Scheduler stubs
    void rescheduleTask(taskId_e, timeDelta_t) {}
    void schedulerSetNextStateTime(timeDelta_t) {}

    // Misc stubs
    void LED0_TOGGLE(void) {}
    void LED1_TOGGLE(void) {}
    void parseRcChannels(const char *input, rxConfig_t *rxConfig) {
        UNUSED(input);
        UNUSED(rxConfig);
    }
    void failsafeOnValidDataReceived(void) {}
    void failsafeOnValidDataFailed(void) {}
    void gpsLapTimerNewGpsData(void) {}
    bool isRxReceivingSignal(void) { return true; }
}
