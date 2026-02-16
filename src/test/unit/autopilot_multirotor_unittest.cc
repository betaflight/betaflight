/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
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
    #include "common/filter.h"
    #include "common/maths.h"
    #include "common/vector.h"

    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/autopilot_multirotor.h"
    #include "flight/imu.h"
    #include "flight/position.h"

    #include "io/gps.h"

    #include "pg/autopilot.h"
    #include "pg/rx.h"

    #include "sensors/acceleration.h"
    #include "rx/rx.h"
    #include "sensors/gyro.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);

    timeUs_t currentTimeUs = 0;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint32_t millisRW;
uint32_t millis() {
    return millisRW;
}

// Test autopilot initialization
TEST(AutopilotMultirotorUnittest, InitializationTest)
{
    autopilotInit();

    // After initialization, autopilot should be in control (sticks inactive by default)
    EXPECT_EQ(isAutopilotInControl(), true);

    // Throttle should be 0
    EXPECT_FLOAT_EQ(getAutopilotThrottle(), 0.0f);
}

// Test stick active status
TEST(AutopilotMultirotorUnittest, SticksActiveStatusTest)
{
    autopilotInit();

    // Initially, autopilot should be in control (sticks inactive)
    EXPECT_EQ(isAutopilotInControl(), true);

    // Set sticks active - autopilot should NOT be in control
    setSticksActiveStatus(true);
    EXPECT_EQ(isAutopilotInControl(), false);

    // Set sticks inactive - autopilot should be in control
    setSticksActiveStatus(false);
    EXPECT_EQ(isAutopilotInControl(), true);
}

// Test position control reset
TEST(AutopilotMultirotorUnittest, ResetPositionControlTest)
{
    autopilotInit();

    gpsLocation_t targetLocation;
    targetLocation.lat = 404635100;  // 40.4635100 degrees
    targetLocation.lon = -795181700; // -79.5181700 degrees
    targetLocation.altCm = 50000;    // 500m altitude

    // Reset position control with a target location
    resetPositionControl(&targetLocation, 100); // 100Hz task rate

    // After reset, autopilot should still be in control (if sticks are inactive)
    EXPECT_EQ(isAutopilotInControl(), true);
}

// Test altitude control
TEST(AutopilotMultirotorUnittest, AltitudeControlBasicTest)
{
    autopilotInit();

    float targetAltitudeCm = 10000.0f; // 100m
    float taskIntervalS = 0.01f;        // 100Hz
    float targetAltitudeStep = 0.0f;    // No step change

    // Run altitude control
    altitudeControl(targetAltitudeCm, taskIntervalS, targetAltitudeStep);

    // Should produce some throttle output
    float throttle = getAutopilotThrottle();
    EXPECT_GE(throttle, -1.0f);
    EXPECT_LE(throttle, 1.0f);
}

// Test below landing altitude check
TEST(AutopilotMultirotorUnittest, BelowLandingAltitudeTest)
{
    autopilotInit();

    // At initialization, should not be below landing altitude
    bool belowLanding = isBelowLandingAltitude();
    EXPECT_EQ(belowLanding, false);
}

// STUBS

extern "C" {
    uint8_t armingFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    acc_t acc;
    attitudeEulerAngles_t attitude = { .values = { 0, 0, 0 } };
    gpsSolutionData_t gpsSol = {};

    float getAltitudeCm(void) { return 10000.0f; } // 100m altitude
    float getAltitudeDerivative(void) { return 0.0f; }
    float getCosTiltAngle(void) { return 1.0f; }
    float getGpsDataIntervalSeconds(void) { return 0.1f; }
    float getGpsDataFrequencyHz(void) { return 10.0f; }
    float rcCommand[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    bool gpsHasNewData(uint16_t* gpsStamp) {
        UNUSED(gpsStamp);
        return false;
    }

    void GPS_distance2d(const gpsLocation_t* from, const gpsLocation_t* to, vector2_t* dest) {
        UNUSED(from);
        UNUSED(to);
        if (dest) {
            dest->x = 0.0f;
            dest->y = 0.0f;
        }
    }

    void navOriginUpdate(const gpsLocation_t* currentPos) {
        UNUSED(currentPos);
    }

    void navOriginLLHtoNED(const gpsLocation_t* llh, vector3_t* ned) {
        UNUSED(llh);
        if (ned) {
            ned->x = 0.0f;
            ned->y = 0.0f;
            ned->z = 0.0f;
        }
    }

    bool navOriginIsValid(void) {
        return true;
    }

    void parseRcChannels(const char *input, rxConfig_t *rxConfig) {
        UNUSED(input);
        UNUSED(rxConfig);
    }

    throttleStatus_e calculateThrottleStatus(void) {
        return THROTTLE_LOW;
    }

    // Waypoint stubs
    bool isWaypointMissionValid(void) { return false; }
    uint8_t getGeoZoneForLocation(const gpsLocation_t *llh) {
        UNUSED(llh);
        return 0;
    }
    bool hasReachedWaypoint(void) { return false; }
    void updateWaypoint(void) { }
    void waypointInitialize(void) { }
    const gpsLocation_t* wpGetWaypointLocation(void) {
        static gpsLocation_t loc = { .lat = 0, .lon = 0, .altCm = 0 };
        return &loc;
    }

    // IMU stubs
    float pidGetPreviousSetpoint(int axis) {
        UNUSED(axis);
        return 0.0f;
    }

    void pidSetAntiGravityState(bool enabled) {
        UNUSED(enabled);
    }
}
