#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "pg/pg_ids.h"

#include "common/maths.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/position_estimator.h"

#include "io/gps.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
}

#include "gtest/gtest.h"

extern "C" {
uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];

acc_t acc;
matrix33_t rMat;
gpsSolutionData_t gpsSol;

static uint32_t enabledSensors = 0;
static bool rfHealthy = false;
static float rfAltCm = 0.0f;
static float baroAltCm = 0.0f;
static timeUs_t fakeMicros = 0;

bool sensors(uint32_t mask) { return (enabledSensors & mask) != 0; }
bool rangefinderIsHealthy(void) { return rfHealthy; }
int32_t rangefinderGetLatestAltitude(void) { return lrintf(rfAltCm); }
float getBaroAltitude(void) { return baroAltCm; }

timeUs_t micros(void) { return fakeMicros; }

bool gpsHasNewData(uint16_t *gpsStamp)
{
    (*gpsStamp)++;
    return true;
}

void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *dest)
{
    // Simplified (no Earth-scale projection): return integer-unit deltas so sign tests work.
    dest->x = (float)(to->lon - from->lon);
    dest->y = (float)(to->lat - from->lat);
}
}

static void stepEstimator(unsigned count = 1)
{
    for (unsigned i = 0; i < count; i++) {
        fakeMicros += 10000; // 100 Hz
        positionEstimatorUpdate();
    }
}

class PositionEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&acc, 0, sizeof(acc));
        memset(&rMat, 0, sizeof(rMat));
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(debug, 0, sizeof(debug));

        // Identity rotation and 1G reciprocal scale so zero accel stays zero.
        rMat.m[0][0] = 1.0f;
        rMat.m[1][1] = 1.0f;
        rMat.m[2][2] = 1.0f;
        acc.dev.acc_1G_rec = 1.0f;
        acc.accADC.z = 1.0f;

        enabledSensors = SENSOR_GPS | SENSOR_BARO | SENSOR_RANGEFINDER;
        rfHealthy = true;
        rfAltCm = 100.0f;
        baroAltCm = 100.0f;
        gpsSol.llh.altCm = 100.0f;
        gpsSol.dop.pdop = 100; // pDOP 1.0

        stateFlags = GPS_FIX;
        armingFlags = ARMED;
        fakeMicros = 0;

        positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_RANGEFINDER_PREFER;
        positionConfigMutable()->altitude_prefer_baro = 100;
        positionConfigMutable()->rangefinder_max_range_cm = 400;

        positionEstimatorInit();
    }
};

TEST_F(PositionEstimatorTest, RangefinderPreferFallsBackAndRecovers)
{
    // Establish offsets/baseline near zero.
    stepEstimator(10);
    const float baseline = positionEstimatorGetAltitudeCm();

    // Rangefinder unavailable: fallback should use baro/GPS updates.
    rfHealthy = false;
    baroAltCm = 150.0f;
    gpsSol.llh.altCm = 150.0f;
    stepEstimator(40);
    const float fallbackAltitude = positionEstimatorGetAltitudeCm();
    EXPECT_GT(fallbackAltitude, baseline + 10.0f);

    // Rangefinder becomes available again at a lower relative altitude.
    rfHealthy = true;
    rfAltCm = 120.0f; // +20cm relative to RF offset baseline
    stepEstimator(40);
    const float recoveredAltitude = positionEstimatorGetAltitudeCm();

    // In RANGEFINDER_PREFER, recovered altitude should be pulled down toward RF value.
    EXPECT_LT(recoveredAltitude, fallbackAltitude - 5.0f);
    EXPECT_TRUE(positionEstimatorIsValidZ());
}

// Regression test: GPS_distance2d must be called as (arm, current) so that a craft
// East of the arm position produces a positive kfX (East) position estimate.
// The bug had the arguments reversed — (current, arm) — giving a negative East estimate.
TEST_F(PositionEstimatorTest, GPSPositionEastOfArmIsPositive)
{
    // Run a couple of steps so the arm location is captured from the initial gpsSol (lon=0).
    stepEstimator(2);

    // Place the craft East of the arm (positive longitude delta).
    gpsSol.llh.lon = 100000;

    // Let the Kalman filter converge toward the GPS position measurement.
    stepEstimator(30);

    EXPECT_GT(positionEstimatorGetEstimate()->position.x, 0.0f);
}

// Regression test: forward thrust when heading East must produce a positive East velocity
// estimate.  rMat Y points West in Betaflight's earth frame, so accelEast must be
// -accEF_NEU.y (not +accEF_NEU.y).  The bug used the wrong sign, causing the IMU
// prediction to oppose GPS-measured East motion.
TEST_F(PositionEstimatorTest, EastThrustProducesPositiveEastVelocity)
{
    // Run a couple of steps so XY fusion is established with the arm at origin.
    stepEstimator(2);

    // Set rMat for heading East (attitude yaw = 90 deg).
    // Row pattern: row0 = (cosYaw, sinYaw, 0) = (0, 1, 0)
    //              row1 = (-sinYaw, cosYaw, 0) = (-1, 0, 0)
    memset(&rMat, 0, sizeof(rMat));
    rMat.m[0][1] = 1.0f;
    rMat.m[1][0] = -1.0f;
    rMat.m[2][2] = 1.0f;

    // Apply forward (East) thrust of 0.5 g alongside the steady-state gravity component.
    acc.accADC.x = 0.5f;
    acc.accADC.z = 1.0f;

    // GPS position and velocity remain zero (craft stationary in GPS frame) so any
    // non-zero velocity estimate is purely driven by the IMU acceleration prediction.
    // After many steps the sign of the steady-state velocity reflects the sign of accelEast.
    stepEstimator(100);

    EXPECT_GT(positionEstimatorGetEstimate()->velocity.x, 0.0f);
}

