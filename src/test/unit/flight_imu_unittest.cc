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

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

#define BARO

extern "C" {
    #include "debug.h"

    #include "common/axis.h"
    #include "common/maths.h"

    #include "sensors/sensors.h"
    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"
    #include "sensors/gyro.h"
    #include "sensors/compass.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"

    #include "config/runtime_config.h"

    #include "rx/rx.h"

    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/imu.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern float q0, q1, q2, q3;
extern "C" { 
void imuComputeRotationMatrix(void);
void imuUpdateEulerAngles(void);
}

void imuComputeQuaternionFromRPY(int16_t initialRoll, int16_t initialPitch, int16_t initialYaw)
{
    if (initialRoll > 1800) initialRoll -= 3600;
    if (initialPitch > 1800) initialPitch -= 3600;
    if (initialYaw > 1800) initialYaw -= 3600;

    float cosRoll = cos_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);
    float sinRoll = sin_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);

    float cosPitch = cos_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);
    float sinPitch = sin_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);

    float cosYaw = cos_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);
    float sinYaw = sin_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);

    q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

    imuComputeRotationMatrix();
}

TEST(FlightImuTest, TestEulerAngleCalculation)
{
    imuComputeQuaternionFromRPY(0, 0, 0);
    imuUpdateEulerAngles();
    EXPECT_FLOAT_EQ(attitude.values.roll, 0);
    EXPECT_FLOAT_EQ(attitude.values.pitch, 0);
    EXPECT_FLOAT_EQ(attitude.values.yaw, 0);

    imuComputeQuaternionFromRPY(450, 450, 0);
    imuUpdateEulerAngles();
    EXPECT_FLOAT_EQ(attitude.values.roll, 450);
    EXPECT_FLOAT_EQ(attitude.values.pitch, 450);
    EXPECT_FLOAT_EQ(attitude.values.yaw, 0);

    imuComputeQuaternionFromRPY(-450, -450, 0);
    imuUpdateEulerAngles();
    EXPECT_FLOAT_EQ(attitude.values.roll, -450);
    EXPECT_FLOAT_EQ(attitude.values.pitch, -450);
    EXPECT_FLOAT_EQ(attitude.values.yaw, 0);

    imuComputeQuaternionFromRPY(1790, 0, 0);
    imuUpdateEulerAngles();
    EXPECT_FLOAT_EQ(attitude.values.roll, 1790);
    EXPECT_FLOAT_EQ(attitude.values.pitch, 0);
    EXPECT_FLOAT_EQ(attitude.values.yaw, 0);

    imuComputeQuaternionFromRPY(-1790, 0, 0);
    imuUpdateEulerAngles();
    EXPECT_FLOAT_EQ(attitude.values.roll, -1790);
    EXPECT_FLOAT_EQ(attitude.values.pitch, 0);
    EXPECT_FLOAT_EQ(attitude.values.yaw, 0);

    imuComputeQuaternionFromRPY(0, 0, 900);
    imuUpdateEulerAngles();
    EXPECT_FLOAT_EQ(attitude.values.roll, 0);
    EXPECT_FLOAT_EQ(attitude.values.pitch, 0);
    EXPECT_FLOAT_EQ(attitude.values.yaw, 900);

    imuComputeQuaternionFromRPY(0, 0, 2700);
    imuUpdateEulerAngles();
    EXPECT_FLOAT_EQ(attitude.values.roll, 0);
    EXPECT_FLOAT_EQ(attitude.values.pitch, 0);
    EXPECT_FLOAT_EQ(attitude.values.yaw, 2700);
}

// STUBS

extern "C" {
uint32_t rcModeActivationMask;
int16_t rcCommand[4];
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

uint16_t acc_1G;
int16_t heading;
gyro_t gyro;
int32_t magADC[XYZ_AXIS_COUNT];
int32_t BaroAlt;
int16_t debug[DEBUG16_VALUE_COUNT];

uint8_t stateFlags;
uint16_t flightModeFlags;
uint8_t armingFlags;

int32_t sonarAlt;
int16_t sonarCfAltCm;
int32_t accADC[XYZ_AXIS_COUNT];
int32_t gyroADC[XYZ_AXIS_COUNT];

int16_t GPS_speed;
int16_t GPS_ground_course;
int16_t GPS_numSat;
int16_t cycleTime = 2000;


uint16_t enableFlightMode(flightModeFlags_e mask)
{
    return flightModeFlags |= (mask);
}

uint16_t disableFlightMode(flightModeFlags_e mask)
{
    return flightModeFlags &= ~(mask);
}

void gyroUpdate(void) {};
bool sensors(uint32_t mask)
{
    UNUSED(mask);
    return false;
};
void updateAccelerationReadings(void)
{
}
bool isGyroCalibrationComplete(void) { return 1; }
bool isCompassReady(void) { return 1; }
uint32_t micros(void) { return 0; }
uint32_t millis(void) { return 0; }
bool isBaroCalibrationComplete(void) { return true; }
void performBaroCalibrationCycle(void) {}
int32_t baroCalculateAltitude(void) { return 0; }

}