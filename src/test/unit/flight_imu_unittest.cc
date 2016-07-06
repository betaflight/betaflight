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
#include <math.h>

#include <limits.h>

#define BARO

extern "C" {
    #include <platform.h>
    #include "build/build_config.h"
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"
    #include "config/profile.h"

    #include "sensors/sensors.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"

    #include "sensors/gyro.h"
    #include "sensors/compass.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"

    #include "fc/runtime_config.h"

    #include "io/motor_and_servo.h"
    #include "fc/rc_controls.h"

    #include "rx/rx.h"

    #include "fc/rc_controls.h"

    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/imu.h"

    PG_REGISTER_PROFILE(pidProfile_t, pidProfile, PG_PID_PROFILE, 0);
    PG_REGISTER_PROFILE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER_PROFILE(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);
    PG_REGISTER_PROFILE(compassConfig_t, compassConfig, PG_COMPASS_CONFIGURATION, 0);
    PG_REGISTER(motorAndServoConfig_t, motorAndServoConfig, PG_MOTOR_AND_SERVO_CONFIG, 0);

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern float q0, q1, q2, q3;
extern "C" {
void imuComputeRotationMatrix(void);
void imuUpdateEulerAngles(void);

int16_t cycleTime = 2000;

}

void imuComputeQuaternionFromRPY(int16_t initialRoll, int16_t initialPitch, int16_t initialYaw)
{
    if (initialRoll > 1800) initialRoll -= 3600;
    if (initialPitch > 1800) initialPitch -= 3600;
    if (initialYaw > 1800) initialYaw -= 3600;

    float cosRoll = cosf(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);
    float sinRoll = sinf(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);

    float cosPitch = cosf(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);
    float sinPitch = sinf(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);

    float cosYaw = cosf(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);
    float sinYaw = sinf(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);

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

acc_t acc;
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
int16_t sonarMaxAltWithTiltCm;
int32_t accADC[XYZ_AXIS_COUNT];
int32_t gyroADC[XYZ_AXIS_COUNT];

int16_t GPS_speed;
int16_t GPS_ground_course;
int16_t GPS_numSat;

float magneticDeclination = 0.0f;

bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }

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
void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims)
{
    UNUSED(rollAndPitchTrims);
}

uint32_t micros(void) { return 0; }
uint32_t millis(void) { return 0; }
bool isBaroCalibrationComplete(void) { return true; }
void performBaroCalibrationCycle(void) {}
int32_t baroCalculateAltitude(void) { return 0; }
}
