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
#include <algorithm>

extern "C" {
    #include <platform.h>

    #include "build/build_config.h"
    #include "build/debug.h"
    #include "common/axis.h"
    #include "common/utils.h"
    #include "drivers/accgyro_fake.h"
    #include "drivers/logging_codes.h"
    #include "io/beeper.h"
    #include "scheduler/scheduler.h"
    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"
    #include "sensors/sensors.h"

    STATIC_UNIT_TESTED bool gyroDetect(gyroDev_t *dev, const extiConfig_t *extiConfig);
    STATIC_UNIT_TESTED void performGyroCalibration(uint8_t gyroMovementCalibrationThreshold);
    extern int32_t gyroZero[XYZ_AXIS_COUNT];
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(SensorGyro, Detect)
{
    const bool detected = gyroDetect(&gyro.dev, NULL);
    EXPECT_EQ(true, detected);
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
}

TEST(SensorGyro, Init)
{
    gyroInit();
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
}

TEST(SensorGyro, Read)
{
    gyroInit();
    fakeGyroSet(5, 6, 7);
    const bool read = gyro.dev.read(&gyro.dev);
    EXPECT_EQ(true, read);
    EXPECT_EQ(5, gyro.dev.gyroADCRaw[X]);
    EXPECT_EQ(6, gyro.dev.gyroADCRaw[Y]);
    EXPECT_EQ(7, gyro.dev.gyroADCRaw[Z]);
}

TEST(SensorGyro, Calibrate)
{
    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
    gyroInit();
    fakeGyroSet(5, 6, 7);
    const bool read = gyro.dev.read(&gyro.dev);
    EXPECT_EQ(true, read);
    gyro.gyroADC[X] = gyro.dev.gyroADCRaw[X];
    gyro.gyroADC[Y] = gyro.dev.gyroADCRaw[Y];
    gyro.gyroADC[Z] = gyro.dev.gyroADCRaw[Z];
    EXPECT_EQ(5, gyro.gyroADC[X]);
    EXPECT_EQ(6, gyro.gyroADC[Y]);
    EXPECT_EQ(7, gyro.gyroADC[Z]);
    static const int gyroMovementCalibrationThreshold = 32;
    gyroZero[X] = 8;
    gyroZero[Y] = 9;
    gyroZero[Z] = 10;
    performGyroCalibration(gyroMovementCalibrationThreshold);
    EXPECT_EQ(0, gyroZero[X]);
    EXPECT_EQ(0, gyroZero[Y]);
    EXPECT_EQ(0, gyroZero[Z]);
    EXPECT_EQ(false, gyroIsCalibrationComplete());
    while (!gyroIsCalibrationComplete()) {
        performGyroCalibration(gyroMovementCalibrationThreshold);
        gyro.gyroADC[X] = gyro.dev.gyroADCRaw[X];
        gyro.gyroADC[Y] = gyro.dev.gyroADCRaw[Y];
        gyro.gyroADC[Z] = gyro.dev.gyroADCRaw[Z];
    }
    EXPECT_EQ(5, gyroZero[X]);
    EXPECT_EQ(6, gyroZero[Y]);
    EXPECT_EQ(7, gyroZero[Z]);
}


// STUBS

extern "C" {

uint32_t micros(void) {return 0;}
void beeper(beeperMode_e) {}
uint8_t detectedSensors[] = { GYRO_NONE, ACC_NONE };
void addBootlogEvent6(bootLogEventCode_e eventCode, uint16_t eventFlags, uint16_t param1, uint16_t param2, uint16_t param3, uint16_t param4)
    {UNUSED(eventCode);UNUSED(eventFlags);UNUSED(param1);UNUSED(param2);UNUSED(param3);UNUSED(param4);}
timeDelta_t getGyroUpdateRate(void) {return gyro.targetLooptime;}
void sensorsSet(uint32_t) {}
void schedulerResetTaskStatistics(cfTaskId_e) {}
}
