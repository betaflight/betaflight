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
    #include "common/maths.h"
    #include "common/utils.h"
    #include "drivers/accgyro/accgyro_fake.h"
    #include "drivers/logging_codes.h"
    #include "io/beeper.h"
    #include "scheduler/scheduler.h"
    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"
    #include "sensors/sensors.h"

    typedef struct gyroCalibration_s {
        int32_t g[XYZ_AXIS_COUNT];
        stdev_t var[XYZ_AXIS_COUNT];
        uint16_t calibratingG;
    } gyroCalibration_t;
    extern gyroCalibration_t gyroCalibration;
    extern gyroDev_t gyroDev0;

    STATIC_UNIT_TESTED gyroSensor_e gyroDetect(gyroDev_t *dev, gyroSensor_e gyroHardware);
    STATIC_UNIT_TESTED void performGyroCalibration(gyroDev_t *dev, gyroCalibration_t *gyroCalibration, uint8_t gyroMovementCalibrationThreshold);
    STATIC_UNIT_TESTED bool isCalibrationComplete(gyroCalibration_t *gyroCalibration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(SensorGyro, Detect)
{
    const gyroSensor_e detected = gyroDetect(&gyroDev0, GYRO_AUTODETECT);
    EXPECT_EQ(GYRO_FAKE, detected);
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
}

TEST(SensorGyro, Init)
{
    const bool initialised = gyroInit();
    EXPECT_EQ(true, initialised);
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
}

TEST(SensorGyro, Read)
{
    gyroInit();
    EXPECT_EQ(GYRO_FAKE, detectedSensors[SENSOR_INDEX_GYRO]);
    fakeGyroSet(5, 6, 7);
    const bool read = gyroDev0.read(&gyroDev0);
    EXPECT_EQ(true, read);
    EXPECT_EQ(5, gyroDev0.gyroADCRaw[X]);
    EXPECT_EQ(6, gyroDev0.gyroADCRaw[Y]);
    EXPECT_EQ(7, gyroDev0.gyroADCRaw[Z]);
}

TEST(SensorGyro, Calibrate)
{
    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
    gyroInit();
    fakeGyroSet(5, 6, 7);
    const bool read = gyroDev0.read(&gyroDev0);
    EXPECT_EQ(true, read);
    EXPECT_EQ(5, gyroDev0.gyroADCRaw[X]);
    EXPECT_EQ(6, gyroDev0.gyroADCRaw[Y]);
    EXPECT_EQ(7, gyroDev0.gyroADCRaw[Z]);
    static const int gyroMovementCalibrationThreshold = 32;
    gyroDev0.gyroZero[X] = 8;
    gyroDev0.gyroZero[Y] = 9;
    gyroDev0.gyroZero[Z] = 10;
    performGyroCalibration(&gyroDev0, &gyroCalibration, gyroMovementCalibrationThreshold);
    EXPECT_EQ(0, gyroDev0.gyroZero[X]);
    EXPECT_EQ(0, gyroDev0.gyroZero[Y]);
    EXPECT_EQ(0, gyroDev0.gyroZero[Z]);
    EXPECT_EQ(false, isCalibrationComplete(&gyroCalibration));
    while (!isCalibrationComplete(&gyroCalibration)) {
        performGyroCalibration(&gyroDev0, &gyroCalibration, gyroMovementCalibrationThreshold);
    }
    EXPECT_EQ(5, gyroDev0.gyroZero[X]);
    EXPECT_EQ(6, gyroDev0.gyroZero[Y]);
    EXPECT_EQ(7, gyroDev0.gyroZero[Z]);
}

TEST(SensorGyro, Update)
{
    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
    EXPECT_EQ(false, isCalibrationComplete(&gyroCalibration));
    gyroInit();
    fakeGyroSet(5, 6, 7);
    gyroUpdate();
    EXPECT_EQ(false, isCalibrationComplete(&gyroCalibration));
    while (!isCalibrationComplete(&gyroCalibration)) {
        gyroUpdate();
    }
    EXPECT_EQ(true, isCalibrationComplete(&gyroCalibration));
    EXPECT_EQ(5, gyroDev0.gyroZero[X]);
    EXPECT_EQ(6, gyroDev0.gyroZero[Y]);
    EXPECT_EQ(7, gyroDev0.gyroZero[Z]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[X]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Z]);
    gyroUpdate();
    // expect zero values since gyro is calibrated
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[X]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Z]);
    fakeGyroSet(15, 26, 97);
    gyroUpdate();
    EXPECT_FLOAT_EQ(10 * gyroDev0.scale, gyro.gyroADCf[X]); // gyroADCf values are scaled
    EXPECT_FLOAT_EQ(20 * gyroDev0.scale, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(90 * gyroDev0.scale, gyro.gyroADCf[Z]);
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
