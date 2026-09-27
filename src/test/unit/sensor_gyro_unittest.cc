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
#include <stddef.h>

#include <limits.h>
#include <algorithm>

#include "platform.h"

extern "C" {
    #include "build/build_config.h"
    #include "build/debug.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/utils.h"
    #include "common/vector.h"
    #include "drivers/accgyro/accgyro_virtual.h"
    #include "drivers/accgyro/accgyro_mpu.h"
    #include "drivers/sensor.h"
    #include "io/beeper.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "scheduler/scheduler.h"
    #include "sensors/gyro.h"
    #include "sensors/gyro_init.h"
    #include "sensors/acceleration.h"
    #include "sensors/sensors.h"

    STATIC_UNIT_TESTED gyroHardware_e gyroDetect(gyroDev_t *dev);
    struct gyroSensor_s;
    STATIC_UNIT_TESTED void performGyroCalibration(struct gyroSensor_s *gyroSensor, uint8_t gyroMovementCalibrationThreshold);
    STATIC_UNIT_TESTED bool virtualGyroRead(gyroDev_t *gyro);
    STATIC_UNIT_TESTED int gyroFindSensorIndexByHardware(const gyroSensor_t *sensors, int count, gyroHardware_e hardware);

    uint8_t debugMode;
    int16_t debug[DEBUG16_VALUE_COUNT];
}

#include "unittest_macros.h"
#include "gtest/gtest.h"
extern gyroSensor_s * const gyroSensorPtr;
extern gyroDev_t * const gyroDevPtr;

extern "C" const pgRegistry_t gyroDeviceConfig_Registry;

typedef struct legacyGyroDeviceConfig_s {
    int8_t index;
    uint8_t busType;
    uint8_t spiBus;
    ioTag_t csnTag;
    uint8_t i2cBus;
    uint8_t i2cAddress;
    ioTag_t extiTag;
    uint8_t alignment;
    sensorAlignment_t customAlignment;
    ioTag_t clkIn;
} legacyGyroDeviceConfig_t;

static_assert(sizeof(legacyGyroDeviceConfig_t) == sizeof(gyroDeviceConfig_t), "legacy gyro device PG stride changed");
static_assert(offsetof(legacyGyroDeviceConfig_t, clkIn) == offsetof(gyroDeviceConfig_t, clkIn), "legacy gyro device PG layout changed");


TEST(SensorGyro, Detect)
{
    const gyroHardware_e detected = gyroDetect(gyroDevPtr);
    EXPECT_EQ(GYRO_VIRTUAL, detected);
}

TEST(SensorGyro, FindsBmi088InSecondImuSlot)
{
    gyroSensor_t sensors[2] = {};
    sensors[0].gyroDev.gyroHardware = GYRO_ICM45686;
    sensors[1].gyroDev.gyroHardware = GYRO_BMI088;

    EXPECT_EQ(1, gyroFindSensorIndexByHardware(sensors, 2, GYRO_BMI088));
    EXPECT_EQ(-1, gyroFindSensorIndexByHardware(sensors, 2, GYRO_BMI270));
}

TEST(SensorGyro, LoadsVersionOneGyroDeviceArrayWithStableStride)
{
    legacyGyroDeviceConfig_t saved[MAX_GYRODEV_COUNT] = {};
    saved[0].index = 3;
    saved[0].csnTag = 11;
    saved[0].clkIn = 12;
#if MAX_GYRODEV_COUNT > 1
    saved[1].index = 4;
    saved[1].csnTag = 21;
    saved[1].clkIn = 22;
#endif

    ASSERT_EQ(1, pgVersion(&gyroDeviceConfig_Registry));
    ASSERT_TRUE(pgLoad(&gyroDeviceConfig_Registry, saved, sizeof(saved), 1));
    EXPECT_EQ(3, gyroDeviceConfig(0)->index);
    EXPECT_EQ(11, gyroDeviceConfig(0)->csnTag);
    EXPECT_EQ(12, gyroDeviceConfig(0)->clkIn);
    EXPECT_EQ(IO_TAG_NONE, gyroDeviceConfig(0)->accCsnTag);
#if MAX_GYRODEV_COUNT > 1
    EXPECT_EQ(4, gyroDeviceConfig(1)->index);
    EXPECT_EQ(21, gyroDeviceConfig(1)->csnTag);
    EXPECT_EQ(22, gyroDeviceConfig(1)->clkIn);
    EXPECT_EQ(IO_TAG_NONE, gyroDeviceConfig(1)->accCsnTag);
#endif
}

TEST(SensorGyro, Init)
{
    pgResetAll();
    const bool initialised = gyroInit();
    EXPECT_TRUE(initialised);
    EXPECT_EQ(GYRO_VIRTUAL, detectedSensors[SENSOR_INDEX_GYRO]);
}

TEST(SensorGyro, Read)
{
    pgResetAll();
    gyroInit();
    virtualGyroSet(gyroDevPtr, 5, 6, 7);
    const bool read = gyroDevPtr->readFn(gyroDevPtr);
    EXPECT_TRUE(read);
    EXPECT_EQ(5, gyroDevPtr->gyroADCRaw[X]);
    EXPECT_EQ(6, gyroDevPtr->gyroADCRaw[Y]);
    EXPECT_EQ(7, gyroDevPtr->gyroADCRaw[Z]);
}

TEST(SensorGyro, Calibrate)
{
    pgResetAll();
    gyroInit();
    gyroSetTargetLooptime(1);
    virtualGyroSet(gyroDevPtr, 5, 6, 7);
    const bool read = gyroDevPtr->readFn(gyroDevPtr);
    EXPECT_TRUE(read);
    EXPECT_EQ(5, gyroDevPtr->gyroADCRaw[X]);
    EXPECT_EQ(6, gyroDevPtr->gyroADCRaw[Y]);
    EXPECT_EQ(7, gyroDevPtr->gyroADCRaw[Z]);
    static const int gyroMovementCalibrationThreshold = 32;
    gyroDevPtr->gyroZero[X] = 8;
    gyroDevPtr->gyroZero[Y] = 9;
    gyroDevPtr->gyroZero[Z] = 10;
    performGyroCalibration(gyroSensorPtr, gyroMovementCalibrationThreshold);
    EXPECT_EQ(8, gyroDevPtr->gyroZero[X]);
    EXPECT_EQ(9, gyroDevPtr->gyroZero[Y]);
    EXPECT_EQ(10, gyroDevPtr->gyroZero[Z]);
    gyroStartCalibration(false);
    EXPECT_FALSE(gyroIsCalibrationComplete());
    while (!gyroIsCalibrationComplete()) {
        gyroDevPtr->readFn(gyroDevPtr);
        performGyroCalibration(gyroSensorPtr, gyroMovementCalibrationThreshold);
    }
    EXPECT_EQ(5, gyroDevPtr->gyroZero[X]);
    EXPECT_EQ(6, gyroDevPtr->gyroZero[Y]);
    EXPECT_EQ(7, gyroDevPtr->gyroZero[Z]);
}

TEST(SensorGyro, Update)
{
    pgResetAll();
    // turn off filters
    gyroConfigMutable()->gyro_lpf1_static_hz = 0;
    gyroConfigMutable()->gyro_lpf2_static_hz = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    gyroInit();
    gyroSetTargetLooptime(1);
    gyroDevPtr->readFn = virtualGyroRead;
    gyroStartCalibration(false);
    EXPECT_FALSE(gyroIsCalibrationComplete());

    virtualGyroSet(gyroDevPtr, 5, 6, 7);
    gyroUpdate();
    while (!gyroIsCalibrationComplete()) {
        virtualGyroSet(gyroDevPtr, 5, 6, 7);
        gyroUpdate();
    }
    EXPECT_TRUE(gyroIsCalibrationComplete());
    EXPECT_EQ(5, gyroDevPtr->gyroZero[X]);
    EXPECT_EQ(6, gyroDevPtr->gyroZero[Y]);
    EXPECT_EQ(7, gyroDevPtr->gyroZero[Z]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[X]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Z]);
    gyroUpdate();
    // expect zero values since gyro is calibrated
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[X]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Y]);
    EXPECT_FLOAT_EQ(0, gyro.gyroADCf[Z]);
    virtualGyroSet(gyroDevPtr, 15, 26, 97);
    gyroUpdate();
    EXPECT_NEAR(10 * gyroDevPtr->scale, gyro.gyroADC[X], 1e-3); // gyro.gyroADC values are scaled
    EXPECT_NEAR(20 * gyroDevPtr->scale, gyro.gyroADC[Y], 1e-3);
    EXPECT_NEAR(90 * gyroDevPtr->scale, gyro.gyroADC[Z], 1e-3);
}

// STUBS

extern "C" {

uint32_t micros(void) {return 0;}
void beeper(beeperMode_e) {}
uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE };
uint8_t detectedGyros[GYRO_COUNT] = { GYRO_NONE };
timeDelta_t getGyroUpdateRate(void) {return gyro.targetLooptime;}
void sensorsSet(uint32_t) {}
void schedulerResetTaskStatistics(taskId_e) {}
uint16_t getAverageSystemLoadPercent(void) {return 0;}
int getArmingDisableFlags(void) {return 0;}
void writeEEPROM(void) {}
}
