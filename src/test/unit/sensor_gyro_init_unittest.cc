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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstring>

#include "platform.h"

extern "C" {
#include "build/debug.h"
#include "drivers/accgyro/accgyro_virtual.h"
#include "io/beeper.h"
#include "pg/pg.h"
#include "scheduler/scheduler.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/sensors.h"
}

#include "gtest/gtest.h"

static uint16_t initializedRates[GYRO_COUNT];
static float initializedScales[GYRO_COUNT];
static gyroHardware_e initializedHardware[GYRO_COUNT];
static unsigned initializationCount;
static unsigned eepromWriteCount;

class SensorGyroInit : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&gyro, 0, sizeof(gyro));
        pgResetAll();
        gyroConfigMutable()->gyro_enabled_bitmask = GYRO_MASK(0) | GYRO_MASK(1);
        gyroConfigMutable()->gyrosDetected = GYRO_MASK(0) | GYRO_MASK(1);
        initializationCount = 0;
        eepromWriteCount = 0;
        for (int i = 0; i < GYRO_COUNT; i++) {
            initializedRates[i] = 8000;
            initializedScales[i] = 0.07f;
            initializedHardware[i] = GYRO_LSM6DSV16X;
        }
    }
};

TEST_F(SensorGyroInit, CombinesMatchingInitializedSensors)
{
    EXPECT_TRUE(gyroInit());
    EXPECT_EQ(2u, initializationCount);
    EXPECT_EQ(GYRO_MASK(0) | GYRO_MASK(1), gyro.gyroEnabledBitmask);
    EXPECT_EQ(gyro.gyroEnabledBitmask, gyroConfig()->gyro_enabled_bitmask);
    EXPECT_EQ(0u, eepromWriteCount);
    EXPECT_EQ(&gyro.gyroSensor[0].gyroDev, gyroActiveDev());
    EXPECT_EQ(8000, gyro.sampleRateHz);
    EXPECT_FLOAT_EQ(0.07f, gyro.scale);
}

TEST_F(SensorGyroInit, RejectsDifferentScalesEstablishedByInitialization)
{
    // The 16X and 32X share a hardware type but establish different scales.
    initializedScales[1] = 0.14f;

    EXPECT_TRUE(gyroInit());
    EXPECT_EQ(2u, initializationCount);
    EXPECT_EQ(GYRO_MASK(0), gyro.gyroEnabledBitmask);
    EXPECT_EQ(GYRO_MASK(0), gyroConfig()->gyro_enabled_bitmask);
    EXPECT_EQ(1u, eepromWriteCount);
    EXPECT_EQ(&gyro.gyroSensor[0].gyroDev, gyroActiveDev());
    EXPECT_FLOAT_EQ(0.07f, gyro.scale);
}

TEST_F(SensorGyroInit, RejectsDifferentSampleRatesEstablishedByInitialization)
{
    initializedRates[1] = 6664;

    EXPECT_TRUE(gyroInit());
    EXPECT_EQ(GYRO_MASK(0), gyro.gyroEnabledBitmask);
    EXPECT_EQ(GYRO_MASK(0), gyroConfig()->gyro_enabled_bitmask);
    EXPECT_EQ(1u, eepromWriteCount);
    EXPECT_EQ(8000, gyro.sampleRateHz);
}

TEST_F(SensorGyroInit, SelectsEnabledSecondSensor)
{
    gyroConfigMutable()->gyro_enabled_bitmask = GYRO_MASK(1);
    initializedRates[1] = 6664;
    initializedScales[1] = 0.14f;

    EXPECT_TRUE(gyroInit());
    EXPECT_EQ(GYRO_MASK(1), gyro.gyroEnabledBitmask);
    EXPECT_EQ(GYRO_MASK(1), gyroConfig()->gyro_enabled_bitmask);
    EXPECT_EQ(0u, eepromWriteCount);
    EXPECT_EQ(&gyro.gyroSensor[1].gyroDev, gyroActiveDev());
    EXPECT_EQ(6664, gyro.sampleRateHz);
    EXPECT_FLOAT_EQ(0.14f, gyro.scale);
}

TEST_F(SensorGyroInit, ExcludesDisabledSensorFromOverflowProtection)
{
    initializedScales[1] = 0.14f;
    initializedHardware[1] = GYRO_ICM20689;

    EXPECT_TRUE(gyroInit());
    EXPECT_EQ(GYRO_MASK(0), gyro.gyroEnabledBitmask);
    EXPECT_FALSE(gyro.gyroSensor[1].gyroDev.gyroHasOverflowProtection);
    EXPECT_TRUE(gyro.gyroHasOverflowProtection);
}

TEST_F(SensorGyroInit, IncludesEveryEnabledSensorInOverflowProtection)
{
    initializedHardware[1] = GYRO_ICM20689;

    EXPECT_TRUE(gyroInit());
    EXPECT_EQ(GYRO_MASK(0) | GYRO_MASK(1), gyro.gyroEnabledBitmask);
    EXPECT_FALSE(gyro.gyroHasOverflowProtection);
}

extern "C" {

uint8_t debugMode;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t detectedSensors[SENSOR_INDEX_COUNT];
uint8_t detectedGyros[GYRO_COUNT];

static void initializeGyro(gyroDev_t *dev)
{
    for (int i = 0; i < GYRO_COUNT; i++) {
        if (dev == &gyro.gyroSensor[i].gyroDev) {
            dev->gyroSampleRateHz = initializedRates[i];
            dev->scale = initializedScales[i];
            dev->gyroHardware = initializedHardware[i];
            initializationCount++;
            return;
        }
    }
}

bool virtualGyroDetect(gyroDev_t *dev)
{
    dev->initFn = initializeGyro;
    return true;
}

uint32_t micros(void) { return 0; }
void beeper(beeperMode_e) {}
timeDelta_t getGyroUpdateRate(void) { return gyro.targetLooptime; }
void sensorsSet(uint32_t) {}
void schedulerResetTaskStatistics(taskId_e) {}
uint16_t getAverageSystemLoadPercent(void) { return 0; }
int getArmingDisableFlags(void) { return 0; }
void writeEEPROM(void) { eepromWriteCount++; }

}
