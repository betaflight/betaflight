/*
 * This file is part of Betaflight.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <cstring>

extern "C" {
#include "platform.h"
#include "target.h"
#include "common/time.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp5xx.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "fc/runtime_config.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"
}

#include "gtest/gtest.h"

namespace {
busDevice_t bus;
uint8_t detectedChip;
bool devicePresent;
uint32_t enabledSensors;

class Bmp5xxDetectionTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        std::memset(&baro, 0, sizeof(baro));
        std::memset(barometerConfigMutable(), 0, sizeof(barometerConfig_t));
        std::memset(detectedSensors, 0, sizeof(detectedSensors));
        bus = {};
        bus.busType = BUS_TYPE_I2C;
        barometerConfigMutable()->baro_busType = BUS_TYPE_I2C;
        detectedChip = 0x50;
        devicePresent = true;
        enabledSensors = 0;
    }

    void detect(baroSensor_e requested)
    {
        barometerConfigMutable()->baro_hardware = requested;
        baroInit();
    }
};

TEST_F(Bmp5xxDetectionTest, KeepsExplicitBmp581WithItsDocumentedPrimaryChipId)
{
    detect(BARO_BMP581);
    EXPECT_TRUE(isBaroReady());
    EXPECT_EQ(BARO_BMP581, detectedSensors[SENSOR_INDEX_BARO]);
    EXPECT_NE(0u, enabledSensors & SENSOR_BARO);
}

TEST_F(Bmp5xxDetectionTest, KeepsExplicitBmp580WithAnAcceptedSecondaryChipId)
{
    detectedChip = 0x51;
    detect(BARO_BMP580);
    EXPECT_TRUE(isBaroReady());
    EXPECT_EQ(BARO_BMP580, detectedSensors[SENSOR_INDEX_BARO]);
}

TEST_F(Bmp5xxDetectionTest, PreservesTheLegacyAutomaticFamilyLabels)
{
    detect(BARO_DEFAULT);
    EXPECT_EQ(BARO_BMP580, detectedSensors[SENSOR_INDEX_BARO]);
    detectedChip = 0x51;
    detect(BARO_DEFAULT);
    EXPECT_EQ(BARO_BMP581, detectedSensors[SENSOR_INDEX_BARO]);
}

TEST_F(Bmp5xxDetectionTest, DoesNotAcceptAMissingFamilyDevice)
{
    devicePresent = false;
    detect(BARO_BMP581);
    EXPECT_FALSE(isBaroReady());
    EXPECT_EQ(0u, enabledSensors & SENSOR_BARO);
}
} // namespace

extern "C" {
uint8_t detectedSensors[SENSOR_INDEX_COUNT];

bool i2cBusSetInstance(extDevice_t *dev, uint32_t)
{
    dev->bus = &bus;
    return true;
}

bool bmp5xxDetect(const bmp5xxConfig_t *, baroDev_t *, uint8_t *chip)
{
    if (devicePresent && chip) {
        *chip = detectedChip;
    }
    return devicePresent;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}
}
