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
#include <string.h>

extern "C" {
#include "platform.h"

#include "config/feature.h"

#include "drivers/dshot.h"

#include "sensors/esc_sensor.h"
#include "sensors/motor_telemetry.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Stubs

extern "C" {

static uint8_t motorCount;
static bool escSensorFeature;
static escSensorData_t escSensorData[MAX_SUPPORTED_MOTORS];
static float dshotRpm[MAX_SUPPORTED_MOTORS];
static timeMs_t nowMs;

bool useDshotTelemetry;
dshotTelemetryState_t dshotTelemetryState;

bool featureIsEnabled(uint32_t mask)
{
    return (mask & FEATURE_ESC_SENSOR) && escSensorFeature;
}

timeMs_t millis(void)
{
    return nowMs;
}

uint8_t getMotorCount(void)
{
    return motorCount;
}

const escSensorData_t *escSensorGetData(uint8_t motorIndex)
{
    return &escSensorData[motorIndex];
}

bool isDshotMotorTelemetryActive(uint8_t motorIndex)
{
    return (dshotTelemetryState.motorState[motorIndex].telemetryTypes & (1 << DSHOT_TELEMETRY_TYPE_eRPM)) != 0;
}

float getDshotRpm(uint8_t motorIndex)
{
    return dshotRpm[motorIndex];
}

// 14 poles: RPM = eRPM / 7, escSensorData_t.rpm is in 100 eRPM
float erpmToRpm(uint32_t erpm)
{
    return erpm * 100.0f / 7.0f;
}

}

static void setDshotTelemetry(unsigned motor, float rpm, int temperature, int voltage, int current)
{
    dshotRpm[motor] = rpm;
    dshotTelemetryMotorState_t *state = &dshotTelemetryState.motorState[motor];
    state->telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_eRPM;
    if (temperature >= 0) {
        state->telemetryTypes |= 1 << DSHOT_TELEMETRY_TYPE_TEMPERATURE;
        state->telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE] = temperature;
    }
    if (voltage >= 0) {
        state->telemetryTypes |= 1 << DSHOT_TELEMETRY_TYPE_VOLTAGE;
        state->telemetryData[DSHOT_TELEMETRY_TYPE_VOLTAGE] = voltage;
    }
    if (current >= 0) {
        state->telemetryTypes |= 1 << DSHOT_TELEMETRY_TYPE_CURRENT;
        state->telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT] = current;
    }
    state->telemetryUpdated |= state->telemetryTypes;
}

static void setEscSensorTelemetry(unsigned motor, uint8_t dataAge, int16_t rpm, uint8_t temperature, uint16_t voltage, int32_t current, int32_t consumption)
{
    escSensorData[motor] = (escSensorData_t){
        .dataAge = dataAge,
        .temperature = temperature,
        .voltage = voltage,
        .current = current,
        .consumption = consumption,
        .rpm = rpm,
    };
}

class MotorTelemetryTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        motorCount = 4;
        escSensorFeature = false;
        useDshotTelemetry = false;
        memset(&dshotTelemetryState, 0, sizeof(dshotTelemetryState));
        memset(escSensorData, 0, sizeof(escSensorData));
        memset(dshotRpm, 0, sizeof(dshotRpm));
        nowMs = 100000;
        for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            escSensorData[i].dataAge = ESC_DATA_INVALID;
        }
    }
};

TEST_F(MotorTelemetryTest, NoSourceEnabled)
{
    EXPECT_FALSE(motorTelemetryIsEnabled());
    EXPECT_EQ(nullptr, motorTelemetryGet(0));
    EXPECT_EQ(nullptr, motorTelemetryGet(MOTOR_TELEMETRY_COMBINED));
}

TEST_F(MotorTelemetryTest, MotorIndexOutOfRange)
{
    useDshotTelemetry = true;
    EXPECT_NE(nullptr, motorTelemetryGet(3));
    EXPECT_EQ(nullptr, motorTelemetryGet(4));
    EXPECT_EQ(nullptr, motorTelemetryGet(MOTOR_TELEMETRY_COMBINED - 1));
    EXPECT_NE(nullptr, motorTelemetryGet(MOTOR_TELEMETRY_COMBINED));
}

TEST_F(MotorTelemetryTest, DshotOnlyConvertsUnits)
{
    useDshotTelemetry = true;
    setDshotTelemetry(0, 12000.0f, 45, 66, 12); // 66 * 0.25V = 16.5V, 12A

    const motorTelemetry_t *telemetry = motorTelemetryGet(0);
    ASSERT_NE(nullptr, telemetry);
    EXPECT_EQ(MOTOR_TELEMETRY_SOURCE_DSHOT, telemetry->sources);
    EXPECT_EQ(MOTOR_TELEMETRY_RPM | MOTOR_TELEMETRY_TEMPERATURE | MOTOR_TELEMETRY_VOLTAGE | MOTOR_TELEMETRY_CURRENT, telemetry->valid);
    EXPECT_EQ(12000u, telemetry->rpm);
    EXPECT_EQ(45, telemetry->temperature);
    EXPECT_EQ(1650, telemetry->voltage);  // 0.01V
    EXPECT_EQ(1200, telemetry->current);  // 0.01A
    EXPECT_FALSE(motorTelemetryHas(telemetry, MOTOR_TELEMETRY_CONSUMPTION));
    EXPECT_EQ(0, telemetry->dataAge);
}

TEST_F(MotorTelemetryTest, DshotWithoutExtendedTelemetryHasRpmOnly)
{
    useDshotTelemetry = true;
    setDshotTelemetry(0, 9000.0f, -1, -1, -1);

    const motorTelemetry_t *telemetry = motorTelemetryGet(0);
    EXPECT_EQ(MOTOR_TELEMETRY_RPM, telemetry->valid);
}

TEST_F(MotorTelemetryTest, DshotMotorNotActiveHasNoData)
{
    useDshotTelemetry = true;

    const motorTelemetry_t *telemetry = motorTelemetryGet(1);
    ASSERT_NE(nullptr, telemetry);
    EXPECT_EQ(0, telemetry->valid);
    EXPECT_EQ(ESC_DATA_INVALID, telemetry->dataAge);
}

TEST_F(MotorTelemetryTest, EscSensorOnly)
{
    escSensorFeature = true;
    setEscSensorTelemetry(0, 2, 700, 50, 1620, 1530, 250); // 700 * 100 eRPM / 7 = 10000 RPM

    const motorTelemetry_t *telemetry = motorTelemetryGet(0);
    ASSERT_NE(nullptr, telemetry);
    EXPECT_EQ(MOTOR_TELEMETRY_SOURCE_ESC_SENSOR, telemetry->sources);
    EXPECT_EQ(10000u, telemetry->rpm);
    EXPECT_EQ(50, telemetry->temperature);
    EXPECT_EQ(1620, telemetry->voltage);
    EXPECT_EQ(1530, telemetry->current);
    EXPECT_EQ(250, telemetry->consumption);
    EXPECT_EQ(2, telemetry->dataAge);
}

TEST_F(MotorTelemetryTest, EscSensorInvalidDataIsIgnored)
{
    escSensorFeature = true;
    setEscSensorTelemetry(0, ESC_DATA_INVALID, 700, 50, 1620, 1530, 250);

    const motorTelemetry_t *telemetry = motorTelemetryGet(0);
    ASSERT_NE(nullptr, telemetry);
    EXPECT_EQ(0, telemetry->valid);
}

TEST_F(MotorTelemetryTest, EscSensorStaleDataIsIgnored)
{
    escSensorFeature = true;
    setEscSensorTelemetry(0, ESC_BATTERY_AGE_MAX, 700, 50, 1620, 1530, 250);
    setEscSensorTelemetry(1, ESC_BATTERY_AGE_MAX + 1, 700, 50, 1620, 1530, 250);

    EXPECT_EQ(ESC_BATTERY_AGE_MAX, motorTelemetryGet(0)->dataAge);
    EXPECT_TRUE(motorTelemetryHas(motorTelemetryGet(0), MOTOR_TELEMETRY_VOLTAGE));
    EXPECT_EQ(0, motorTelemetryGet(1)->valid);
}

// A DShot extended telemetry value that does not update expires
TEST_F(MotorTelemetryTest, DshotExtendedTelemetryExpires)
{
    useDshotTelemetry = true;
    setDshotTelemetry(0, 12000.0f, 40, 64, 10);
    EXPECT_TRUE(motorTelemetryHas(motorTelemetryGet(0), MOTOR_TELEMETRY_VOLTAGE));

    // Voltage updates, temperature and current do not
    nowMs += 4000;
    dshotTelemetryState.motorState[0].telemetryUpdated |= 1 << DSHOT_TELEMETRY_TYPE_VOLTAGE;
    EXPECT_TRUE(motorTelemetryHas(motorTelemetryGet(0), MOTOR_TELEMETRY_TEMPERATURE));

    nowMs += 1000;
    const motorTelemetry_t *telemetry = motorTelemetryGet(0);
    EXPECT_FALSE(motorTelemetryHas(telemetry, MOTOR_TELEMETRY_TEMPERATURE));
    EXPECT_FALSE(motorTelemetryHas(telemetry, MOTOR_TELEMETRY_CURRENT));
    EXPECT_TRUE(motorTelemetryHas(telemetry, MOTOR_TELEMETRY_VOLTAGE));
    // RPM does not expire here, DShot telemetry is still active
    EXPECT_TRUE(motorTelemetryHas(telemetry, MOTOR_TELEMETRY_RPM));
}

// Stale ESC sensor data must not replace fresh DShot data
TEST_F(MotorTelemetryTest, FreshDshotVoltageWhenEscSensorDataIsStale)
{
    useDshotTelemetry = true;
    escSensorFeature = true;
    setDshotTelemetry(0, 12000.0f, 40, 64, -1);
    setEscSensorTelemetry(0, ESC_BATTERY_AGE_MAX + 1, 700, 55, 1610, 1520, 300);

    const motorTelemetry_t *telemetry = motorTelemetryGet(0);
    ASSERT_NE(nullptr, telemetry);
    EXPECT_EQ(MOTOR_TELEMETRY_SOURCE_DSHOT, telemetry->sources);
    EXPECT_EQ(1600, telemetry->voltage);
    EXPECT_EQ(0, telemetry->dataAge);
    EXPECT_FALSE(motorTelemetryHas(telemetry, MOTOR_TELEMETRY_CURRENT));
    EXPECT_FALSE(motorTelemetryHas(telemetry, MOTOR_TELEMETRY_CONSUMPTION));
}

TEST_F(MotorTelemetryTest, BothSourcesMergeByField)
{
    useDshotTelemetry = true;
    escSensorFeature = true;
    setDshotTelemetry(0, 12000.0f, 40, 64, 10);
    setEscSensorTelemetry(0, 1, 700, 55, 1610, 1520, 300);

    const motorTelemetry_t *telemetry = motorTelemetryGet(0);
    ASSERT_NE(nullptr, telemetry);
    EXPECT_EQ(MOTOR_TELEMETRY_SOURCE_DSHOT | MOTOR_TELEMETRY_SOURCE_ESC_SENSOR, telemetry->sources);
    EXPECT_EQ(12000u, telemetry->rpm);      // DShot
    EXPECT_EQ(55, telemetry->temperature);  // ESC sensor
    EXPECT_EQ(1610, telemetry->voltage);    // ESC sensor
    EXPECT_EQ(1520, telemetry->current);    // ESC sensor
    EXPECT_EQ(300, telemetry->consumption); // ESC sensor
    EXPECT_EQ(1, telemetry->dataAge);       // age of the ESC sensor data
}

TEST_F(MotorTelemetryTest, EscSensorRpmWhenDshotNotActive)
{
    useDshotTelemetry = true;
    escSensorFeature = true;
    setEscSensorTelemetry(0, 0, 700, 55, 1610, 1520, 300);

    const motorTelemetry_t *telemetry = motorTelemetryGet(0);
    EXPECT_EQ(MOTOR_TELEMETRY_SOURCE_ESC_SENSOR, telemetry->sources);
    EXPECT_EQ(10000u, telemetry->rpm);
}

TEST_F(MotorTelemetryTest, CombinedUsesValidMotorsOnly)
{
    useDshotTelemetry = true;
    setDshotTelemetry(0, 10000.0f, 40, 64, 10);
    setDshotTelemetry(1, 12000.0f, 60, 68, 20);
    // Motors 2 and 3 have no telemetry

    const motorTelemetry_t *combined = motorTelemetryGet(MOTOR_TELEMETRY_COMBINED);
    ASSERT_NE(nullptr, combined);
    EXPECT_EQ(11000u, combined->rpm);       // average
    EXPECT_EQ(60, combined->temperature);   // maximum
    EXPECT_EQ(1650, combined->voltage);     // average of 16.00V and 17.00V
    EXPECT_TRUE(motorTelemetryHas(combined, MOTOR_TELEMETRY_RPM));
    EXPECT_TRUE(motorTelemetryHas(combined, MOTOR_TELEMETRY_TEMPERATURE));
    EXPECT_TRUE(motorTelemetryHas(combined, MOTOR_TELEMETRY_VOLTAGE));
    // A sum of 2 of the 4 motors is not valid
    EXPECT_FALSE(motorTelemetryHas(combined, MOTOR_TELEMETRY_CURRENT));
    // Motors 2 and 3 have no data, they do not make the data stale
    EXPECT_EQ(0, combined->dataAge);
}

TEST_F(MotorTelemetryTest, CombinedNoDataIsInvalid)
{
    useDshotTelemetry = true;

    const motorTelemetry_t *combined = motorTelemetryGet(MOTOR_TELEMETRY_COMBINED);
    ASSERT_NE(nullptr, combined);
    EXPECT_EQ(0, combined->valid);
    EXPECT_EQ(ESC_DATA_INVALID, combined->dataAge);
}

// ESC sensor data of motor 3 is not valid, but DShot RPM is available.
// The current and consumption sums must not be valid.
TEST_F(MotorTelemetryTest, CombinedSumNotValidWhenOneMotorHasNoEscSensorData)
{
    useDshotTelemetry = true;
    escSensorFeature = true;
    for (unsigned i = 0; i < 4; i++) {
        setDshotTelemetry(i, 10000.0f, -1, -1, -1);
    }
    for (unsigned i = 0; i < 3; i++) {
        setEscSensorTelemetry(i, 1, 700, 50, 1600, 1000, 100);
    }

    const motorTelemetry_t *combined = motorTelemetryGet(MOTOR_TELEMETRY_COMBINED);
    ASSERT_NE(nullptr, combined);
    EXPECT_FALSE(motorTelemetryHas(combined, MOTOR_TELEMETRY_CURRENT));
    EXPECT_FALSE(motorTelemetryHas(combined, MOTOR_TELEMETRY_CONSUMPTION));
    EXPECT_TRUE(motorTelemetryHas(combined, MOTOR_TELEMETRY_VOLTAGE));
    EXPECT_EQ(1600, combined->voltage);
    EXPECT_EQ(1, combined->dataAge);
}

// One motor without telemetry does not make the voltage of the other motors stale
TEST_F(MotorTelemetryTest, CombinedVoltageAgeIgnoresMotorWithoutData)
{
    escSensorFeature = true;
    for (unsigned i = 0; i < 3; i++) {
        setEscSensorTelemetry(i, 2, 700, 50, 1600, 1000, 100);
    }
    // Motor 3: ESC sensor data is not valid

    const motorTelemetry_t *combined = motorTelemetryGet(MOTOR_TELEMETRY_COMBINED);
    ASSERT_NE(nullptr, combined);
    EXPECT_TRUE(motorTelemetryHas(combined, MOTOR_TELEMETRY_VOLTAGE));
    EXPECT_EQ(1600, combined->voltage);
    EXPECT_EQ(2, combined->dataAge);
    EXPECT_LE(combined->dataAge, ESC_BATTERY_AGE_MAX);
}

TEST_F(MotorTelemetryTest, CombinedEscSensor)
{
    escSensorFeature = true;
    motorCount = 2;
    setEscSensorTelemetry(0, 3, 700, 50, 1600, 1000, 100);
    setEscSensorTelemetry(1, 5, 1400, 70, 1620, 2000, 200);

    const motorTelemetry_t *combined = motorTelemetryGet(MOTOR_TELEMETRY_COMBINED);
    ASSERT_NE(nullptr, combined);
    EXPECT_EQ(15000u, combined->rpm);
    EXPECT_EQ(70, combined->temperature);
    EXPECT_EQ(1610, combined->voltage);
    EXPECT_EQ(3000, combined->current);
    EXPECT_EQ(300, combined->consumption);
    EXPECT_EQ(5, combined->dataAge);
}
