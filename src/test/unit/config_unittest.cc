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
#include <stdbool.h>

extern "C" {
    #include "platform.h"

    #include "blackbox/blackbox.h"

    #include "common/sensor_alignment.h"

    #include "config/config.h"
    #include "config/config_eeprom.h"
    #include "config/feature.h"

    #include "drivers/dshot.h"
    #include "drivers/motor.h"

    #include "fc/controlrate_profile.h"
    #include "fc/core.h"
    #include "fc/rc.h"
    #include "fc/rc_adjustments.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/pid_init.h"
    #include "flight/servos.h"

    #include "io/beeper.h"
    #include "io/gps.h"
    #include "io/ledstrip.h"
    #include "io/serial.h"

    #include "msp/msp_box.h"

    #include "pg/beeper.h"
    #include "pg/gyrodev.h"
    #include "pg/motor.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "rx/rx.h"

    #include "scheduler/scheduler.h"

    #include "sensors/acceleration.h"
    #include "sensors/battery.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"

    #include "telemetry/telemetry.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 2);
    PG_REGISTER_ARRAY(batteryProfile_t, BATTERY_PROFILE_COUNT, batteryProfiles, PG_BATTERY_PROFILES, 1);
    PG_REGISTER(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 3);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 4);
    PG_REGISTER(compassConfig_t, compassConfig, PG_COMPASS_CONFIG, 4);
    PG_REGISTER_ARRAY(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles,
        PG_CONTROL_RATE_PROFILES, 7);
    PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 2);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 4);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 10);
    PG_REGISTER_ARRAY(gyroDeviceConfig_t, MAX_GYRODEV_COUNT, gyroDeviceConfig, PG_GYRO_DEVICE_CONFIG, 1);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 2);
    PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions,
        PG_MODE_ACTIVATION_PROFILE, 5);
    PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 3);
    PG_REGISTER(pidConfig_t, pidConfig, PG_PID_CONFIG, 4);
    PG_REGISTER_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 13);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 4);
    PG_REGISTER(serialPinConfig_t, serialPinConfig, PG_SERIAL_PIN_CONFIG, 0);
    PG_REGISTER(throttleCorrectionConfig_t, throttleCorrectionConfig, PG_THROTTLE_CORRECTION_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Position of the last call to each stub in the call sequence (0: no call)
static int callSequence;
static int pidInitCall;
static int initRcProcessingCall;

class ChangePidProfileTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        pgResetAll();
        callSequence = 0;
        pidInitCall = 0;
        initRcProcessingCall = 0;
    }
};

// initRcProcessing() copies feedforward settings from pidRuntime, and
// pidInit() sets pidRuntime from the new profile. Thus a PID profile
// switch must call initRcProcessing() after pidInit().
TEST_F(ChangePidProfileTest, InitialisesRcProcessingAfterPidInit)
{
    changePidProfile(1);

    EXPECT_EQ(1, getCurrentPidProfileIndex());
    EXPECT_GT(pidInitCall, 0);
    EXPECT_GT(initRcProcessingCall, pidInitCall);
}

TEST_F(ChangePidProfileTest, IgnoresProfileIndexOutOfRange)
{
    changePidProfile(PID_PROFILE_COUNT);

    EXPECT_EQ(0, getCurrentPidProfileIndex());
    EXPECT_EQ(0, pidInitCall);
    EXPECT_EQ(0, initRcProcessingCall);
}

// STUBS

extern "C" {
    gyro_t gyro;
    uint32_t featuresSupportedByBuild = 0;
    const mixer_t mixers[1] = {};
    const mixerRules_t servoMixers[1] = {};
    const ratesSettingsLimits_t ratesSettingLimits[RATES_TYPE_COUNT] = {};

    void pidInit(const pidProfile_t *) { pidInitCall = ++callSequence; }
    void initRcProcessing(void) { initRcProcessingCall = ++callSequence; }

    void accInitFilters(void) { }
    void activeAdjustmentRangeReset(void) { }
    void beeperConfirmationBeeps(uint8_t) { }
    void buildAlignmentFromStandardAlignment(sensorAlignment_t *, sensor_align_e) { }
    bool checkMotorProtocolEnabled(const motorDevConfig_t *, bool *) { return false; }
    configResetReason_e diagnoseEEPROM(uint16_t *) { return (configResetReason_e)0; }
    void failsafeReset(void) { }
    void featureDisableImmediate(const uint32_t) { }
    void featureEnableImmediate(const uint32_t) { }
    void featureInit(void) { }
    bool featureIsConfigured(const uint32_t) { return false; }
    void imuConfigure(uint16_t, uint8_t) { }
    void initActiveBoxIds(void) { }
    void initEscEndpoints(void) { }
    bool isEEPROMStructureValid(void) { return true; }
    bool isModeActivationConditionLinked(boxId_e) { return false; }
    bool isModeActivationConditionPresent(boxId_e) { return false; }
    bool isSerialConfigValid(void) { return true; }
    void loadBatteryProfile(void) { }
    void loadControlRateProfile(void) { }
    bool loadEEPROM(void) { return true; }
    void mixerInitProfile(void) { }
    bool mixerModeIsFixedWing(mixerMode_e) { return false; }
    void rcControlsInit(void) { }
    void reevaluateLedConfig(void) { }
    void removeModeActivationCondition(boxId_e) { }
    void resumeRxSignal(void) { }
    void schedulerIgnoreTaskExecTime(void) { }
    void serialDropConflictingAssignments(void) { }
    void serialRemovePort(serialPortIdentifier_e) { }
    void serialResetFeatureAssignments(void) { }
    void setAccelerationTrims(union flightDynamicsTrims_u *) { }
    void setArmingDisabled(armingDisableFlags_e) { }
    void suspendRxSignal(void) { }
    void telemetryValidateProviders(void) { }
    void validateAndfixMotorOutputReordering(uint8_t *, const unsigned) { }
    void writeConfigToEEPROM(void) { }
}
