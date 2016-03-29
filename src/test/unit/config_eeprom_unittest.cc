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
#include <stddef.h>

extern "C" {
    #include "platform.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/color.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "flight/pid.h"

    #include "drivers/sensor.h"
    #include "drivers/timer.h"
    #include "drivers/accgyro.h"
    #include "drivers/pwm_rx.h"
    #include "drivers/serial.h"

    #include "io/rc_controls.h"
    #include "io/escservo.h"
    #include "io/gimbal.h"
    #include "io/gps.h"
    #include "io/serial.h"
    #include "io/ledstrip.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/gyro.h"
    #include "sensors/battery.h"
    #include "sensors/boardalignment.h"

    #include "flight/mixer.h"
    #include "flight/imu.h"
    #include "flight/navigation.h"
    #include "flight/failsafe.h"

    #include "telemetry/telemetry.h"

    #include "config/config.h"
    #include "config/config_eeprom.h"
    #include "config/config_profile.h"
    #include "config/config_master.h"

    #include "platform.h"

    failsafeConfig_t failsafeConfig;

    extern profile_t profileStorage[MAX_PROFILE_COUNT];

    gimbalConfig_t testGimbalConfig[MAX_PROFILE_COUNT];
    gimbalConfig_t *gimbalConfig = &testGimbalConfig[0];

    pidProfile_t testPidProfile[MAX_PROFILE_COUNT];
    pidProfile_t *pidProfile = &testPidProfile[0];

    typedef struct someProfileSpecificData_s {
        uint8_t uint8;
        uint16_t uint16;
        uint32_t uint32;
    } PG_PACKED someProfileSpecificData_t;

    someProfileSpecificData_t someProfileSpecificDataStorage[MAX_PROFILE_COUNT];
    someProfileSpecificData_t *someProfileSpecificData;

    escAndServoConfig_t escAndServoConfig;
    gyroConfig_t gyroConfig;
    sensorTrims_t sensorTrims;
    batteryConfig_t batteryConfig;
    controlRateConfig_t controlRateProfiles[MAX_CONTROL_RATE_PROFILE_COUNT];
    serialConfig_t serialConfig;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


const pgRegistry_t __pg_registry[] =
{
    {
        .base = (uint8_t *)&masterConfig,
        .ptr = 0,
        .size = sizeof(masterConfig),
        .pgn = PG_MASTER,
        .format = 0,
        .flags = PGC_SYSTEM
    },
    {
        .base = (uint8_t *)&profileStorage,
        .ptr = (uint8_t **)&currentProfile,
        .size = sizeof(profileStorage[0]),
        .pgn = PG_PROFILE,
        .format = 0,
        .flags = PGC_PROFILE
    },
    {
        .base = (uint8_t *)&someProfileSpecificDataStorage,
        .ptr = (uint8_t **)&someProfileSpecificData,
        .size = sizeof(someProfileSpecificDataStorage[0]),
        .pgn = 1,
        .format = 0,
        .flags = PGC_PROFILE
    },
    {
        .base = nullptr,
        .ptr = 0,
        .size = 0,
        .pgn = 0,
        .format = 0,
        .flags = 0
    },
};

TEST(configTest, resetEEPROM)
{
    resetEEPROM();
}

TEST(configTest, modify)
{
    resetEEPROM();

    masterConfig.looptime = 123;
    writeEEPROM();

    EXPECT_EQ(123, masterConfig.looptime);

    // overwrite the values with something else before loading
    masterConfig.looptime = 456;

    readEEPROM();
    EXPECT_EQ(123, masterConfig.looptime);
}

TEST(configTest, modifyProfile)
{
    resetEEPROM();

    // default profile is 0

    someProfileSpecificData->uint32 = 1;
    someProfileSpecificData->uint16 = 2;
    someProfileSpecificData->uint8  = 3;
    changeProfile(1); // changing saves the EEPROM

    someProfileSpecificData->uint32 = 4;
    someProfileSpecificData->uint16 = 5;
    someProfileSpecificData->uint8  = 6;
    changeProfile(2); // changing saves the EEPROM

    someProfileSpecificData->uint32 = 7;
    someProfileSpecificData->uint16 = 8;
    someProfileSpecificData->uint8  = 9;
    changeProfile(0);  // changing saves the EEPROM

    EXPECT_EQ(1, someProfileSpecificData->uint32);
    EXPECT_EQ(2, someProfileSpecificData->uint16);
    EXPECT_EQ(3, someProfileSpecificData->uint8);

    changeProfile(1); // changing saves the EEPROM
    EXPECT_EQ(4, someProfileSpecificData->uint32);
    EXPECT_EQ(5, someProfileSpecificData->uint16);
    EXPECT_EQ(6, someProfileSpecificData->uint8);

    changeProfile(2); // changing saves the EEPROM
    EXPECT_EQ(7, someProfileSpecificData->uint32);
    EXPECT_EQ(8, someProfileSpecificData->uint16);
    EXPECT_EQ(9, someProfileSpecificData->uint8);
}

// STUBS
extern "C" {

void applyDefaultLedStripConfig(ledConfig_t *) {}
void applyDefaultColors(hsvColor_t *, uint8_t) {}
void beeperConfirmationBeeps(uint8_t) {}
void StopPwmAllMotors(void) {}
void useRxConfig(rxConfig_t *) {}
void useRcControlsConfig(modeActivationCondition_t *) {}
void useFailsafeConfig(void) {}
void useBarometerConfig(barometerConfig_t *) {}
void telemetryUseConfig(telemetryConfig_t *) {}
void suspendRxSignal(void) {}
void setAccelerationTrims(flightDynamicsTrims_t *) {}
void resumeRxSignal(void) {}
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfiguration_t *) {}
void resetAdjustmentStates(void) {}
void pidSetController(pidControllerType_e) {}
void parseRcChannels(const char *, rxConfig_t *) {}
#ifdef USE_SERVOS
void mixerUseConfigs(servoParam_t *, flight3DConfig_t *, mixerConfig_t *, airplaneConfig_t *, rxConfig_t *) {}
#else
void mixerUseConfigs(flight3DConfig_t *, escAndServoConfig_t *, mixerConfig_t *, airplaneConfig_t *, rxConfig_t *) {}
#endif
bool isSerialConfigValid(serialConfig_t *) {return true;}
void imuConfigure(imuRuntimeConfig_t *, accDeadband_t *,float ,uint16_t) {}
void gpsUseProfile(gpsProfile_t *) {}
void gpsUsePIDs(pidProfile_t *) {}
void generateYawCurve(controlRateConfig_t *) {}
void generatePitchRollCurve(controlRateConfig_t *) {}
void generateThrottleCurve(controlRateConfig_t *) {}
void delay(uint32_t) {}
void configureAltitudeHold(pidProfile_t *, barometerConfig_t *, rcControlsConfig_t *) {}

const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
#ifdef USE_VCP
    SERIAL_PORT_USB_VCP,
#endif
#ifdef USE_UART1
    SERIAL_PORT_UART1,
#endif
#ifdef USE_UART2
    SERIAL_PORT_UART2,
#endif
#ifdef USE_UART3
    SERIAL_PORT_UART3,
#endif
#ifdef USE_SOFTSERIAL1
    SERIAL_PORT_SOFTSERIAL1,
#endif
#ifdef USE_SOFTSERIAL2
    SERIAL_PORT_SOFTSERIAL2,
#endif
};
}

