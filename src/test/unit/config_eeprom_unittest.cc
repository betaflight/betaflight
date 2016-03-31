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
    #include "build_config.h"
    
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
    #include "io/transponder_ir.h"

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
    #include "config/config_system.h"
    #include "config/feature.h"

    #include "platform.h"

    PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);
    PG_REGISTER_PROFILE(gimbalConfig_t, gimbalConfig, PG_GIMBAL_CONFIG, 0);
    PG_REGISTER_PROFILE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);

    pidProfile_t testPidProfile[MAX_PROFILE_COUNT];
    pidProfile_t *pidProfile = &testPidProfile[0];

    typedef struct someProfileSpecificData_s {
        uint8_t uint8;
        uint16_t uint16;
        uint32_t uint32;
    } PG_PACKED someProfileSpecificData_t;

    PG_REGISTER(master_t, masterConfig, 0, 0);
    PG_REGISTER_PROFILE(someProfileSpecificData_t, someProfileSpecificData, 1, 0);

    PG_REGISTER(escAndServoConfig_t, escAndServoConfig, PG_ESC_AND_SERVO_CONFIG, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(sensorTrims_t, sensorTrims, PG_SENSOR_TRIMS, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER_ARR(controlRateConfig_t, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 0);
    PG_REGISTER(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);
    PG_REGISTER(pwmRxConfig_t, pwmRxConfig, PG_DRIVER_PWM_RX_CONFIG, 0);
    PG_REGISTER(armingConfig_t, armingConfig, PG_ARMING_CONFIG, 0);
    PG_REGISTER(transponderConfig_t, transponderConfig, PG_TRANSPONDER_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER(imuConfig_t, imuConfig, PG_IMU_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(motor3DConfig_t, motor3DConfig, PG_MOTOR_3D_CONFIG, 0);
    
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


TEST(configTest, resetEEPROM)
{
    resetEEPROM();
}

TEST(configTest, modify)
{
    resetEEPROM();

    imuConfig.looptime = 123;
    writeEEPROM();

    EXPECT_EQ(123, imuConfig.looptime);

    // overwrite the values with something else before loading
    imuConfig.looptime = 456;

    readEEPROM();
    EXPECT_EQ(123, imuConfig.looptime);
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
void mixerUseConfigs(servoParam_t *, airplaneConfig_t *) {}
#else
void mixerUseConfigs(airplaneConfig_t *) {}
#endif
bool isSerialConfigValid(serialConfig_t *) {return true;}
void imuConfigure(imuRuntimeConfig_t *, accDeadband_t *,float ,uint16_t) {}
void gpsUseProfile(gpsProfile_t *) {}
void gpsUsePIDs(pidProfile_t *) {}
void generateYawCurve(controlRateConfig_t *) {}
void generatePitchRollCurve(controlRateConfig_t *) {}
void generateThrottleCurve(controlRateConfig_t *) {}
void delay(uint32_t) {}
void configureAltitudeHold(barometerConfig_t *) {}

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

