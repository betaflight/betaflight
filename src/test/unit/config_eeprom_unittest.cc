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
    #include "build/build_config.h"
    
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/color.h"
    #include "common/utils.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "flight/pid.h"

    #include "drivers/sensor.h"
    #include "drivers/timer.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"
    #include "drivers/pwm_rx.h"
    #include "drivers/serial.h"

    #include "fc/rc_controls.h"
    #include "fc/rate_profile.h"
    #include "fc/rc_adjustments.h"
    #include "io/motor_and_servo.h"
    #include "io/gimbal.h"
    #include "io/gps.h"
    #include "io/serial.h"
    #include "io/ledstrip.h"
    #include "io/transponder_ir.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"
    #include "sensors/battery.h"
    #include "sensors/boardalignment.h"

    #include "flight/mixer.h"
    #include "flight/servos.h"
    #include "flight/imu.h"
    #include "flight/navigation.h"
    #include "flight/failsafe.h"
    #include "flight/altitudehold.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/frsky.h"
    #include "telemetry/hott.h"

    #include "config/config_eeprom.h"
    #include "config/config_system.h"
    #include "config/feature.h"
    #include "config/profile.h"

    #include "fc/config.h"

    #include "platform.h"

    PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);
    PG_REGISTER_PROFILE(gimbalConfig_t, gimbalConfig, PG_GIMBAL_CONFIG, 0);
    PG_REGISTER_PROFILE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER_PROFILE(pidProfile_t, pidProfile, PG_PID_PROFILE, 0);
    PG_REGISTER_PROFILE(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER_PROFILE(rateProfileSelection_t, rateProfileSelection, PG_RATE_PROFILE_SELECTION, 0);
    PG_REGISTER_PROFILE(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);
    PG_REGISTER_PROFILE(throttleCorrectionConfig_t, throttleCorrectionConfig, PG_THROTTLE_CORRECTION_CONFIG, 0);
    PG_REGISTER_PROFILE(compassConfig_t, compassConfig, PG_COMPASS_CONFIGURATION, 0);
    PG_REGISTER_PROFILE(gpsProfile_t, gpsProfile, PG_NAVIGATION_CONFIG, 0);
    PG_REGISTER_PROFILE(modeActivationProfile_t, modeActivationProfile, PG_MODE_ACTIVATION_PROFILE, 0);
    PG_REGISTER_PROFILE(servoProfile_t, servoProfile, PG_SERVO_PROFILE, 0);

    PG_REGISTER(motorAndServoConfig_t, motorAndServoConfig, PG_MOTOR_AND_SERVO_CONFIG, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(sensorTrims_t, sensorTrims, PG_SENSOR_TRIMS, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER_ARR(controlRateConfig_t, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 0);
    PG_REGISTER(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);
    PG_REGISTER(pwmRxConfig_t, pwmRxConfig, PG_DRIVER_PWM_RX_CONFIG, 0);
    PG_REGISTER(armingConfig_t, armingConfig, PG_ARMING_CONFIG, 0);
    PG_REGISTER(transponderConfig_t, transponderConfig, PG_TRANSPONDER_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER(imuConfig_t, imuConfig, PG_IMU_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(motor3DConfig_t, motor3DConfig, PG_MOTOR_3D_CONFIG, 0);
    PG_REGISTER(airplaneConfig_t, airplaneConfig, PG_AIRPLANE_ALT_HOLD_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(frskyTelemetryConfig_t, frskyTelemetryConfig, PG_FRSKY_TELEMETRY_CONFIG, 0);
    PG_REGISTER(hottTelemetryConfig_t, hottTelemetryConfig, PG_HOTT_TELEMETRY_CONFIG, 0);

    PG_REGISTER(boardAlignment_t, boardAlignment, PG_BOARD_ALIGNMENT, 1);
    PG_REGISTER(sensorSelectionConfig_t, sensorSelectionConfig, PG_SENSOR_SELECTION_CONFIG, 0);
    PG_REGISTER(sensorAlignmentConfig_t, sensorAlignmentConfig, PG_SENSOR_ALIGNMENT_CONFIG, 0);
    PG_REGISTER_ARR(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);

    typedef struct someSystemData_s {
        uint32_t uint32;
        uint16_t uint16;
        uint8_t uint8;
    }  PG_PACKED someSystemData_t;

    PG_DECLARE(someSystemData_t, someSystemData);
    PG_REGISTER_WITH_RESET_FN(someSystemData_t, someSystemData, PG_RESERVED_FOR_TESTING_1, 0);
    void pgResetFn_someSystemData(someSystemData_t *instance)
    {
        instance->uint32 = 0xFFFFFFFF;
        instance->uint16 = 0xAAAA;
        instance->uint8 = 0x88;
    }

    PG_DECLARE(someSystemData_t, someSystemData_NR);  // NullReset
    PG_REGISTER(someSystemData_t, someSystemData_NR, 0, 0);

    PG_DECLARE(someSystemData_t, someSystemData_TR);  // TemplateReset
    PG_REGISTER_WITH_RESET_TEMPLATE(someSystemData_t, someSystemData_TR, 0, 0);
    PG_RESET_TEMPLATE(someSystemData_t, someSystemData_TR,
        .uint32 = 0xFFFFFFFF,
        .uint16 = 0xAAAA,
        .uint8 = 0x88,
    );

    typedef struct someSystemArrayItem_s {
        uint16_t uint16;
        uint32_t uint32;
        uint8_t uint8;
    }  PG_PACKED someSystemArrayItem_t;

    PG_DECLARE_ARR(someSystemArrayItem_t, 10, someSystemArray);

    void pgResetFn_someSystemArray(someSystemArrayItem_t *instance)
    {
        for (int i = 0; i < 10; i++) {
            instance->uint16 = 0xFFFF;
            instance->uint32 = 0xAAAAAAAA;
            instance->uint8 = 0x88;

            instance++;
        }
    }

    PG_REGISTER_ARR_WITH_RESET_FN(someSystemArrayItem_t, 10, someSystemArray, PG_RESERVED_FOR_TESTING_2, 0);


    typedef struct someProfileSpecificData_s {
        uint8_t uint8;
        uint16_t uint16;
        uint32_t uint32;
    } PG_PACKED someProfileSpecificData_t;

    PG_DECLARE_PROFILE(someProfileSpecificData_t, someProfileSpecificData);
    PG_REGISTER_PROFILE_WITH_RESET_FN(someProfileSpecificData_t, someProfileSpecificData, PG_RESERVED_FOR_TESTING_3, 0);
    void pgResetFn_someProfileSpecificData(someProfileSpecificData_t *instance)
    {
        instance->uint8 = 0xFF;
        instance->uint16 = 0xAAAA;
        instance->uint32 = 0x88888888;
    }

    PG_DECLARE_PROFILE(someProfileSpecificData_t, someProfileSpecificData_TR);
    PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(someProfileSpecificData_t, someProfileSpecificData_TR, 0, 0);
    PG_RESET_TEMPLATE(someProfileSpecificData_t, someProfileSpecificData_TR,
        .uint8 = 0xFF,
        .uint16 = 0xAAAA,
        .uint32 = 0x88888888,
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


//#define DEBUG_PG_INSTANCES
TEST(configTest, fixUnusedWarning)
{
    someSystemArray_arr();
}

TEST(configTest, resetEEPROM)
{
    // when

    someSystemData_NR()->uint32 = 0x55555555;  // Set to non-zero
    someSystemData_NR()->uint16 = 0x5555;
    someSystemData_NR()->uint8  = 0x55;

    resetEEPROM();

    // then

    //
    // check system settings
    //

    EXPECT_EQ(0xFFFFFFFF, someSystemData()->uint32);
    EXPECT_EQ(0xAAAA, someSystemData()->uint16);
    EXPECT_EQ(0x88, someSystemData()->uint8);

    EXPECT_EQ(0, someSystemData_NR()->uint32);
    EXPECT_EQ(0, someSystemData_NR()->uint16);
    EXPECT_EQ(0, someSystemData_NR()->uint8);

    EXPECT_EQ(0xFFFFFFFF, someSystemData_TR()->uint32);
    EXPECT_EQ(0xAAAA, someSystemData_TR()->uint16);
    EXPECT_EQ(0x88, someSystemData_TR()->uint8);

    for (int i = 0; i < 10; i++) {
        someSystemArrayItem_t *item = someSystemArray(i);

#ifdef DEBUG_PG_INSTANCES
        printf("iteration: %d\n", i);
#endif
        EXPECT_EQ(0xFFFF, item->uint16);
        EXPECT_EQ(0xAAAAAAAA, item->uint32);
        EXPECT_EQ(0x88, item->uint8);
    }


    //
    // check each profile
    //

    EXPECT_EQ(0xFF, someProfileSpecificData()->uint8);
    EXPECT_EQ(0xAAAA, someProfileSpecificData()->uint16);
    EXPECT_EQ(0x88888888, someProfileSpecificData()->uint32);

    EXPECT_EQ(0xFF, someProfileSpecificData_TR()->uint8);
    EXPECT_EQ(0xAAAA, someProfileSpecificData_TR()->uint16);
    EXPECT_EQ(0x88888888, someProfileSpecificData_TR()->uint32);

    // and

    changeProfile(1);
    EXPECT_EQ(0xFF, someProfileSpecificData()->uint8);
    EXPECT_EQ(0xAAAA, someProfileSpecificData()->uint16);
    EXPECT_EQ(0x88888888, someProfileSpecificData()->uint32);

    EXPECT_EQ(0xFF, someProfileSpecificData_TR()->uint8);
    EXPECT_EQ(0xAAAA, someProfileSpecificData_TR()->uint16);
    EXPECT_EQ(0x88888888, someProfileSpecificData_TR()->uint32);

    // and

    changeProfile(2);
    EXPECT_EQ(0xFF, someProfileSpecificData()->uint8);
    EXPECT_EQ(0xAAAA, someProfileSpecificData()->uint16);
    EXPECT_EQ(0x88888888, someProfileSpecificData()->uint32);

    EXPECT_EQ(0xFF, someProfileSpecificData_TR()->uint8);
    EXPECT_EQ(0xAAAA, someProfileSpecificData_TR()->uint16);
    EXPECT_EQ(0x88888888, someProfileSpecificData_TR()->uint32);
}

TEST(configTest, modify)
{
    resetEEPROM();

    imuConfig()->looptime = 123;
    writeEEPROM();

    EXPECT_EQ(123, imuConfig()->looptime);

    // overwrite the values with something else before loading
    imuConfig()->looptime = 456;

    readEEPROM();
    EXPECT_EQ(123, imuConfig()->looptime);
}

TEST(configTest, modifyProfiles)
{
    resetEEPROM();

    // default profile is 0

    someProfileSpecificData()->uint32 = 1;
    someProfileSpecificData()->uint16 = 2;
    someProfileSpecificData()->uint8  = 3;
    changeProfile(1); // changing saves the EEPROM

    someProfileSpecificData()->uint32 = 4;
    someProfileSpecificData()->uint16 = 5;
    someProfileSpecificData()->uint8  = 6;
    changeProfile(2); // changing saves the EEPROM

    someProfileSpecificData()->uint32 = 7;
    someProfileSpecificData()->uint16 = 8;
    someProfileSpecificData()->uint8  = 9;
    changeProfile(0);  // changing saves the EEPROM

    EXPECT_EQ(1, someProfileSpecificData()->uint32);
    EXPECT_EQ(2, someProfileSpecificData()->uint16);
    EXPECT_EQ(3, someProfileSpecificData()->uint8);

    changeProfile(1); // changing saves the EEPROM
    EXPECT_EQ(4, someProfileSpecificData()->uint32);
    EXPECT_EQ(5, someProfileSpecificData()->uint16);
    EXPECT_EQ(6, someProfileSpecificData()->uint8);

    changeProfile(2); // changing saves the EEPROM
    EXPECT_EQ(7, someProfileSpecificData()->uint32);
    EXPECT_EQ(8, someProfileSpecificData()->uint16);
    EXPECT_EQ(9, someProfileSpecificData()->uint8);
}


/*
 * Test that the config items whose default values are zero are indeed set to zero by resetConf().
 */

// FIXME delete this test and any stubs when migration to parameter groups is complete as it will be redundant.

TEST(ConfigUnittest, TestResetConfigZeroValues)
{
    resetEEPROM();

    EXPECT_EQ(0, profileSelection()->current_profile_index); // default profile
    EXPECT_EQ(0, imuConfig()->dcm_ki); // 0.003 * 10000

    EXPECT_EQ(0, sensorTrims()->accZero.values.pitch);
    EXPECT_EQ(0, sensorTrims()->accZero.values.roll);
    EXPECT_EQ(0, sensorTrims()->accZero.values.yaw);

    EXPECT_EQ(ALIGN_DEFAULT, sensorAlignmentConfig()->gyro_align);
    EXPECT_EQ(ALIGN_DEFAULT, sensorAlignmentConfig()->acc_align);
    EXPECT_EQ(ALIGN_DEFAULT, sensorAlignmentConfig()->mag_align);

    EXPECT_EQ(0, boardAlignment()->rollDegrees);
    EXPECT_EQ(0, boardAlignment()->pitchDegrees);
    EXPECT_EQ(0, boardAlignment()->yawDegrees);

    EXPECT_EQ(ACC_DEFAULT, sensorSelectionConfig()->acc_hardware);   // default/autodetect
    EXPECT_EQ(MAG_DEFAULT, sensorSelectionConfig()->mag_hardware);   // default/autodetect
    EXPECT_EQ(BARO_DEFAULT, sensorSelectionConfig()->baro_hardware); // default/autodetect

    EXPECT_EQ(0, batteryConfig()->currentMeterOffset);
    EXPECT_EQ(0, batteryConfig()->batteryCapacity);

    EXPECT_EQ(0, telemetryConfig()->telemetry_inversion);
    EXPECT_EQ(0, telemetryConfig()->telemetry_switch);

    EXPECT_EQ(0, frskyTelemetryConfig()->gpsNoFixLatitude);
    EXPECT_EQ(0, frskyTelemetryConfig()->gpsNoFixLongitude);
    EXPECT_EQ(FRSKY_FORMAT_DMS, frskyTelemetryConfig()->frsky_coordinate_format);
    EXPECT_EQ(FRSKY_UNIT_METRICS, frskyTelemetryConfig()->frsky_unit);
    EXPECT_EQ(0, frskyTelemetryConfig()->frsky_vfas_precision);

    EXPECT_EQ(0, rxConfig()->serialrx_provider);
    EXPECT_EQ(0, rxConfig()->spektrum_sat_bind);

    EXPECT_EQ(0, rxConfig()->rssi_channel);
    EXPECT_EQ(0, rxConfig()->rssi_ppm_invert);
    EXPECT_EQ(0, rxConfig()->rcSmoothing);

    EXPECT_EQ(INPUT_FILTERING_DISABLED, pwmRxConfig()->inputFilteringMode);

    EXPECT_EQ(0, armingConfig()->retarded_arm);

    EXPECT_EQ(0, mixerConfig()->servo_lowpass_enable);

    EXPECT_EQ(GPS_NMEA, gpsConfig()->provider);
    EXPECT_EQ(SBAS_AUTO, gpsConfig()->sbasMode);
    EXPECT_EQ(GPS_AUTOBAUD_OFF, gpsConfig()->autoBaud);

    EXPECT_EQ(0, systemConfig()->emf_avoidance);

    EXPECT_EQ(0, controlRateProfiles(0)->thrExpo8);
    EXPECT_EQ(0, controlRateProfiles(0)->dynThrPID);
    EXPECT_EQ(0, controlRateProfiles(0)->rcYawExpo8);
    for (uint8_t axis = 0; axis < FD_INDEX_COUNT; axis++) {
        EXPECT_EQ(0, controlRateProfiles(0)->rates[axis]);
    }

    EXPECT_EQ(0, failsafeConfig()->failsafe_kill_switch); // default failsafe switch action is identical to rc link loss
    EXPECT_EQ(0, failsafeConfig()->failsafe_procedure);   // default full failsafe procedure is 0: auto-landing

    // custom mixer. clear by defaults.
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        EXPECT_EQ(0.0f, customMotorMixer(i)->throttle);
    }
}

// STUBS
extern "C" {

void applyDefaultLedStripConfig(void) {}
void applyDefaultColors(void) {}
void beeperConfirmationBeeps(uint8_t) {}
void StopPwmAllMotors(void) {}
void useRxConfig(rxConfig_t *) {}
void useRcControlsConfig(modeActivationCondition_t *) {}
void useFailsafeConfig(void) {}
void suspendRxSignal(void) {}
void setAccelerationTrims(flightDynamicsTrims_t *) {}
void resumeRxSignal(void) {}
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfiguration_t *) {}
void resetAdjustmentStates(void) {}
void pidSetController(pidControllerType_e) {}
void parseRcChannels(const char *, rxConfig_t *) {}
#ifdef USE_SERVOS
void mixerUseConfigs(servoParam_t *) {}
#else
void mixerUseConfigs(void) {}
#endif
bool isSerialConfigValid(serialConfig_t *) {return true;}
void imuConfigure(imuRuntimeConfig_t *, accDeadband_t *,float ,uint16_t) {}
void gpsUseProfile(gpsProfile_t *) {}
void gpsUsePIDs(pidProfile_t *) {}
void generateYawCurve(controlRateConfig_t *) {}
void generatePitchRollCurve(controlRateConfig_t *) {}
void generateThrottleCurve(controlRateConfig_t *) {}
void delay(uint32_t) {}

void setControlRateProfile(uint8_t) {}
void resetControlRateConfig(controlRateConfig_t *) {}
void configureRateProfileSelection(uint8_t, uint8_t) {}
void activateControlRateConfig() {}

void recalculateMagneticDeclination(void) {}

void pgReset_serialConfig(serialConfig_t *) {}

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

