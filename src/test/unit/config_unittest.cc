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
#include <string.h>

extern "C" {
    #include <platform.h>
    #include "config/config.h"
    #include "common/axis.h"
    #include "common/color.h"
    #include "common/maths.h"
    #include "drivers/sensor.h"
    #include "drivers/timer.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"
    #include "drivers/pwm_rx.h"
    #include "drivers/serial.h"
    #include "io/escservo.h"
    #include "io/gimbal.h"
    #include "io/gps.h"
    #include "io/ledstrip.h"
    #include "io/rc_controls.h"
    #include "io/serial.h"
    #include "telemetry/telemetry.h"
    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"
    #include "sensors/boardalignment.h"
    #include "sensors/barometer.h"
    #include "sensors/battery.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"
    #include "flight/pid.h"
    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/navigation.h"
    #include "rx/rx.h"
    #include "config/config_profile.h"
    #include "config/config_master.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    void resetConf(void);
    extern master_t masterConfig;
}

/*
 * Test that the config items whose default values are zero are indeed set to zero by resetConf().
 */
TEST(ConfigUnittest, TestResetConfigZeroValues)
{
    memset(&masterConfig, 0xa5, sizeof(master_t));
    resetConf();

    EXPECT_EQ(0, masterConfig.current_profile_index); // default profile
    EXPECT_EQ(0, masterConfig.dcm_ki); // 0.003 * 10000

//resetAccelerometerTrims(&masterConfig.accZero);
    EXPECT_EQ(0, masterConfig.accZero.values.pitch);
    EXPECT_EQ(0, masterConfig.accZero.values.roll);
    EXPECT_EQ(0, masterConfig.accZero.values.yaw);

//resetSensorAlignment(&masterConfig.sensorAlignmentConfig);
    EXPECT_EQ(ALIGN_DEFAULT, masterConfig.sensorAlignmentConfig.gyro_align);
    EXPECT_EQ(ALIGN_DEFAULT, masterConfig.sensorAlignmentConfig.acc_align);
    EXPECT_EQ(ALIGN_DEFAULT, masterConfig.sensorAlignmentConfig.mag_align);

    EXPECT_EQ(0, masterConfig.boardAlignment.rollDegrees);
    EXPECT_EQ(0, masterConfig.boardAlignment.pitchDegrees);
    EXPECT_EQ(0, masterConfig.boardAlignment.yawDegrees);

    EXPECT_EQ(ACC_DEFAULT, masterConfig.acc_hardware);   // default/autodetect
    EXPECT_EQ(MAG_DEFAULT, masterConfig.mag_hardware);   // default/autodetect
    EXPECT_EQ(BARO_DEFAULT, masterConfig.baro_hardware); // default/autodetect

    EXPECT_EQ(0, masterConfig.batteryConfig.currentMeterOffset);
    EXPECT_EQ(0, masterConfig.batteryConfig.batteryCapacity);

    EXPECT_EQ(0, masterConfig.telemetryConfig.telemetry_inversion);
    EXPECT_EQ(0, masterConfig.telemetryConfig.telemetry_switch);
    EXPECT_EQ(0, masterConfig.telemetryConfig.gpsNoFixLatitude);
    EXPECT_EQ(0, masterConfig.telemetryConfig.gpsNoFixLongitude);
    EXPECT_EQ(FRSKY_FORMAT_DMS, masterConfig.telemetryConfig.frsky_coordinate_format);
    EXPECT_EQ(FRSKY_UNIT_METRICS, masterConfig.telemetryConfig.frsky_unit);
    EXPECT_EQ(0, masterConfig.telemetryConfig.frsky_vfas_precision);

    EXPECT_EQ(0, masterConfig.rxConfig.serialrx_provider);
    EXPECT_EQ(0, masterConfig.rxConfig.spektrum_sat_bind);

    EXPECT_EQ(0, masterConfig.rxConfig.rssi_channel);
    EXPECT_EQ(0, masterConfig.rxConfig.rssi_ppm_invert);
    EXPECT_EQ(0, masterConfig.rxConfig.rcSmoothing);

    EXPECT_EQ(INPUT_FILTERING_DISABLED, masterConfig.inputFilteringMode);

    EXPECT_EQ(0, masterConfig.retarded_arm);

//    resetMixerConfig(&masterConfig.mixerConfig);
    EXPECT_EQ(0, masterConfig.mixerConfig.servo_lowpass_enable);

    EXPECT_EQ(GPS_NMEA, masterConfig.gpsConfig.provider);
    EXPECT_EQ(SBAS_AUTO, masterConfig.gpsConfig.sbasMode);
    EXPECT_EQ(GPS_AUTOBAUD_OFF, masterConfig.gpsConfig.autoBaud);

    EXPECT_EQ(0, masterConfig.emf_avoidance);

// resetControlRateConfig(&masterConfig.controlRateProfiles[0]);
    EXPECT_EQ(0, masterConfig.controlRateProfiles[0].thrExpo8);
    EXPECT_EQ(0, masterConfig.controlRateProfiles[0].dynThrPID);
    EXPECT_EQ(0, masterConfig.controlRateProfiles[0].rcYawExpo8);
    for (uint8_t axis = 0; axis < FD_INDEX_COUNT; axis++) {
        EXPECT_EQ(0, masterConfig.controlRateProfiles[0].rates[axis]);
    }

    EXPECT_EQ(0, masterConfig.failsafeConfig.failsafe_kill_switch); // default failsafe switch action is identical to rc link loss
    EXPECT_EQ(0, masterConfig.failsafeConfig.failsafe_procedure);   // default full failsafe procedure is 0: auto-landing

    // custom mixer. clear by defaults.
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        EXPECT_EQ(0.0f, masterConfig.customMotorMixer[i].throttle);
    }
}

// STUBS
extern "C" {

void applyDefaultLedStripConfig(ledConfig_t *) {}
void applyDefaultColors(hsvColor_t *, uint8_t) {}
void beeperConfirmationBeeps(uint8_t) {}
void StopPwmAllMotors(void) {}
void useRxConfig(rxConfig_t *) {}
void useRcControlsConfig(modeActivationCondition_t *, escAndServoConfig_t *, pidProfile_t *) {}
void useGyroConfig(gyroConfig_t *, float) {}
void useFailsafeConfig(failsafeConfig_t *) {}
void useBarometerConfig(barometerConfig_t *) {}
void telemetryUseConfig(telemetryConfig_t *) {}
void suspendRxSignal(void) {}
void setAccelerationTrims(flightDynamicsTrims_t *) {}
void resumeRxSignal(void) {}
void resetRollAndPitchTrims(rollAndPitchTrims_t *) {}
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfiguration_t *) {}
void resetAdjustmentStates(void) {}
void pidSetController(pidControllerType_e) {}
void parseRcChannels(const char *, rxConfig_t *) {}
#ifdef USE_SERVOS
void mixerUseConfigs(servoParam_t *, gimbalConfig_t *, flight3DConfig_t *, escAndServoConfig_t *, mixerConfig_t *, airplaneConfig_t *, rxConfig_t *) {}
#else
void mixerUseConfigs(flight3DConfig_t *, escAndServoConfig_t *, mixerConfig_t *, airplaneConfig_t *, rxConfig_t *) {}
#endif
bool isSerialConfigValid(serialConfig_t *) {return true;}
void imuConfigure(imuRuntimeConfig_t *, pidProfile_t *,accDeadband_t *,float ,uint16_t) {}
void gpsUseProfile(gpsProfile_t *) {}
void gpsUsePIDs(pidProfile_t *) {}
void generateYawCurve(controlRateConfig_t *) {}
void generatePitchRollCurve(controlRateConfig_t *) {}
void generateThrottleCurve(controlRateConfig_t *) {}
void delay(uint32_t) {}
void configureAltitudeHold(pidProfile_t *, barometerConfig_t *, rcControlsConfig_t *, escAndServoConfig_t *) {}
void failureMode(uint8_t) {}

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

