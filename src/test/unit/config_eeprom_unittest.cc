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
    #include "config/parameter_group.h"
    #include "platform.h"

    failsafeConfig_t failsafeConfig;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

const pgRegistry_t __pg_registry[] =
{
    {
        .base = &masterConfig,
        .size = sizeof(masterConfig),
        .pgn = 0,
        .format = 0
    },
    {
        .base = nullptr,
        .size = 0,
        .pgn = 0,
        .format = 0
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

    masterConfig.looptime = 456;
    readEEPROM();
    EXPECT_EQ(123, masterConfig.looptime);
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

