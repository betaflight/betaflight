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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/io_pca9685.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"

#include "fc/fc_tasks.h"
#include "fc/mw.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/motors.h"
#include "io/servos.h"
#include "io/pwmdriver_i2c.h"
#include "io/serial.h"
#include "io/serial_cli.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/hil.h"
#include "flight/failsafe.h"
#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/feature.h"
#include "config/config_profile.h"
#include "config/config_master.h"

extern uint32_t currentTime;

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)

cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = 1000000 / 10,              // run every 100 ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    [TASK_GYROPID] = {
        .taskName = "GYRO/PID",
        .taskFunc = taskMainPidLoopChecker,
        .desiredPeriod = 1000,
        .staticPriority = TASK_PRIORITY_REALTIME,
    },

    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = 1000000 / 100,     // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .staticPriority = TASK_PRIORITY_LOW,
    },

    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = taskUpdateBeeper,
        .desiredPeriod = 1000000 / 100,     // 100 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_BATTERY] = {
        .taskName = "BATTERY",
        .taskFunc = taskUpdateBattery,
        .desiredPeriod = 1000000 / 50,      // 50 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = taskUpdateRxCheck,
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = 1000000 / 50,      // If event-based scheduling doesn't work, fallback to periodic scheduling
        .staticPriority = TASK_PRIORITY_HIGH,
    },

#ifdef GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = taskProcessGPS,
        .desiredPeriod = 1000000 / 25,      // GPS usually don't go raster than 10Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = 1000000 / 10,      // Compass is updated at 10 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = 1000000 / 20,
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef SONAR
    [TASK_SONAR] = {
        .taskName = "SONAR",
        .taskFunc = taskUpdateSonar,
        .desiredPeriod = 70000,                 // every 70 ms, approximately 14 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef DISPLAY
    [TASK_DISPLAY] = {
        .taskName = "DISPLAY",
        .taskFunc = taskUpdateDisplay,
        .desiredPeriod = 1000000 / 10,
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = 1000000 / 250,         // 250 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = taskLedStrip,
        .desiredPeriod = 1000000 / 100,         // 100 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef USE_PMW_SERVO_DRIVER
    [TASK_PWMDRIVER] = {
        .taskName = "PWMDRIVER",
        .taskFunc = taskSyncPwmDriver,
        .desiredPeriod = 1000000 / 200,         // 200 Hz
        .staticPriority = TASK_PRIORITY_HIGH,
    },
#endif

#ifdef STACK_CHECK
    [TASK_STACK_CHECK] = {
        .taskName = "STACKCHECK",
        .taskFunc = taskStackCheck,
        .desiredPeriod = 1000000 / 10,          // 10 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif
};

void taskHandleSerial(void)
{
#ifdef USE_CLI
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }
#endif
    mspSerialProcess(ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA);
}

void taskUpdateBeeper(void)
{
    beeperUpdate();          //call periodic beeper handler
}

void taskUpdateBattery(void)
{
    static uint32_t vbatLastServiced = 0;
    static uint32_t ibatLastServiced = 0;

    if (feature(FEATURE_VBAT)) {
        if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
            uint32_t vbatTimeDelta = currentTime - vbatLastServiced;
            vbatLastServiced = currentTime;
            updateBattery(vbatTimeDelta);
        }
    }

    if (feature(FEATURE_CURRENT_METER)) {
        int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

        if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
            ibatLastServiced = currentTime;
            updateCurrentMeter(ibatTimeSinceLastServiced, &masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
        }
    }
}

#ifdef GPS
void taskProcessGPS(void)
{
    // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
    // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
    // change this based on available hardware
    if (feature(FEATURE_GPS)) {
        gpsThread();
    }

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTime);
    }
}
#endif

#ifdef MAG
void taskUpdateCompass(void)
{
    if (sensors(SENSOR_MAG)) {
        updateCompass(&masterConfig.magZero);
    }
}
#endif

#ifdef BARO
void taskUpdateBaro(void)
{
    if (sensors(SENSOR_BARO)) {
        const uint32_t newDeadline = baroUpdate();
        if (newDeadline != 0) {
            rescheduleTask(TASK_SELF, newDeadline);
        }
    }

    //updatePositionEstimator_BaroTopic(currentTime);
}
#endif

#ifdef SONAR
void taskUpdateSonar(void)
{
    if (sensors(SENSOR_SONAR)) {
        rangefinderUpdate();
    }

    //updatePositionEstimator_SonarTopic(currentTime);
}
#endif

#ifdef DISPLAY
void taskUpdateDisplay(void)
{
    if (feature(FEATURE_DISPLAY)) {
        updateDisplay();
    }
}
#endif

#ifdef TELEMETRY
void taskTelemetry(void)
{
    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
    }
}
#endif

#ifdef LED_STRIP
void taskLedStrip(void)
{
    if (feature(FEATURE_LED_STRIP)) {
        updateLedStrip();
    }
}
#endif

#ifdef USE_PMW_SERVO_DRIVER
void taskSyncPwmDriver(void) {

    if (feature(FEATURE_PWM_SERVO_DRIVER)) {
        pwmDriverSync();
    }
}
#endif

void fcTasksInit(void)
{
    schedulerInit();

    rescheduleTask(TASK_GYROPID, gyro.targetLooptime);
    setTaskEnabled(TASK_GYROPID, true);

    setTaskEnabled(TASK_SERIAL, true);
#ifdef BEEPER
    setTaskEnabled(TASK_BEEPER, true);
#endif
    setTaskEnabled(TASK_BATTERY, feature(FEATURE_VBAT) || feature(FEATURE_CURRENT_METER));
    setTaskEnabled(TASK_RX, true);
#ifdef GPS
    setTaskEnabled(TASK_GPS, feature(FEATURE_GPS));
#endif
#ifdef MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#if (defined(MPU6500_SPI_INSTANCE) || defined(MPU9250_SPI_INSTANCE)) && defined(USE_MAG_AK8963)
    // fixme temporary solution for AK6983 via slave I2C on MPU9250
    rescheduleTask(TASK_COMPASS, 1000000 / 40);
#endif
#endif
#ifdef BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif
#ifdef SONAR
    setTaskEnabled(TASK_SONAR, sensors(SENSOR_SONAR));
#endif
#ifdef DISPLAY
    setTaskEnabled(TASK_DISPLAY, feature(FEATURE_DISPLAY));
#endif
#ifdef TELEMETRY
    setTaskEnabled(TASK_TELEMETRY, feature(FEATURE_TELEMETRY));
#endif
#ifdef LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, feature(FEATURE_LED_STRIP));
#endif
#ifdef STACK_CHECK
    setTaskEnabled(TASK_STACK_CHECK, true);
#endif

#ifdef USE_PMW_SERVO_DRIVER
    setTaskEnabled(TASK_PWMDRIVER, feature(FEATURE_PWM_SERVO_DRIVER));
#endif
}
