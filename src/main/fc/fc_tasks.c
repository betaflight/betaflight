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

#include "cms/cms.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/stack_check.h"

#include "fc/cli.h"
#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/fc_msp.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "navigation/navigation.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/pwmdriver_i2c.h"
#include "io/serial.h"
#include "io/rcsplit.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"
#include "rx/eleres.h"
#include "rx/rx_spi.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"

#include "telemetry/telemetry.h"

#include "config/feature.h"

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)

void taskHandleSerial(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
#ifdef USE_CLI
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }
#endif
    mspSerialProcess(ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand);
}

void taskUpdateBattery(timeUs_t currentTimeUs)
{
#ifdef USE_ADC
    static timeUs_t vbatLastServiced = 0;
    if (feature(FEATURE_VBAT)) {
        if (cmpTimeUs(currentTimeUs, vbatLastServiced) >= VBATINTERVAL) {
            timeUs_t vbatTimeDelta = currentTimeUs - vbatLastServiced;
            vbatLastServiced = currentTimeUs;
            batteryUpdate(vbatTimeDelta);
        }
    }
#endif

    static timeUs_t ibatLastServiced = 0;
    if (feature(FEATURE_CURRENT_METER)) {
        timeUs_t ibatTimeSinceLastServiced = cmpTimeUs(currentTimeUs, ibatLastServiced);

        if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
            ibatLastServiced = currentTimeUs;
            currentMeterUpdate(ibatTimeSinceLastServiced);
        }
    }
}

#ifdef GPS
void taskProcessGPS(timeUs_t currentTimeUs)
{
    // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
    // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
    // change this based on available hardware
    if (feature(FEATURE_GPS)) {
        gpsThread();
    }

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTimeUs);
    }
}
#endif

#ifdef MAG
void taskUpdateCompass(timeUs_t currentTimeUs)
{
    if (sensors(SENSOR_MAG)) {
        compassUpdate(currentTimeUs);
    }
}
#endif

#ifdef BARO
void taskUpdateBaro(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (sensors(SENSOR_BARO)) {
        const uint32_t newDeadline = baroUpdate();
        if (newDeadline != 0) {
            rescheduleTask(TASK_SELF, newDeadline);
        }
    }

    updatePositionEstimator_BaroTopic(currentTimeUs);
}
#endif

#ifdef PITOT
void taskUpdatePitot(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (sensors(SENSOR_PITOT)) {
        pitotUpdate();
    }
}
#endif

#ifdef USE_RANGEFINDER
void taskUpdateRangefinder(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!sensors(SENSOR_RANGEFINDER))
        return;

    // Update and adjust task to update at required rate
    const uint32_t newDeadline = rangefinderUpdate();
    if (newDeadline != 0) {
        rescheduleTask(TASK_SELF, newDeadline);
    }

    /*
     * Process raw rangefinder readout
     */
    if (rangefinderProcess(calculateCosTiltAngle())) {
        updatePositionEstimator_SurfaceTopic(currentTimeUs, rangefinderGetLatestAltitude());
    }
}
#endif

#ifdef USE_DASHBOARD
void taskDashboardUpdate(timeUs_t currentTimeUs)
{
    if (feature(FEATURE_DASHBOARD)) {
        dashboardUpdate(currentTimeUs);
    }
}
#endif

#ifdef TELEMETRY
void taskTelemetry(timeUs_t currentTimeUs)
{
    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(currentTimeUs);
    }
}
#endif

#ifdef LED_STRIP
void taskLedStrip(timeUs_t currentTimeUs)
{
    if (feature(FEATURE_LED_STRIP)) {
        ledStripUpdate(currentTimeUs);
    }
}
#endif

#ifdef USE_PMW_SERVO_DRIVER
void taskSyncPwmDriver(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (feature(FEATURE_PWM_SERVO_DRIVER)) {
        pwmDriverSync();
    }
}
#endif

#ifdef ASYNC_GYRO_PROCESSING
void taskAttitude(timeUs_t currentTimeUs)
{
    imuUpdateAttitude(currentTimeUs);
}

void taskAcc(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    imuUpdateAccelerometer();
}
#endif

#ifdef OSD
void taskUpdateOsd(timeUs_t currentTimeUs)
{
    if (feature(FEATURE_OSD)) {
        osdUpdate(currentTimeUs);
    }
}
#endif

void fcTasksInit(void)
{
    schedulerInit();

#ifdef ASYNC_GYRO_PROCESSING
    rescheduleTask(TASK_PID, getPidUpdateRate());
    setTaskEnabled(TASK_PID, true);

    if (getAsyncMode() != ASYNC_MODE_NONE) {
        rescheduleTask(TASK_GYRO, getGyroUpdateRate());
        setTaskEnabled(TASK_GYRO, true);
    }

    if (getAsyncMode() == ASYNC_MODE_ALL && sensors(SENSOR_ACC)) {
        rescheduleTask(TASK_ACC, getAccUpdateRate());
        setTaskEnabled(TASK_ACC, true);

        rescheduleTask(TASK_ATTI, getAttitudeUpdateRate());
        setTaskEnabled(TASK_ATTI, true);
    }

#else
    rescheduleTask(TASK_GYROPID, getGyroUpdateRate());
    setTaskEnabled(TASK_GYROPID, true);
#endif

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
#if defined(MPU6500_SPI_INSTANCE) && defined(USE_MAG_AK8963)
    // fixme temporary solution for AK6983 via slave I2C on MPU9250
    rescheduleTask(TASK_COMPASS, TASK_PERIOD_HZ(40));
#endif
#endif
#ifdef BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif
#ifdef PITOT
    setTaskEnabled(TASK_PITOT, sensors(SENSOR_PITOT));
#endif
#ifdef USE_RANGEFINDER
    setTaskEnabled(TASK_RANGEFINDER, sensors(SENSOR_RANGEFINDER));
#endif
#ifdef USE_DASHBOARD
    setTaskEnabled(TASK_DASHBOARD, feature(FEATURE_DASHBOARD));
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
#ifdef OSD
    setTaskEnabled(TASK_OSD, feature(FEATURE_OSD));
#endif
#ifdef CMS
#ifdef USE_MSP_DISPLAYPORT
    setTaskEnabled(TASK_CMS, true);
#else
    setTaskEnabled(TASK_CMS, feature(FEATURE_OSD) || feature(FEATURE_DASHBOARD));
#endif
#endif
}

cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = TASK_PERIOD_HZ(10),              // run every 100 ms, 10Hz
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    #ifdef ASYNC_GYRO_PROCESSING
        [TASK_PID] = {
            .taskName = "PID",
            .taskFunc = taskMainPidLoop,
            .desiredPeriod = TASK_PERIOD_HZ(500), // Run at 500Hz
            .staticPriority = TASK_PRIORITY_HIGH,
        },

        [TASK_GYRO] = {
            .taskName = "GYRO",
            .taskFunc = taskGyro,
            .desiredPeriod = TASK_PERIOD_HZ(1000), //Run at 1000Hz
            .staticPriority = TASK_PRIORITY_REALTIME,
        },

        [TASK_ACC] = {
            .taskName = "ACC",
            .taskFunc = taskAcc,
            .desiredPeriod = TASK_PERIOD_HZ(520), //520Hz is ACC bandwidth (260Hz) * 2
            .staticPriority = TASK_PRIORITY_HIGH,
        },

        [TASK_ATTI] = {
            .taskName = "ATTITUDE",
            .taskFunc = taskAttitude,
            .desiredPeriod = TASK_PERIOD_HZ(60), //With acc LPF at 15Hz 60Hz attitude refresh should be enough
            .staticPriority = TASK_PRIORITY_HIGH,
        },

    #else

        /*
         * Legacy synchronous PID/gyro/acc/atti mode
         * for 64kB targets and other smaller targets
         */

        [TASK_GYROPID] = {
            .taskName = "GYRO/PID",
            .taskFunc = taskMainPidLoop,
            .desiredPeriod = TASK_PERIOD_US(1000),
            .staticPriority = TASK_PRIORITY_REALTIME,
        },
    #endif

    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .staticPriority = TASK_PRIORITY_LOW,
    },

#ifdef BEEPER
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = beeperUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),     // 100 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

    [TASK_BATTERY] = {
        .taskName = "BATTERY",
        .taskFunc = taskUpdateBattery,
        .desiredPeriod = TASK_PERIOD_HZ(50),      // 50 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = taskUpdateRxCheck,
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = TASK_PERIOD_HZ(50),      // If event-based scheduling doesn't work, fallback to periodic scheduling
        .staticPriority = TASK_PRIORITY_HIGH,
    },

#ifdef GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = taskProcessGPS,
        .desiredPeriod = TASK_PERIOD_HZ(25),      // GPS usually don't go raster than 10Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = TASK_PERIOD_HZ(10),      // Compass is updated at 10 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = TASK_PERIOD_HZ(20),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef PITOT
    [TASK_PITOT] = {
        .taskName = "PITOT",
        .taskFunc = taskUpdatePitot,
        .desiredPeriod = TASK_PERIOD_HZ(10),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_RANGEFINDER
    [TASK_RANGEFINDER] = {
        .taskName = "RANGEFINDER",
        .taskFunc = taskUpdateRangefinder,
        .desiredPeriod = TASK_PERIOD_MS(70),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_DASHBOARD
    [TASK_DASHBOARD] = {
        .taskName = "DASHBOARD",
        .taskFunc = taskDashboardUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = TASK_PERIOD_HZ(500),         // 500 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = taskLedStrip,
        .desiredPeriod = TASK_PERIOD_HZ(100),         // 100 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef USE_PMW_SERVO_DRIVER
    [TASK_PWMDRIVER] = {
        .taskName = "PWMDRIVER",
        .taskFunc = taskSyncPwmDriver,
        .desiredPeriod = TASK_PERIOD_HZ(200),         // 200 Hz
        .staticPriority = TASK_PRIORITY_HIGH,
    },
#endif

#ifdef STACK_CHECK
    [TASK_STACK_CHECK] = {
        .taskName = "STACKCHECK",
        .taskFunc = taskStackCheck,
        .desiredPeriod = TASK_PERIOD_HZ(10),          // 10 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef OSD
    [TASK_OSD] = {
        .taskName = "OSD",
        .taskFunc = taskUpdateOsd,
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef CMS
    [TASK_CMS] = {
        .taskName = "CMS",
        .taskFunc = cmsHandler,
        .desiredPeriod = TASK_PERIOD_HZ(50),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_RCSPLIT
    [TASK_RCSPLIT] = {
        .taskName = "RCSPLIT",
        .taskFunc = rcSplitProcess,
        .desiredPeriod = TASK_PERIOD_HZ(10),        // 10 Hz, 100ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif
};
