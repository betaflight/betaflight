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

#include <platform.h>

#include "cms/cms.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/serial.h"

#include "fc/config.h"
#include "fc/fc_msp.h"
#include "fc/fc_tasks.h"
#include "fc/mw.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/altitudehold.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/transponder_ir.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sonar.h"

#include "scheduler/scheduler.h"

#include "telemetry/telemetry.h"

#include "config/feature.h"
#include "config/config_profile.h"
#include "config/config_master.h"


/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)


static void taskUpdateAccelerometer(uint32_t currentTime)
{
    UNUSED(currentTime);

    imuUpdateAccelerometer(&masterConfig.accelerometerTrims);
}

static void taskHandleSerial(uint32_t currentTime)
{
    UNUSED(currentTime);
#ifdef USE_CLI
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }
#endif
    mspSerialProcess(ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand);
}

static void taskUpdateBattery(uint32_t currentTime)
{
#ifdef USE_ADC
    static uint32_t vbatLastServiced = 0;
    if (feature(FEATURE_VBAT)) {
        if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
            vbatLastServiced = currentTime;
            updateBattery();
        }
    }
#endif

    static uint32_t ibatLastServiced = 0;
    if (feature(FEATURE_CURRENT_METER)) {
        const int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

        if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
            ibatLastServiced = currentTime;
            updateCurrentMeter(ibatTimeSinceLastServiced, &masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
        }
    }
}

static void taskUpdateRxMain(uint32_t currentTime)
{
    processRx(currentTime);
    isRXDataNew = true;

#if !defined(BARO) && !defined(SONAR)
    // updateRcCommands sets rcCommand, which is needed by updateAltHoldState and updateSonarAltHoldState
    updateRcCommands();
#endif
    updateLEDs();

#ifdef BARO
    if (sensors(SENSOR_BARO)) {
        updateAltHoldState();
    }
#endif

#ifdef SONAR
    if (sensors(SENSOR_SONAR)) {
        updateSonarAltHoldState();
    }
#endif
}

#ifdef MAG
static void taskUpdateCompass(uint32_t currentTime)
{
    if (sensors(SENSOR_MAG)) {
        compassUpdate(currentTime, &masterConfig.magZero);
    }
}
#endif

#ifdef BARO
static void taskUpdateBaro(uint32_t currentTime)
{
    UNUSED(currentTime);

    if (sensors(SENSOR_BARO)) {
        const uint32_t newDeadline = baroUpdate();
        if (newDeadline != 0) {
            rescheduleTask(TASK_SELF, newDeadline);
        }
    }
}
#endif

#if defined(BARO) || defined(SONAR)
static void taskCalculateAltitude(uint32_t currentTime)
{
    if (false
#if defined(BARO)
        || (sensors(SENSOR_BARO) && isBaroReady())
#endif
#if defined(SONAR)
        || sensors(SENSOR_SONAR)
#endif
        ) {
        calculateEstimatedAltitude(currentTime);
    }}
#endif

#ifdef TELEMETRY
static void taskTelemetry(uint32_t currentTime)
{
    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(currentTime, &masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
    }
}
#endif

void fcTasksInit(void)
{
    schedulerInit();
    rescheduleTask(TASK_GYROPID, gyro.targetLooptime);
    setTaskEnabled(TASK_GYROPID, true);

    if (sensors(SENSOR_ACC)) {
        setTaskEnabled(TASK_ACCEL, true);
        rescheduleTask(TASK_ACCEL, accSamplingInterval);
    }

    setTaskEnabled(TASK_ATTITUDE, sensors(SENSOR_ACC));
    setTaskEnabled(TASK_SERIAL, true);
    setTaskEnabled(TASK_BATTERY, feature(FEATURE_VBAT) || feature(FEATURE_CURRENT_METER));
    setTaskEnabled(TASK_RX, true);

#ifdef BEEPER
    setTaskEnabled(TASK_BEEPER, true);
#endif
#ifdef GPS
    setTaskEnabled(TASK_GPS, feature(FEATURE_GPS));
#endif
#ifdef MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#if defined(USE_SPI) && defined(USE_MAG_AK8963)
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
#if defined(BARO) || defined(SONAR)
    setTaskEnabled(TASK_ALTITUDE, sensors(SENSOR_BARO) || sensors(SENSOR_SONAR));
#endif
#ifdef USE_DASHBOARD
    setTaskEnabled(TASK_DASHBOARD, feature(FEATURE_DASHBOARD));
#endif
#ifdef TELEMETRY
    setTaskEnabled(TASK_TELEMETRY, feature(FEATURE_TELEMETRY));
    // Reschedule telemetry to 500hz for Jeti Exbus
    if (feature(FEATURE_TELEMETRY) || masterConfig.rxConfig.serialrx_provider == SERIALRX_JETIEXBUS) {
        rescheduleTask(TASK_TELEMETRY, 2000);
    }
#endif
#ifdef LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, feature(FEATURE_LED_STRIP));
#endif
#ifdef TRANSPONDER
    setTaskEnabled(TASK_TRANSPONDER, feature(FEATURE_TRANSPONDER));
#endif
#ifdef OSD
    setTaskEnabled(TASK_OSD, feature(FEATURE_OSD));
#endif
#ifdef USE_BST
    setTaskEnabled(TASK_BST_MASTER_PROCESS, true);
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
        .desiredPeriod = 1000000 / 10,              // run every 100 ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    [TASK_GYROPID] = {
        .taskName = "PID",
        .subTaskName = "GYRO",
        .taskFunc = taskMainPidLoopCheck,
        .desiredPeriod = TASK_GYROPID_DESIRED_PERIOD,
        .staticPriority = TASK_PRIORITY_REALTIME,
    },

    [TASK_ACCEL] = {
        .taskName = "ACCEL",
        .taskFunc = taskUpdateAccelerometer,
        .desiredPeriod = 1000000 / 1000,    // every 1ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_ATTITUDE] = {
        .taskName = "ATTITUDE",
        .taskFunc = imuUpdateAttitude,
        .desiredPeriod = 1000000 / 100,
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = rxUpdateCheck,
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = 1000000 / 50,      // If event-based scheduling doesn't work, fallback to periodic scheduling
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
        .desiredPeriod = 1000000 / 100,     // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .staticPriority = TASK_PRIORITY_LOW,
    },

    [TASK_BATTERY] = {
        .taskName = "BATTERY",
        .taskFunc = taskUpdateBattery,
        .desiredPeriod = 1000000 / 50,      // 50 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

#ifdef BEEPER
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = beeperUpdate,
        .desiredPeriod = 1000000 / 100,     // 100 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = gpsUpdate,
        .desiredPeriod = 1000000 / 10,      // GPS usually don't go raster than 10Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = taskUpdateCompass,
        .desiredPeriod = 1000000 / 10,      // Compass is updated at 10 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = 1000000 / 20,
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef SONAR
    [TASK_SONAR] = {
        .taskName = "SONAR",
        .taskFunc = sonarUpdate,
        .desiredPeriod = 70000,             // 70ms required so that SONAR pulses do not interfer with each other
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#if defined(BARO) || defined(SONAR)
    [TASK_ALTITUDE] = {
        .taskName = "ALTITUDE",
        .taskFunc = taskCalculateAltitude,
        .desiredPeriod = 1000000 / 40,
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef TRANSPONDER
    [TASK_TRANSPONDER] = {
        .taskName = "TRANSPONDER",
        .taskFunc = transponderUpdate,
        .desiredPeriod = 1000000 / 250,         // 250 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_DASHBOARD
    [TASK_DASHBOARD] = {
        .taskName = "DASHBOARD",
        .taskFunc = dashboardUpdate,
        .desiredPeriod = 1000000 / 10,
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif
#ifdef OSD
    [TASK_OSD] = {
        .taskName = "OSD",
        .taskFunc = osdUpdate,
        .desiredPeriod = 1000000 / 60,          // 60 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif
#ifdef TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = 1000000 / 250,         // 250 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = ledStripUpdate,
        .desiredPeriod = 1000000 / 100,         // 100 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_BST
    [TASK_BST_MASTER_PROCESS] = {
        .taskName = "BST_MASTER_PROCESS",
        .taskFunc = taskBstMasterProcess,
        .desiredPeriod = 1000000 / 50,          // 50 Hz
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef CMS
    [TASK_CMS] = {
        .taskName = "CMS",
        .taskFunc = cmsHandler,
        .desiredPeriod = 1000000 / 60,          // 60 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif
};
