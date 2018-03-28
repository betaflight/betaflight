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

#include "build/debug.h"

#include "cms/cms.h"

#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/stack_check.h"
#include "drivers/transponder_ir.h"
#include "drivers/vtx_common.h"
#ifdef USB_CDC_HID
//TODO: Make it platform independent in the future
#include "vcpf4/usbd_cdc_vcp.h"
#include "usbd_hid_core.h"
//TODO: Nicer way to handle this...
#undef MIN
#endif

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/fc_dispatch.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "interface/cli.h"
#include "interface/msp.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/osd_slave.h"
#include "io/piniobox.h"
#include "io/serial.h"
#include "io/transponder_ir.h"
#include "io/vtx_tramp.h" // Will be gone
#include "io/rcdevice_cam.h"
#include "io/vtx.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

#include "scheduler/scheduler.h"

#include "telemetry/telemetry.h"

#ifdef USE_USB_CDC_HID
//TODO: Make it platform independent in the future
#include "vcpf4/usbd_cdc_vcp.h"
#include "usbd_hid_core.h"
#endif

#ifdef USE_BST
void taskBstMasterProcess(timeUs_t currentTimeUs);
#endif

bool taskSerialCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(currentDeltaTimeUs);

    return mspSerialWaiting();
}

static void taskHandleSerial(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
#ifdef USE_CLI
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }
#endif
#ifndef OSD_SLAVE
    bool evaluateMspData = ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA;
#else
    bool evaluateMspData = osdSlaveIsLocked ?  MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA;;
#endif
    mspSerialProcess(evaluateMspData, mspFcProcessCommand, mspFcProcessReply);
}

void taskBatteryAlerts(timeUs_t currentTimeUs)
{
    if (!ARMING_FLAG(ARMED)) {
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }
    batteryUpdateStates(currentTimeUs);
    batteryUpdateAlarms();
}

#ifndef USE_OSD_SLAVE
static void taskUpdateAccelerometer(timeUs_t currentTimeUs)
{
    accUpdate(currentTimeUs, &accelerometerConfigMutable()->accelerometerTrims);
}

static void taskUpdateRxMain(timeUs_t currentTimeUs)
{
    if (!processRx(currentTimeUs)) {
        return;
    }
#ifdef USE_USB_CDC_HID
    if (!ARMING_FLAG(ARMED)) {
        int8_t report[8];
        for (int i = 0; i < 8; i++) {
	        	report[i] = scaleRange(constrain(rcData[i], 1000, 2000), 1000, 2000, -127, 127);
        }
        USBD_HID_SendReport(&USB_OTG_dev, (uint8_t*)report, sizeof(report));
    }
#endif

#if !defined(USE_ALT_HOLD)
    // updateRcCommands sets rcCommand, which is needed by updateAltHoldState and updateSonarAltHoldState
    updateRcCommands();
#endif
    updateArmingStatus();

#ifdef USE_ALT_HOLD
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        updateAltHoldState();
    }
#endif

#ifdef USE_RANGEFINDER
    if (sensors(SENSOR_RANGEFINDER)) {
        updateRangefinderAltHoldState();
    }
#endif
#endif // USE_ALT_HOLD
}
#endif

#ifdef USE_BARO
static void taskUpdateBaro(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (sensors(SENSOR_BARO)) {
        const uint32_t newDeadline = baroUpdate();
        if (newDeadline != 0) {
            rescheduleTask(TASK_SELF, newDeadline);
        }
    }
}
#endif

#if defined(USE_BARO) || defined(USE_RANGEFINDER)
static void taskCalculateAltitude(timeUs_t currentTimeUs)
{
    if (false
#if defined(USE_BARO)
        || (sensors(SENSOR_BARO) && isBaroReady())
#endif
#if defined(USE_RANGEFINDER)
        || sensors(SENSOR_RANGEFINDER)
#endif
        ) {
        calculateEstimatedAltitude(currentTimeUs);
    }}
#endif // USE_BARO || USE_RANGEFINDER

#ifdef USE_TELEMETRY
static void taskTelemetry(timeUs_t currentTimeUs)
{
    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(currentTimeUs);
    }
}
#endif

#ifdef USE_CAMERA_CONTROL
void taskCameraControl(uint32_t currentTime)
{
    if (ARMING_FLAG(ARMED)) {
        return;
    }

    cameraControlProcess(currentTime);
}
#endif

void fcTasksInit(void)
{
    schedulerInit();
    setTaskEnabled(TASK_SERIAL, true);
    rescheduleTask(TASK_SERIAL, TASK_PERIOD_HZ(serialConfig()->serial_update_rate_hz));

    const bool useBatteryVoltage = batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;
    setTaskEnabled(TASK_BATTERY_VOLTAGE, useBatteryVoltage);
    const bool useBatteryCurrent = batteryConfig()->currentMeterSource != CURRENT_METER_NONE;
    setTaskEnabled(TASK_BATTERY_CURRENT, useBatteryCurrent);
#ifdef USE_OSD_SLAVE
    const bool useBatteryAlerts = batteryConfig()->useVBatAlerts || batteryConfig()->useConsumptionAlerts;
#else
    const bool useBatteryAlerts = batteryConfig()->useVBatAlerts || batteryConfig()->useConsumptionAlerts || feature(FEATURE_OSD);
#endif
    setTaskEnabled(TASK_BATTERY_ALERTS, (useBatteryVoltage || useBatteryCurrent) && useBatteryAlerts);

#ifdef USE_TRANSPONDER
    setTaskEnabled(TASK_TRANSPONDER, feature(FEATURE_TRANSPONDER));
#endif

#ifdef STACK_CHECK
    setTaskEnabled(TASK_STACK_CHECK, true);
#endif

#ifdef USE_OSD_SLAVE
    setTaskEnabled(TASK_OSD_SLAVE, true);
#else
    if (sensors(SENSOR_GYRO)) {
        rescheduleTask(TASK_GYROPID, gyro.targetLooptime);
        setTaskEnabled(TASK_GYROPID, true);
    }

    if (sensors(SENSOR_ACC)) {
        setTaskEnabled(TASK_ACCEL, true);
        rescheduleTask(TASK_ACCEL, acc.accSamplingInterval);
        setTaskEnabled(TASK_ATTITUDE, true);
    }

    setTaskEnabled(TASK_RX, true);

    setTaskEnabled(TASK_DISPATCH, dispatchIsEnabled());

#ifdef BEEPER
    setTaskEnabled(TASK_BEEPER, true);
#endif
#ifdef USE_GPS
    setTaskEnabled(TASK_GPS, feature(FEATURE_GPS));
#endif
#ifdef USE_MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#endif
#ifdef USE_BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif
#ifdef USE_RANGEFINDER
    setTaskEnabled(TASK_RANGEFINDER, sensors(SENSOR_RANGEFINDER));
#endif
#if defined(USE_BARO) || defined(USE_RANGEFINDER)
    setTaskEnabled(TASK_ALTITUDE, sensors(SENSOR_BARO) || sensors(SENSOR_RANGEFINDER));
#endif
#ifdef USE_DASHBOARD
    setTaskEnabled(TASK_DASHBOARD, feature(FEATURE_DASHBOARD));
#endif
#ifdef USE_TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        setTaskEnabled(TASK_TELEMETRY, true);
        if (rxConfig()->serialrx_provider == SERIALRX_JETIEXBUS) {
            // Reschedule telemetry to 500hz for Jeti Exbus
            rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
        } else if (rxConfig()->serialrx_provider == SERIALRX_CRSF) {
            // Reschedule telemetry to 500hz, 2ms for CRSF
            rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
        }
    }
#endif
#ifdef USE_LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, feature(FEATURE_LED_STRIP));
#endif
#ifdef USE_TRANSPONDER
    setTaskEnabled(TASK_TRANSPONDER, feature(FEATURE_TRANSPONDER));
#endif
#ifdef USE_OSD
    setTaskEnabled(TASK_OSD, feature(FEATURE_OSD));
#endif
#ifdef USE_BST
    setTaskEnabled(TASK_BST_MASTER_PROCESS, true);
#endif
#ifdef USE_ESC_SENSOR
    setTaskEnabled(TASK_ESC_SENSOR, feature(FEATURE_ESC_SENSOR));
#endif
#ifdef USE_ADC_INTERNAL
    setTaskEnabled(TASK_ADC_INTERNAL, true);
#endif
#ifdef USE_PINIOBOX
    setTaskEnabled(TASK_PINIOBOX, true);
#endif
#ifdef USE_CMS
#ifdef USE_MSP_DISPLAYPORT
    setTaskEnabled(TASK_CMS, true);
#else
    setTaskEnabled(TASK_CMS, feature(FEATURE_OSD) || feature(FEATURE_DASHBOARD));
#endif
#endif
#ifdef USE_VTX_CONTROL
#if defined(USE_VTX_RTC6705) || defined(USE_VTX_SMARTAUDIO) || defined(USE_VTX_TRAMP)
    setTaskEnabled(TASK_VTXCTRL, true);
#endif
#endif
#ifdef USE_CAMERA_CONTROL
    setTaskEnabled(TASK_CAMCTRL, true);
#endif
#ifdef USE_RCDEVICE
    setTaskEnabled(TASK_RCDEVICE, rcdeviceIsEnabled());
#endif
#endif
}

cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
        .desiredPeriod = TASK_PERIOD_HZ(10),        // 10Hz, every 100 ms
        .staticPriority = TASK_PRIORITY_MEDIUM_HIGH,
    },

    [TASK_SERIAL] = {
        .taskName = "SERIAL",
        .taskFunc = taskHandleSerial,
#ifdef USE_OSD_SLAVE
        .checkFunc = taskSerialCheck,
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .staticPriority = TASK_PRIORITY_REALTIME,
#else
        .desiredPeriod = TASK_PERIOD_HZ(100),       // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
        .staticPriority = TASK_PRIORITY_LOW,
#endif
    },

    [TASK_BATTERY_ALERTS] = {
        .taskName = "BATTERY_ALERTS",
        .taskFunc = taskBatteryAlerts,
        .desiredPeriod = TASK_PERIOD_HZ(5),        // 5 Hz
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_BATTERY_VOLTAGE] = {
        .taskName = "BATTERY_VOLTAGE",
        .taskFunc = batteryUpdateVoltage,
        .desiredPeriod = TASK_PERIOD_HZ(50),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
    [TASK_BATTERY_CURRENT] = {
        .taskName = "BATTERY_CURRENT",
        .taskFunc = batteryUpdateCurrentMeter,
        .desiredPeriod = TASK_PERIOD_HZ(50),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

#ifdef USE_TRANSPONDER
    [TASK_TRANSPONDER] = {
        .taskName = "TRANSPONDER",
        .taskFunc = transponderUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(250),       // 250 Hz, 4ms
        .staticPriority = TASK_PRIORITY_LOW,
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

#ifdef USE_OSD_SLAVE
    [TASK_OSD_SLAVE] = {
        .taskName = "OSD_SLAVE",
        .checkFunc = osdSlaveCheck,
        .taskFunc = osdSlaveUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(60),        // 60 Hz
        .staticPriority = TASK_PRIORITY_HIGH,
    },

#else

    [TASK_GYROPID] = {
        .taskName = "PID",
        .subTaskName = "GYRO",
        .taskFunc = taskMainPidLoop,
        .desiredPeriod = TASK_GYROPID_DESIRED_PERIOD,
        .staticPriority = TASK_PRIORITY_REALTIME,
    },

    [TASK_ACCEL] = {
        .taskName = "ACC",
        .taskFunc = taskUpdateAccelerometer,
        .desiredPeriod = TASK_PERIOD_HZ(1000),      // 1000Hz, every 1ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_ATTITUDE] = {
        .taskName = "ATTITUDE",
        .taskFunc = imuUpdateAttitude,
        .desiredPeriod = TASK_PERIOD_HZ(100),
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = rxUpdateCheck,
        .taskFunc = taskUpdateRxMain,
        .desiredPeriod = TASK_PERIOD_HZ(50),        // If event-based scheduling doesn't work, fallback to periodic scheduling
        .staticPriority = TASK_PRIORITY_HIGH,
    },

    [TASK_DISPATCH] = {
        .taskName = "DISPATCH",
        .taskFunc = dispatchProcess,
        .desiredPeriod = TASK_PERIOD_HZ(1000),
        .staticPriority = TASK_PRIORITY_HIGH,
    },

#ifdef BEEPER
    [TASK_BEEPER] = {
        .taskName = "BEEPER",
        .taskFunc = beeperUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),       // 100 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_GPS
    [TASK_GPS] = {
        .taskName = "GPS",
        .taskFunc = gpsUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),        // Required to prevent buffer overruns if running at 115200 baud (115 bytes / period < 256 bytes buffer)
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_MAG
    [TASK_COMPASS] = {
        .taskName = "COMPASS",
        .taskFunc = compassUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10),        // Compass is updated at 10 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_BARO
    [TASK_BARO] = {
        .taskName = "BARO",
        .taskFunc = taskUpdateBaro,
        .desiredPeriod = TASK_PERIOD_HZ(20),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_RANGEFINDER
    [TASK_RANGEFINDER] = {
        .taskName = "RANGEFINDER",
        .taskFunc = rangefinderUpdate,
        .desiredPeriod = TASK_PERIOD_MS(70), // XXX HCSR04 sonar specific value.
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#if defined(USE_BARO) || defined(USE_RANGEFINDER)
    [TASK_ALTITUDE] = {
        .taskName = "ALTITUDE",
        .taskFunc = taskCalculateAltitude,
        .desiredPeriod = TASK_PERIOD_HZ(40),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_DASHBOARD
    [TASK_DASHBOARD] = {
        .taskName = "DASHBOARD",
        .taskFunc = dashboardUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10),
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_OSD
    [TASK_OSD] = {
        .taskName = "OSD",
        .taskFunc = osdUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(60),        // 60 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_TELEMETRY
    [TASK_TELEMETRY] = {
        .taskName = "TELEMETRY",
        .taskFunc = taskTelemetry,
        .desiredPeriod = TASK_PERIOD_HZ(250),       // 250 Hz, 4ms
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_LED_STRIP
    [TASK_LEDSTRIP] = {
        .taskName = "LEDSTRIP",
        .taskFunc = ledStripUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(100),       // 100 Hz, 10ms
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_BST
    [TASK_BST_MASTER_PROCESS] = {
        .taskName = "BST_MASTER_PROCESS",
        .taskFunc = taskBstMasterProcess,
        .desiredPeriod = TASK_PERIOD_HZ(50),        // 50 Hz, 20ms
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef USE_ESC_SENSOR
    [TASK_ESC_SENSOR] = {
        .taskName = "ESC_SENSOR",
        .taskFunc = escSensorProcess,
        .desiredPeriod = TASK_PERIOD_HZ(100),       // 100 Hz, 10ms
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_CMS
    [TASK_CMS] = {
        .taskName = "CMS",
        .taskFunc = cmsHandler,
        .desiredPeriod = TASK_PERIOD_HZ(60),        // 60 Hz
        .staticPriority = TASK_PRIORITY_LOW,
    },
#endif

#ifdef USE_VTX_CONTROL
    [TASK_VTXCTRL] = {
        .taskName = "VTXCTRL",
        .taskFunc = vtxUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(5),          // 5 Hz, 200ms
        .staticPriority = TASK_PRIORITY_IDLE,
    },
#endif

#ifdef USE_RCDEVICE
    [TASK_RCDEVICE] = {
        .taskName = "RCDEVICE",
        .taskFunc = rcdeviceUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(10),        // 10 Hz, 100ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif

#ifdef USE_CAMERA_CONTROL
    [TASK_CAMCTRL] = {
        .taskName = "CAMCTRL",
        .taskFunc = taskCameraControl,
        .desiredPeriod = TASK_PERIOD_HZ(5),
        .staticPriority = TASK_PRIORITY_IDLE
    },
#endif

#ifdef USE_ADC_INTERNAL
    [TASK_ADC_INTERNAL] = {
        .taskName = "ADCINTERNAL",
        .taskFunc = adcInternalProcess,
        .desiredPeriod = TASK_PERIOD_HZ(1),
        .staticPriority = TASK_PRIORITY_IDLE
    },
#endif

#ifdef USE_PINIOBOX
    [TASK_PINIOBOX] = {
        .taskName = "PINIOBOX",
        .taskFunc = pinioBoxUpdate,
        .desiredPeriod = TASK_PERIOD_HZ(20),
        .staticPriority = TASK_PRIORITY_IDLE
    },
#endif
#endif
};
