/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#include "cli/cli.h"

#include "cms/cms.h"

#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/camera_control.h"
#include "drivers/compass/compass.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/stack_check.h"
#include "drivers/transponder_ir.h"
#include "drivers/usb_io.h"
#include "drivers/vtx_common.h"

#include "fc/config.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/dispatch.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/piniobox.h"
#include "io/serial.h"
#include "io/transponder_ir.h"
#include "io/vtx_tramp.h" // Will be gone
#include "io/rcdevice_cam.h"
#include "io/usb_cdc_hid.h"
#include "io/vtx.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"
#include "sensors/rangefinder.h"

#include "telemetry/telemetry.h"

#ifdef USE_BST
#include "i2c_bst.h"
#endif

#ifdef USE_USB_CDC_HID
//TODO: Make it platform independent in the future
#ifdef STM32F4
#include "vcpf4/usbd_cdc_vcp.h"
#include "usbd_hid_core.h"
#elif defined(STM32F7)
#include "usbd_cdc_interface.h"
#include "usbd_hid.h"
#endif
#endif

#include "tasks.h"
#include "FreeRTOS.h"
#include "task.h"

// Target stack margin per task. See cfTasks array, below, for details.
#define TASK_STACK_MARGIN 16

#define TASK_STACK_DEFAULT 128

static void taskMain()
{
#ifdef USE_SDCARD
    afatfs_poll();
#endif
}

static void taskHandleSerial()
{
#if defined(USE_VCP)
    DEBUG_SET(DEBUG_USB, 0, usbCableIsInserted());
    DEBUG_SET(DEBUG_USB, 1, usbVcpIsConnected());
#endif

#ifdef USE_CLI
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }
#endif
    bool evaluateMspData = ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA;
    mspSerialProcess(evaluateMspData, mspFcProcessCommand, mspFcProcessReply);
}

static void taskBatteryAlerts()
{
    if (!ARMING_FLAG(ARMED)) {
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }
    batteryUpdateStates();
    batteryUpdateAlarms();
}

static void taskUpdateRxMain()
{
	rxUpdateCheck();

	if (!processRx()) {
        return;
    }

    isRXDataNew = true;

#ifdef USE_USB_CDC_HID
    if (!ARMING_FLAG(ARMED)) {
        sendRcDataToHid();
    }
#endif

    // updateRcCommands sets rcCommand, which is needed by updateAltHoldState and updateSonarAltHoldState
    updateRcCommands();
    updateArmingStatus();
}

#ifdef USE_BARO
static void taskUpdateBaro()
{
    if (sensors(SENSOR_BARO)) {
        cfTasks[TASK_BARO].desiredPeriod = baroUpdate();
    }
}
#endif

#if defined(USE_BARO) || defined(USE_GPS)
static void taskCalculateAltitude()
{
    calculateEstimatedAltitude();
}
#endif // USE_BARO || USE_GPS

#ifdef USE_TELEMETRY
static void taskTelemetry()
{
    if (!cliMode && featureIsEnabled(FEATURE_TELEMETRY)) {
        subTaskTelemetryPollSensors();
        telemetryProcess();
    }
}
#endif

#ifdef USE_CAMERA_CONTROL
static void taskCameraControl()
{
    if (ARMING_FLAG(ARMED)) {
        return;
    }

    cameraControlProcess();
}
#endif

void setTaskEnabled(cfTaskId_e taskId, bool enabled)
{
    TaskHandle_t task = (taskId == TASK_SELF) ? xTaskGetCurrentTaskHandle() : cfTasks[taskId].taskId;

    if (task) {
		if (enabled) {
			vTaskResume(task);
		} else {
			vTaskSuspend(task);
		}
    } else if (!enabled) {
    	// The task is not yet running, so prevent this happening
    	cfTasks[taskId].taskFunc = NULL;
    }
}

void fcTaskReschedule(cfTaskId_e taskId, uint32_t newPeriod)
{
    cfTasks[taskId].desiredPeriod = newPeriod;
}

uint16_t fcTaskGetPeriod(cfTaskId_e taskId)
{
    return cfTasks[taskId].desiredPeriod;
}

void fcTasksInit(void)
{
    setTaskEnabled(TASK_MAIN, true);

    setTaskEnabled(TASK_SERIAL, true);
    fcTaskReschedule(TASK_SERIAL, TASK_PERIOD_HZ(serialConfig()->serial_update_rate_hz));

    const bool useBatteryVoltage = batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE;
    setTaskEnabled(TASK_BATTERY_VOLTAGE, useBatteryVoltage);
    const bool useBatteryCurrent = batteryConfig()->currentMeterSource != CURRENT_METER_NONE;
    setTaskEnabled(TASK_BATTERY_CURRENT, useBatteryCurrent);
    const bool useBatteryAlerts = batteryConfig()->useVBatAlerts || batteryConfig()->useConsumptionAlerts || featureIsEnabled(FEATURE_OSD);
    setTaskEnabled(TASK_BATTERY_ALERTS, (useBatteryVoltage || useBatteryCurrent) && useBatteryAlerts);

#ifdef USE_TRANSPONDER
    setTaskEnabled(TASK_TRANSPONDER, featureIsEnabled(FEATURE_TRANSPONDER));
#endif

#ifdef STACK_CHECK
    setTaskEnabled(TASK_STACK_CHECK, true);
#endif

    if (sensors(SENSOR_GYRO)) {
        fcTaskReschedule(TASK_GYROPID, gyro.targetLooptime);
        setTaskEnabled(TASK_GYROPID, true);
    }

#ifdef USE_RANGEFINDER
    if (sensors(SENSOR_RANGEFINDER)) {
        setTaskEnabled(TASK_RANGEFINDER, featureIsEnabled(FEATURE_RANGEFINDER));
    }
#endif

    setTaskEnabled(TASK_RX, true);

    setTaskEnabled(TASK_DISPATCH, dispatchIsEnabled());

#ifdef USE_BEEPER
    setTaskEnabled(TASK_BEEPER, true);
#endif

#ifdef USE_GPS
    setTaskEnabled(TASK_GPS, featureIsEnabled(FEATURE_GPS));
#endif

#ifdef USE_MAG
    setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
#endif

#ifdef USE_BARO
    setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
#endif

#if defined(USE_BARO) || defined(USE_GPS)
    setTaskEnabled(TASK_ALTITUDE, sensors(SENSOR_BARO) || featureIsEnabled(FEATURE_GPS));
#endif

#ifdef USE_DASHBOARD
    setTaskEnabled(TASK_DASHBOARD, featureIsEnabled(FEATURE_DASHBOARD));
#endif

#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        setTaskEnabled(TASK_TELEMETRY, true);
        if (rxConfig()->serialrx_provider == SERIALRX_JETIEXBUS) {
            // Reschedule telemetry to 500hz for Jeti Exbus
            fcTaskReschedule(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
        } else if (rxConfig()->serialrx_provider == SERIALRX_CRSF) {
            // Reschedule telemetry to 500hz, 2ms for CRSF
            fcTaskReschedule(TASK_TELEMETRY, TASK_PERIOD_HZ(500));
        }
    }
#endif

#ifdef USE_LED_STRIP
    setTaskEnabled(TASK_LEDSTRIP, featureIsEnabled(FEATURE_LED_STRIP));
#endif

#ifdef USE_TRANSPONDER
    setTaskEnabled(TASK_TRANSPONDER, featureIsEnabled(FEATURE_TRANSPONDER));
#endif

#ifdef USE_OSD
    setTaskEnabled(TASK_OSD, featureIsEnabled(FEATURE_OSD) && osdInitialized());
#endif

#ifdef USE_BST
    setTaskEnabled(TASK_BST_MASTER_PROCESS, true);
#endif

#ifdef USE_ESC_SENSOR
    setTaskEnabled(TASK_ESC_SENSOR, featureIsEnabled(FEATURE_ESC_SENSOR));
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
    setTaskEnabled(TASK_CMS, featureIsEnabled(FEATURE_OSD) || featureIsEnabled(FEATURE_DASHBOARD));
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
}

#if defined(USE_TASK_STATISTICS)
#define DEFINE_TASK(taskNameParam, subTaskNameParam, taskFuncParam, desiredPeriodParam, priorityParam, stackSizeParam) {  \
    .taskName = taskNameParam, \
    .subTaskName = subTaskNameParam, \
    .taskFunc = taskFuncParam, \
    .desiredPeriod = desiredPeriodParam, \
    .priority = priorityParam, \
	.stackSize = stackSizeParam \
}
#else
#define DEFINE_TASK(taskNameParam, subTaskNameParam, taskFuncParam, desiredPeriodParam, priorityParam, stackSizeParam) {  \
    .taskFunc = taskFuncParam, \
    .desiredPeriod = desiredPeriodParam, \
    .priority = priorityParam \
 	.stackSize = stackSizeParam \
}
#endif

void FAST_CODE FAST_CODE_NOINLINE task( void *pvParameters )
{
	cfTask_t *task = (cfTask_t *)pvParameters;

	task->lastExecutedAt = xTaskGetTickCount();

    while (true) {
        task->taskFunc(micros());

        vTaskDelayUntil(&task->lastExecutedAt, (TickType_t)(configTICK_RATE_HZ * (uint32_t)task->desiredPeriod/1000));
    }
}

void tasksLaunch()
{
	// Launch each task in turn
	for (int i = 0; i < TASK_COUNT; i++) {
		if (cfTasks[i].taskFunc && (i != TASK_GYROPID)) {
			// Apply default stack size if none specified
			if (cfTasks[i].stackSize == 0) {
				cfTasks[i].stackSize = TASK_STACK_DEFAULT;
			}
			xTaskCreate(task,
					    cfTasks[i].taskName,
						cfTasks[i].stackSize + TASK_STACK_MARGIN,
						(void *)&cfTasks[i],
						tskIDLE_PRIORITY + cfTasks[i].priority,
						&cfTasks[i].taskId);
		}
	}

	xTaskCreate(taskMainPidLoop,
			    cfTasks[TASK_GYROPID].taskName,
				cfTasks[TASK_GYROPID].stackSize + TASK_STACK_MARGIN,
			    (void *)&cfTasks[TASK_GYROPID],
			    tskIDLE_PRIORITY + cfTasks[TASK_GYROPID].priority,
			    &cfTasks[TASK_GYROPID].taskId);
}

/* Note that stack sizes, where defined, are set to leave TASK_STACK_MARGIN words on each stack free. This may need review/revision. */
cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_MAIN] = DEFINE_TASK("SYSTEM", "UPDATE", taskMain, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM_HIGH, 33),
    [TASK_SERIAL] = DEFINE_TASK("SERIAL", NULL, taskHandleSerial, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW, 0), // 100 Hz should be enough to flush up to 115 bytes @ 115200 baud
    [TASK_BATTERY_ALERTS] = DEFINE_TASK("BATTERY_ALERTS", NULL, taskBatteryAlerts, TASK_PERIOD_HZ(5), TASK_PRIORITY_MEDIUM, 33),
    [TASK_BATTERY_VOLTAGE] = DEFINE_TASK("BATTERY_VOLTAGE", NULL, batteryUpdateVoltage, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM, 71),
    [TASK_BATTERY_CURRENT] = DEFINE_TASK("BATTERY_CURRENT", NULL, batteryUpdateCurrentMeter, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM, 67),

#ifdef USE_TRANSPONDER
    [TASK_TRANSPONDER] = DEFINE_TASK("TRANSPONDER", NULL, transponderUpdate, TASK_PERIOD_HZ(250), TASK_PRIORITY_LOW, 0),
#endif

#ifdef STACK_CHECK
    [TASK_STACK_CHECK] = DEFINE_TASK("STACKCHECK", NULL, taskStackCheck, TASK_PERIOD_HZ(10), TASK_PRIORITY_IDLE, 0),
#endif

	[TASK_GYROPID] = DEFINE_TASK("PID", "GYRO", NULL, TASK_GYROPID_DESIRED_PERIOD, TASK_PRIORITY_REALTIME, 101),

    [TASK_ATTITUDE] = DEFINE_TASK("ATTITUDE", NULL, imuUpdateAttitude, TASK_PERIOD_HZ(100), TASK_PRIORITY_MEDIUM, 113),
    [TASK_RX] = DEFINE_TASK("RX", NULL, taskUpdateRxMain, TASK_PERIOD_HZ(150), TASK_PRIORITY_HIGH, 83), // If event-based scheduling doesn't work, fallback to periodic scheduling
    [TASK_DISPATCH] = DEFINE_TASK("DISPATCH", NULL, dispatchProcess, TASK_PERIOD_HZ(1000), TASK_PRIORITY_HIGH, 33),

#ifdef USE_BEEPER
    [TASK_BEEPER] = DEFINE_TASK("BEEPER", NULL, beeperUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW, 73),
#endif

#ifdef USE_GPS
    [TASK_GPS] = DEFINE_TASK("GPS", NULL, gpsUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_MEDIUM, 33), // Required to prevent buffer overruns if running at 115200 baud (115 bytes / period < 256 bytes buffer)
#endif

#ifdef USE_MAG
    [TASK_COMPASS] = DEFINE_TASK("COMPASS", NULL, compassUpdate,TASK_PERIOD_HZ(10), TASK_PRIORITY_LOW, 0),
#endif

#ifdef USE_BARO
    [TASK_BARO] = DEFINE_TASK("BARO", NULL, taskUpdateBaro, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW, 50),
#endif

#if defined(USE_BARO) || defined(USE_GPS)
    [TASK_ALTITUDE] = DEFINE_TASK("ALTITUDE", NULL, taskCalculateAltitude, TASK_PERIOD_HZ(40), TASK_PRIORITY_LOW, 80),
#endif

#ifdef USE_DASHBOARD
    [TASK_DASHBOARD] = DEFINE_TASK("DASHBOARD", NULL, dashboardUpdate, TASK_PERIOD_HZ(10), TASK_PRIORITY_LOW, 33),
#endif

#ifdef USE_OSD
    [TASK_OSD] = DEFINE_TASK("OSD", NULL, osdUpdate, TASK_PERIOD_HZ(60), TASK_PRIORITY_LOW, 33),
#endif

#ifdef USE_TELEMETRY
    [TASK_TELEMETRY] = DEFINE_TASK("TELEMETRY", NULL, taskTelemetry, TASK_PERIOD_HZ(250), TASK_PRIORITY_LOW, 33),
#endif

#ifdef USE_LED_STRIP
    [TASK_LEDSTRIP] = DEFINE_TASK("LEDSTRIP", NULL, ledStripUpdate, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW, 0),
#endif

#ifdef USE_BST
    [TASK_BST_MASTER_PROCESS] = DEFINE_TASK("BST_MASTER_PROCESS", NULL, taskBstMasterProcess, TASK_PERIOD_HZ(50), TASK_PRIORITY_IDLE, 0),
#endif

#ifdef USE_ESC_SENSOR
    [TASK_ESC_SENSOR] = DEFINE_TASK("ESC_SENSOR", NULL, escSensorProcess, TASK_PERIOD_HZ(100), TASK_PRIORITY_LOW, 33),
#endif

#ifdef USE_CMS
    [TASK_CMS] = DEFINE_TASK("CMS", NULL, cmsHandler, TASK_PERIOD_HZ(60), TASK_PRIORITY_LOW, 33),
#endif

#ifdef USE_VTX_CONTROL
    [TASK_VTXCTRL] = DEFINE_TASK("VTXCTRL", NULL, vtxUpdate, TASK_PERIOD_HZ(5), TASK_PRIORITY_IDLE, 33),
#endif

#ifdef USE_RCDEVICE
    [TASK_RCDEVICE] = DEFINE_TASK("RCDEVICE", NULL, rcdeviceUpdate, TASK_PERIOD_HZ(20), TASK_PRIORITY_MEDIUM, 33),
#endif

#ifdef USE_CAMERA_CONTROL
    [TASK_CAMCTRL] = DEFINE_TASK("CAMCTRL", NULL, taskCameraControl, TASK_PERIOD_HZ(5), TASK_PRIORITY_IDLE, 33),
#endif

#ifdef USE_ADC_INTERNAL
    [TASK_ADC_INTERNAL] = DEFINE_TASK("ADCINTERNAL", NULL, adcInternalProcess, TASK_PERIOD_HZ(1), TASK_PRIORITY_IDLE, 33),
#endif

#ifdef USE_PINIOBOX
    [TASK_PINIOBOX] = DEFINE_TASK("PINIOBOX", NULL, pinioBoxUpdate, TASK_PERIOD_HZ(20), TASK_PRIORITY_IDLE, 33),
#endif

#ifdef USE_RANGEFINDER
    [TASK_RANGEFINDER] = DEFINE_TASK("RANGEFINDER", NULL, rangefinderUpdate, TASK_PERIOD_HZ(10), TASK_PRIORITY_IDLE, 0),
#endif
};
