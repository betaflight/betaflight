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
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"

#include "flight/flight.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/autotune.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial_cli.h"
#include "io/serial.h"
#include "io/statusindicator.h"
#include "rx/rx.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "rx/msp.h"
#include "telemetry/telemetry.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

// June 2013     V2.2-dev

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

/* for VBAT monitoring frequency */
#define VBATFREQ 6        // to read battery voltage - nth number of loop iterations

int16_t debug[4];
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
int16_t headFreeModeHold;

int16_t telemTemperature1;      // gyro sensor temperature

extern uint8_t dynP8[3], dynI8[3], dynD8[3];
extern failsafe_t *failsafe;

typedef void (*pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination, rollAndPitchTrims_t *accelerometerTrims);

extern pidControllerFuncPtr pid_controller;

void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    currentProfile->accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
    currentProfile->accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;

    saveConfigAndNotify();
}

#ifdef AUTOTUNE

void updateAutotuneState(void)
{
    static bool landedAfterAutoTuning = false;
    static bool autoTuneWasUsed = false;

    if (rcOptions[BOXAUTOTUNE]) {
        if (!FLIGHT_MODE(AUTOTUNE_MODE)) {
            if (ARMING_FLAG(ARMED)) {
                if (isAutotuneIdle() || landedAfterAutoTuning) {
                    autotuneReset();
                    landedAfterAutoTuning = false;
                }
                autotuneBeginNextPhase(&currentProfile->pidProfile, currentProfile->pidController);
                ENABLE_FLIGHT_MODE(AUTOTUNE_MODE);
                autoTuneWasUsed = true;
            } else {
                if (havePidsBeenUpdatedByAutotune()) {
                    saveConfigAndNotify();
                    autotuneReset();
                }
            }
        }
        return;
    }

    if (FLIGHT_MODE(AUTOTUNE_MODE)) {
        autotuneEndPhase();
        DISABLE_FLIGHT_MODE(AUTOTUNE_MODE);
    }

    if (!ARMING_FLAG(ARMED) && autoTuneWasUsed) {
        landedAfterAutoTuning = true;
    }
}
#endif

bool isCalibrating()
{
#ifdef BARO
    if (sensors(SENSOR_ACC) && !isBaroCalibrationComplete()) {
        return false;
    }
#endif

    // Note: compass calibration is handled completely differently, outside of the main loop, see f.CALIBRATE_MAG

    return (!isAccelerationCalibrationComplete() && sensors(SENSOR_ACC)) || (!isGyroCalibrationComplete());
}

void annexCode(void)
{
    int32_t tmp, tmp2;
    int32_t axis, prop1, prop2;

    static uint8_t batteryWarningEnabled = false;
    static uint8_t vbatTimer = 0;
    static int32_t vbatCycleTime = 0;

    // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < currentProfile->tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)currentProfile->dynThrPID * (rcData[THROTTLE] - currentProfile->tpa_breakpoint) / (2000 - currentProfile->tpa_breakpoint);
        } else {
            prop2 = 100 - currentProfile->dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - masterConfig.rxConfig.midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (currentProfile->deadband) {
                if (tmp > currentProfile->deadband) {
                    tmp -= currentProfile->deadband;
                } else {
                    tmp = 0;
                }
            }

            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t)currentProfile->controlRateConfig.rollPitchRate * tmp / 500;
            prop1 = (uint16_t)prop1 * prop2 / 100;
        }
        if (axis == YAW) {
            if (currentProfile->yaw_deadband) {
                if (tmp > currentProfile->yaw_deadband) {
                    tmp -= currentProfile->yaw_deadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = tmp * -masterConfig.yaw_control_direction;
            prop1 = 100 - (uint16_t)currentProfile->controlRateConfig.yawRate * abs(tmp) / 500;
        }
        // FIXME axis indexes into pids.  use something like lookupPidIndex(rc_alias_e alias) to reduce coupling.
        dynP8[axis] = (uint16_t)currentProfile->pidProfile.P8[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)currentProfile->pidProfile.I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)currentProfile->pidProfile.D8[axis] * prop1 / 100;

        if (rcData[axis] < masterConfig.rxConfig.midrc)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], masterConfig.rxConfig.mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - masterConfig.rxConfig.mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - masterConfig.rxConfig.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if (FLIGHT_MODE(HEADFREE_MODE)) {
        float radDiff = degreesToRadians(heading - headFreeModeHold);
        float cosDiff = cosf(radDiff);
        float sinDiff = sinf(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }

    if (feature(FEATURE_VBAT | FEATURE_CURRENT_METER)) {
        vbatCycleTime += cycleTime;
        if (!(++vbatTimer % VBATFREQ)) {

        	if (feature(FEATURE_VBAT)) {
        		updateBatteryVoltage();
                batteryWarningEnabled = shouldSoundBatteryAlarm();
        	}

        	if (feature(FEATURE_CURRENT_METER)) {
        		updateCurrentMeter(vbatCycleTime);
        	}
        	vbatCycleTime = 0;
        }
    }

    beepcodeUpdateState(batteryWarningEnabled);

    if (ARMING_FLAG(ARMED)) {
        LED0_ON;
    } else {
        if (rcOptions[BOXARM] == 0) {
            ENABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (isCalibrating()) {
            LED0_TOGGLE;
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (!STATE(SMALL_ANGLE)) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (rcOptions[BOXAUTOTUNE]) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (ARMING_FLAG(OK_TO_ARM)) {
            disableWarningLed();
        } else {
            enableWarningLed(currentTime);
        }

        updateWarningLed(currentTime);
    }

#if 0
    debug[0] = armingFlags;
#endif

#ifdef TELEMETRY
    checkTelemetryState();
#endif

    handleSerial();

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTime);
    }
#endif

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);
}

void mwDisarm(void)
{
    if (ARMING_FLAG(ARMED))
        DISABLE_ARMING_FLAG(ARMED);
}

void mwArm(void)
{
    if (ARMING_FLAG(OK_TO_ARM)) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }
        if (!ARMING_FLAG(PREVENT_ARMING)) {
            ENABLE_ARMING_FLAG(ARMED);
            headFreeModeHold = heading;
            return;
        }
    }

    if (!ARMING_FLAG(ARMED)) {
        blinkLedAndSoundBeeper(2, 255, 1);
    }
}

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
uint16_t InflightcalibratingA = 0;

void handleInflightCalibrationStickPosition(void)
{
    if (AccInflightCalibrationMeasurementDone) {
        // trigger saving into eeprom after landing
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    } else {
        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
        if (AccInflightCalibrationArmed) {
            queueConfirmationBeep(2);
        } else {
            queueConfirmationBeep(3);
        }
    }
}

void updateInflightCalibrationState(void)
{
    if (AccInflightCalibrationArmed && ARMING_FLAG(ARMED) && rcData[THROTTLE] > masterConfig.rxConfig.mincheck && !rcOptions[BOXARM]) {   // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = false;
    }
    if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
            InflightcalibratingA = 50;
        AccInflightCalibrationActive = true;
    } else if (AccInflightCalibrationMeasurementDone && !ARMING_FLAG(ARMED)) {
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    }
}

void updateMagHold(void)
{
    if (abs(rcCommand[YAW]) < 70 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = heading - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -masterConfig.yaw_control_direction;
        if (STATE(SMALL_ANGLE))
            rcCommand[YAW] -= dif * currentProfile->pidProfile.P8[PIDMAG] / 30;    // 18 deg
    } else
        magHold = heading;
}

typedef enum {
#ifdef MAG
	UPDATE_COMPASS_TASK,
#endif
#ifdef BARO
	UPDATE_BARO_TASK,
#endif
#ifdef SONAR
	UPDATE_SONAR_TASK,
#endif
#if defined(BARO) || defined(SONAR)
	CALCULATE_ALTITUDE_TASK,
#endif
	UPDATE_GPS_TASK
} periodicTasks;

#define PERIODIC_TASK_COUNT (UPDATE_GPS_TASK + 1)


void executePeriodicTasks(void)
{
    static int periodicTaskIndex = 0;

    switch (periodicTaskIndex++) {
#ifdef MAG
    case UPDATE_COMPASS_TASK:
        if (sensors(SENSOR_MAG)) {
            updateCompass(&masterConfig.magZero);
        }
        break;
#endif

#ifdef BARO
    case UPDATE_BARO_TASK:
        if (sensors(SENSOR_BARO)) {
            baroUpdate(currentTime);
        }
        break;
#endif

#if defined(BARO) || defined(SONAR)
    case CALCULATE_ALTITUDE_TASK:

#if defined(BARO) && !defined(SONAR)
        if (sensors(SENSOR_BARO) && isBaroReady()) {
#endif
#if defined(BARO) && defined(SONAR)
        if ((sensors(SENSOR_BARO) && isBaroReady()) || sensors(SENSOR_SONAR)) {
#endif
#if !defined(BARO) && defined(SONAR)
        if (sensors(SENSOR_SONAR)) {
#endif
            calculateEstimatedAltitude(currentTime);
        }
        break;
#endif

#ifdef GPS
    case UPDATE_GPS_TASK:
        // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
        // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
        // change this based on available hardware
        if (feature(FEATURE_GPS)) {
            gpsThread();
        }
        break;
#endif
#ifdef SONAR
    case UPDATE_SONAR_TASK:
        if (sensors(SENSOR_SONAR)) {
            Sonar_update();
        }
        break;
#endif
    }

    if (periodicTaskIndex >= PERIODIC_TASK_COUNT) {
        periodicTaskIndex = 0;
    }
}

void processRx(void)
{
    calculateRxChannelsAndUpdateFailsafe(currentTime);

    // in 3D mode, we need to be able to disarm by switch at any time
    if (feature(FEATURE_3D)) {
        if (!rcOptions[BOXARM])
            mwDisarm();
    }

    updateRSSI(currentTime);

    if (feature(FEATURE_FAILSAFE)) {

        if (currentTime > FAILSAFE_POWER_ON_DELAY_US && !failsafe->vTable->isEnabled()) {
            failsafe->vTable->enable();
        }

        failsafe->vTable->updateState();
    }

    throttleStatus_e throttleStatus = calculateThrottleStatus(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);

    if (throttleStatus == THROTTLE_LOW) {
        resetErrorAngle();
        resetErrorGyro();
    }

    processRcStickPositions(&masterConfig.rxConfig, throttleStatus, currentProfile->activate, masterConfig.retarded_arm);

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        updateInflightCalibrationState();
    }

    updateRcOptions(currentProfile->activate);

    bool canUseHorizonMode = true;

    if ((rcOptions[BOXANGLE] || (feature(FEATURE_FAILSAFE) && failsafe->vTable->hasTimerElapsed())) && (sensors(SENSOR_ACC))) {
        // bumpless transfer to Level mode
    	canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
            resetErrorAngle();
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
    }

	if (rcOptions[BOXHORIZON] && canUseHorizonMode) {

		DISABLE_FLIGHT_MODE(ANGLE_MODE);

		if (!FLIGHT_MODE(HORIZON_MODE)) {
			resetErrorAngle();
			ENABLE_FLIGHT_MODE(HORIZON_MODE);
		}
	} else {
		DISABLE_FLIGHT_MODE(HORIZON_MODE);
	}

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        LED1_ON;
    } else {
        LED1_OFF;
    }

#ifdef  MAG
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        if (rcOptions[BOXMAG]) {
            if (!FLIGHT_MODE(MAG_MODE)) {
                ENABLE_FLIGHT_MODE(MAG_MODE);
                magHold = heading;
            }
        } else {
            DISABLE_FLIGHT_MODE(MAG_MODE);
        }
        if (rcOptions[BOXHEADFREE]) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
        if (rcOptions[BOXHEADADJ]) {
            headFreeModeHold = heading; // acquire new heading
        }
    }
#endif

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        updateGpsWaypointsAndMode();
    }
#endif

    if (rcOptions[BOXPASSTHRU]) {
        ENABLE_FLIGHT_MODE(PASSTHRU_MODE);
    } else {
        DISABLE_FLIGHT_MODE(PASSTHRU_MODE);
    }

    if (masterConfig.mixerConfiguration == MULTITYPE_FLYING_WING || masterConfig.mixerConfiguration == MULTITYPE_AIRPLANE) {
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
    }
}

void loop(void)
{
    static uint32_t loopTime;
#if defined(BARO) || defined(SONAR)
    static bool haveProcessedAnnexCodeOnce = false;
#endif

    updateRx();

    if (shouldProcessRx(currentTime)) {
        processRx();

#ifdef BARO
	// the 'annexCode' initialses rcCommand, updateAltHoldState depends on valid rcCommand data.
	if (haveProcessedAnnexCodeOnce) {
		if (sensors(SENSOR_BARO)) {
			updateAltHoldState();
		}
	}
#endif

#ifdef SONAR
	// the 'annexCode' initialses rcCommand, updateAltHoldState depends on valid rcCommand data.
	if (haveProcessedAnnexCodeOnce) {
		if (sensors(SENSOR_SONAR)) {
			updateSonarAltHoldState();
		}
	}
#endif

    } else {
        // not processing rx this iteration
        executePeriodicTasks();
    }

    currentTime = micros();
    if (masterConfig.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + masterConfig.looptime;

        computeIMU(&currentProfile->accelerometerTrims, masterConfig.mixerConfiguration);

        // Measure loop rate just after reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;

        annexCode();
#if defined(BARO) || defined(SONAR)
        haveProcessedAnnexCodeOnce = true;
#endif

#ifdef AUTOTUNE
        updateAutotuneState();
#endif

#ifdef MAG
        if (sensors(SENSOR_MAG)) {
        	updateMagHold();
        }
#endif

#if defined(BARO) || defined(SONAR)
        if (sensors(SENSOR_BARO) || sensors(SENSOR_SONAR)) {
            if (FLIGHT_MODE(BARO_MODE) || FLIGHT_MODE(SONAR_MODE)) {
                updateAltHold();
            }
        }
#endif

        if (currentProfile->throttle_correction_value && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
            rcCommand[THROTTLE] += calculateThrottleAngleCorrection(currentProfile->throttle_correction_value);
        }

#ifdef GPS
        if (sensors(SENSOR_GPS)) {
            if ((FLIGHT_MODE(GPS_HOME_MODE) || FLIGHT_MODE(GPS_HOLD_MODE)) && STATE(GPS_FIX_HOME)) {
                updateGpsStateForHomeAndHoldMode();
            }
        }
#endif

        // PID - note this is function pointer set by setPIDController()
        pid_controller(
			&currentProfile->pidProfile,
			&currentProfile->controlRateConfig,
			masterConfig.max_angle_inclination,
			&currentProfile->accelerometerTrims
        );

        mixTable();
        writeServos();
        writeMotors();
    }

#ifdef TELEMETRY
    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        handleTelemetry();
    }
#endif

#ifdef LED_STRIP
    if (feature(FEATURE_LED_STRIP)) {
        updateLedStrip();
    }
#endif
}
