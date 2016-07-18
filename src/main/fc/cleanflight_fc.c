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

#include <platform.h>
#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"
#include "common/streambuf.h"

#include "config/parameter_group.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/gyro_sync.h"

#include "fc/rc_controls.h"
#include "fc/rate_profile.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_curves.h"
#include "fc/fc_serial.h"
#include "fc/fc_tasks.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/statusindicator.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"

#include "fc/runtime_config.h"
#include "fc/config.h"
#include "config/feature.h"

// June 2013     V2.2-dev

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)
#define GYRO_WATCHDOG_DELAY 100  // Watchdog for boards without interrupt for gyro

uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

float dT;

int16_t magHold;
int16_t headFreeModeHold;

uint8_t motorControlEnable = false;

int16_t telemTemperature1;      // gyro sensor temperature
static uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

extern uint32_t currentTime;
extern uint8_t PIDweight[3];
extern uint8_t dynP8[3], dynI8[3], dynD8[3];

static bool isRXDataNew;
static pt1Filter_t filteredCycleTimeState;
uint16_t filteredCycleTime;

extern pidControllerFuncPtr pid_controller;

void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    accelerometerConfig()->accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
    accelerometerConfig()->accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;

    saveConfigAndNotify();
}

#ifdef GTUNE

void updateGtuneState(void)
{
    static bool GTuneWasUsed = false;

    if (rcModeIsActive(BOXGTUNE)) {
        if (!FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            ENABLE_FLIGHT_MODE(GTUNE_MODE);
            init_Gtune();
            GTuneWasUsed = true;
        }
        if (!FLIGHT_MODE(GTUNE_MODE) && !ARMING_FLAG(ARMED) && GTuneWasUsed) {
            saveConfigAndNotify();
            GTuneWasUsed = false;
        }
    } else {
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            DISABLE_FLIGHT_MODE(GTUNE_MODE);
        }
    }
}
#endif

bool isCalibrating(void)
{
#ifdef BARO
    if (sensors(SENSOR_BARO) && !isBaroCalibrationComplete()) {
        return true;
    }
#endif

    // Note: compass calibration is handled completely differently, outside of the main loop, see f.CALIBRATE_MAG

    return (!isAccelerationCalibrationComplete() && sensors(SENSOR_ACC)) || (!isGyroCalibrationComplete());
}

/*
This function processes RX dependent coefficients when new RX commands are available
Those are: TPA, throttle expo
*/
static void updateRcCommands(void)
{
    int32_t prop2;

    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    if (rcData[THROTTLE] < currentControlRateProfile->tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
        } else {
            prop2 = 100 - currentControlRateProfile->dynThrPID;
        }
    }

    for (int axis = 0; axis < 3; axis++) {
        int32_t prop1;
        int32_t tmp = MIN(ABS(rcData[axis] - rxConfig()->midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (rcControlsConfig()->deadband) {
                if (tmp > rcControlsConfig()->deadband) {
                    tmp -= rcControlsConfig()->deadband;
                } else {
                    tmp = 0;
                }
            }

            rcCommand[axis] = rcLookupPitchRoll(tmp);
            prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * tmp / 500;
            prop1 = (uint16_t)prop1 * prop2 / 100;
            // non coupled PID reduction scaler used in PID controller 1 and PID controller 2. 100 means 100% of the pids
            PIDweight[axis] = prop2;
        } else {
            if (rcControlsConfig()->yaw_deadband) {
                if (tmp > rcControlsConfig()->yaw_deadband) {
                    tmp -= rcControlsConfig()->yaw_deadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = rcLookupYaw(tmp) * -rcControlsConfig()->yaw_control_direction;
            prop1 = 100 - (uint16_t)currentControlRateProfile->rates[axis] * ABS(tmp) / 500;
            // YAW TPA disabled.
            PIDweight[axis] = 100;
        }
#ifndef SKIP_PID_MW23
        // FIXME axis indexes into pids.  use something like lookupPidIndex(rc_alias_e alias) to reduce coupling.
        dynP8[axis] = (uint16_t)pidProfile()->P8[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)pidProfile()->I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)pidProfile()->D8[axis] * prop1 / 100;
#endif

        if (rcData[axis] < rxConfig()->midrc) {
            rcCommand[axis] = -rcCommand[axis];
        }
    }

    int32_t tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);       // [MINCHECK;2000] -> [0;1000]
    rcCommand[THROTTLE] = rcLookupThrottle(tmp);

    if (FLIGHT_MODE(HEADFREE_MODE)) {
        float radDiff = degreesToRadians(DECIDEGREES_TO_DEGREES(attitude.values.yaw) - headFreeModeHold);
        float cosDiff = cos_approx(radDiff);
        float sinDiff = sin_approx(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }
}

typedef enum {
    ARM_PREV_NONE       = 0,
    ARM_PREV_CLI        = 0x00205, //         0b1000000101  2 flashes - CLI active in the configurator
    ARM_PREV_FAILSAFE   = 0x00815, //       0b100000010101  3 flashes - Failsafe mode
    ARM_PREV_ANGLE      = 0x02055, //     0b10000001010101  4 flashes - Maximum arming angle exceeded
    ARM_PREV_CALIB      = 0x08155, //   0b1000000101010101  5 flashes - Calibration active
    ARM_PREV_OVERLOAD   = 0x20555  // 0b100000010101010101  6 flashes - System overload
} armingPreventedReason_e;

armingPreventedReason_e getArmingPreventionBlinkMask(void)
{
    if (isCalibrating()) {
        return ARM_PREV_CALIB;
    }
    if (rcModeIsActive(BOXFAILSAFE) || failsafePhase() == FAILSAFE_LANDED) {
        return ARM_PREV_FAILSAFE;
    }
    if (!imuIsAircraftArmable(armingConfig()->max_arm_angle)) {
        return ARM_PREV_ANGLE;
    }
    if (cliMode) {
        return ARM_PREV_CLI;
    }
    if (isSystemOverloaded()) {
        return ARM_PREV_OVERLOAD;
    }
    return ARM_PREV_NONE;
}

static void updateLEDs(void)
{
    if (ARMING_FLAG(ARMED)) {
        LED0_ON;
    } else {
        if (rcModeIsActive(BOXARM) == 0) {
            ENABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (!imuIsAircraftArmable(armingConfig()->max_arm_angle)) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (isCalibrating() || isSystemOverloaded()) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }

        uint32_t nextBlinkMask = getArmingPreventionBlinkMask();
        warningLedSetBlinkMask(nextBlinkMask);
        warningLedUpdate();
    }
}

void mwDisarm(void)
{
    if (ARMING_FLAG(ARMED)) {
        DISABLE_ARMING_FLAG(ARMED);

#ifdef BLACKBOX
        if (feature(FEATURE_BLACKBOX)) {
            finishBlackbox();
        }
#endif

        beeper(BEEPER_DISARMING);      // emit disarm tone
    }
}

#define TELEMETRY_FUNCTION_MASK (FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_SMARTPORT | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_MAVLINK)

void releaseSharedTelemetryPorts(void) {
    serialPort_t *sharedPort = findSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP_SERVER);
     while (sharedPort) {
         mspSerialReleasePortIfAllocated(sharedPort);
         sharedPort = findNextSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP_SERVER);
     }
}

void mwArm(void)
{
    if (ARMING_FLAG(OK_TO_ARM)) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }
        if (rcModeIsActive(BOXFAILSAFE)) {
            return;
        }
        if (!ARMING_FLAG(PREVENT_ARMING)) {
            ENABLE_ARMING_FLAG(ARMED);
            headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);

#ifdef BLACKBOX
            if (feature(FEATURE_BLACKBOX)) {
                serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP_SERVER);
                if (sharedBlackboxAndMspPort) {
                    mspSerialReleasePortIfAllocated(sharedBlackboxAndMspPort);
                }
                startBlackbox();
            }
#endif
            disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;   // start disarm timeout, will be extended when throttle is nonzero

            //beep to indicate arming
#ifdef GPS
            if (feature(FEATURE_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5)
                beeper(BEEPER_ARMING_GPS_FIX);
            else
                beeper(BEEPER_ARMING);
#else
            beeper(BEEPER_ARMING);
#endif

            return;
        }
    }

    if (!ARMING_FLAG(ARMED)) {
        beeperConfirmationBeeps(1);
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
            beeper(BEEPER_ACC_CALIBRATION);
        } else {
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
        }
    }
}

void updateInflightCalibrationState(void)
{
    if (AccInflightCalibrationArmed && ARMING_FLAG(ARMED) && rcData[THROTTLE] > rxConfig()->mincheck && !rcModeIsActive(BOXARM)) {   // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = false;
    }
    if (rcModeIsActive(BOXCALIB)) {      // Use the Calib Option to activate : Calib = TRUE measurement started, Land and Calib = 0 measurement stored
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
    if (ABS(rcCommand[YAW]) < 15 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -rcControlsConfig()->yaw_control_direction;
        if (STATE(SMALL_ANGLE))
            rcCommand[YAW] -= dif * pidProfile()->P8[PIDMAG] / 30;    // 18 deg
    } else
        magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}

void processRx(void)
{
    static bool armedBeeperOn = false;

    calculateRxChannelsAndUpdateFailsafe(currentTime);

    // in 3D mode, we need to be able to disarm by switch at any time
    if (feature(FEATURE_3D)) {
        if (!rcModeIsActive(BOXARM))
            mwDisarm();
    }

    updateRSSI(currentTime);

    if (feature(FEATURE_FAILSAFE)) {

        if (currentTime > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
            failsafeStartMonitoring();
        }

        failsafeUpdateState();
    }

    throttleStatus_e throttleStatus = calculateThrottleStatus(rxConfig(), rcControlsConfig()->deadband3d_throttle);
    rollPitchStatus_e rollPitchStatus =  calculateRollPitchCenterStatus(rxConfig());

    /* In airmode Iterm should be prevented to grow when Low thottle and Roll + Pitch Centered.
     This is needed to prevent Iterm winding on the ground, but keep full stabilisation on 0 throttle while in air
     Low Throttle + roll and Pitch centered is assuming the copter is on the ground. Done to prevent complex air/ground detections */
    if (throttleStatus == THROTTLE_LOW) {
        if (rcModeIsActive(BOXAIRMODE) && !failsafeIsActive() && ARMING_FLAG(ARMED)) {
            if (rollPitchStatus == CENTERED) {
                ENABLE_STATE(ANTI_WINDUP);
            } else {
                DISABLE_STATE(ANTI_WINDUP);
            }
        } else {
#ifndef SKIP_PID_MW23
            pidResetITermAngle();
#endif
            pidResetITerm();
        }
    } else {
        DISABLE_STATE(ANTI_WINDUP);
    }

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    if (ARMING_FLAG(ARMED)
        && feature(FEATURE_MOTOR_STOP)
        && !STATE(FIXED_WING)
    ) {
        if (isUsingSticksForArming()) {
            if (throttleStatus == THROTTLE_LOW) {
                if (armingConfig()->auto_disarm_delay != 0
                    && (int32_t)(disarmAt - millis()) < 0
                ) {
                    // auto-disarm configured and delay is over
                    mwDisarm();
                    armedBeeperOn = false;
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                // throttle is not low
                if (armingConfig()->auto_disarm_delay != 0) {
                    // extend disarm time
                    disarmAt = millis() + armingConfig()->auto_disarm_delay * 1000;
                }

                if (armedBeeperOn) {
                    beeperSilence();
                    armedBeeperOn = false;
                }
            }
        } else {
            // arming is via AUX switch; beep while throttle low
            if (throttleStatus == THROTTLE_LOW) {
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            } else if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    }

    processRcStickPositions(rxConfig(), throttleStatus, armingConfig()->retarded_arm, armingConfig()->disarm_kill_switch);

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        updateInflightCalibrationState();
    }

    rcModeUpdateActivated(modeActivationProfile()->modeActivationConditions);

    if (!cliMode) {
        updateAdjustmentStates(adjustmentProfile()->adjustmentRanges);
        processRcAdjustments(currentControlRateProfile, rxConfig());
    }

    bool canUseHorizonMode = true;

    if ((rcModeIsActive(BOXANGLE) || (feature(FEATURE_FAILSAFE) && failsafeIsActive())) && (sensors(SENSOR_ACC))) {
        // bumpless transfer to Level mode
        canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
#ifndef SKIP_PID_MW23
            pidResetITermAngle();
#endif
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
    }

    if (rcModeIsActive(BOXHORIZON) && canUseHorizonMode) {

        DISABLE_FLIGHT_MODE(ANGLE_MODE);

        if (!FLIGHT_MODE(HORIZON_MODE)) {
#ifndef SKIP_PID_MW23
            pidResetITermAngle();
#endif
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
        if (rcModeIsActive(BOXMAG)) {
            if (!FLIGHT_MODE(MAG_MODE)) {
                ENABLE_FLIGHT_MODE(MAG_MODE);
                magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            }
        } else {
            DISABLE_FLIGHT_MODE(MAG_MODE);
        }
        if (rcModeIsActive(BOXHEADFREE)) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
        if (rcModeIsActive(BOXHEADADJ)) {
            headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw); // acquire new heading
        }
    }
#endif

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        updateGpsWaypointsAndMode();
    }
#endif

    if (rcModeIsActive(BOXPASSTHRU)) {
        ENABLE_FLIGHT_MODE(PASSTHRU_MODE);
    } else {
        DISABLE_FLIGHT_MODE(PASSTHRU_MODE);
    }

    if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE) {
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
    }

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        if ((!telemetryConfig()->telemetry_switch && ARMING_FLAG(ARMED))
            || (telemetryConfig()->telemetry_switch && rcModeIsActive(BOXTELEMETRY))) {
            releaseSharedTelemetryPorts();
        } else {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspSerialAllocatePorts();
        }
    }
#endif

}

void filterRc(void){
    static int16_t lastCommand[4] = { 0, 0, 0, 0 };
    static int16_t deltaRC[4] = { 0, 0, 0, 0 };
    static int16_t factor, rcInterpolationFactor;
    uint16_t rxRefreshRate;

    // Set RC refresh rate for sampling and channels to filter
    initRxRefreshRate(&rxRefreshRate);

    rcInterpolationFactor = rxRefreshRate / filteredCycleTime + 1;

    if (isRXDataNew) {
        for (int channel=0; channel < 4; channel++) {
            deltaRC[channel] = rcCommand[channel] -  (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);
            lastCommand[channel] = rcCommand[channel];
        }

        isRXDataNew = false;
        factor = rcInterpolationFactor - 1;
    } else {
        factor--;
    }

    // Interpolate steps of rcCommand
    if (factor > 0) {
        for (int channel=0; channel < 4; channel++) {
            rcCommand[channel] = lastCommand[channel] - deltaRC[channel] * factor/rcInterpolationFactor;
         }
    } else {
        factor = 0;
    }
}

#if defined(BARO) || defined(SONAR)
static bool haveUpdatedRcCommandsOnce = false;
#endif

void taskMainPidLoop(void)
{
    cycleTime = getTaskDeltaTime(TASK_SELF);
    dT = (float)cycleTime * 0.000001f;

    // Calculate average cycle time and average jitter
    filteredCycleTime = pt1FilterApply4(&filteredCycleTimeState, cycleTime, 1, dT);

#ifdef DEBUG_CYCLE_TIME
    debug[0] = cycleTime;
    debug[1] = cycleTime - filteredCycleTime;
#endif

    imuUpdateGyroAndAttitude();

    updateRcCommands(); // this must be called here since applyAltHold directly manipulates rcCommands[]

    if (rxConfig()->rcSmoothing) {
        filterRc();
    }

#if defined(BARO) || defined(SONAR)
    haveUpdatedRcCommandsOnce = true;
#endif

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature) {
        gyro.temperature(&telemTemperature1);
    }

#ifdef MAG
        if (sensors(SENSOR_MAG)) {
            updateMagHold();
        }
#endif

#ifdef GTUNE
        updateGtuneState();
#endif

#if defined(BARO) || defined(SONAR)
        if (sensors(SENSOR_BARO) || sensors(SENSOR_SONAR)) {
            if (FLIGHT_MODE(BARO_MODE) || FLIGHT_MODE(SONAR_MODE)) {
                applyAltHold();
            }
        }
#endif

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rcData[THROTTLE] <= rxConfig()->mincheck
#ifndef USE_QUAD_MIXER_ONLY
#ifdef USE_SERVOS
                && !((mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI) && mixerConfig()->tri_unarmed_servo)
#endif
                && mixerConfig()->mixerMode != MIXER_AIRPLANE
                && mixerConfig()->mixerMode != MIXER_FLYING_WING
#endif
    ) {
        rcCommand[YAW] = 0;
    }

    if (throttleCorrectionConfig()->throttle_correction_value && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
        rcCommand[THROTTLE] += calculateThrottleAngleCorrection(throttleCorrectionConfig()->throttle_correction_value);
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
        pidProfile(),
        currentControlRateProfile,
        imuConfig()->max_angle_inclination,
        &accelerometerConfig()->accelerometerTrims,
        rxConfig()
    );

    mixTable();

#ifdef USE_SERVOS
    filterServos();
    writeServos();
#endif

    if (motorControlEnable) {
        writeMotors();
    }

#ifdef USE_SDCARD
        afatfs_poll();
#endif

#ifdef BLACKBOX
    if (!cliMode && feature(FEATURE_BLACKBOX)) {
        handleBlackbox();
    }
#endif
}

// Function for loop trigger
void taskMainPidLoopChecker(void) {
    // getTaskDeltaTime() returns delta time freezed at the moment of entering the scheduler. currentTime is freezed at the very same point.
    // To make busy-waiting timeout work we need to account for time spent within busy-waiting loop
    uint32_t currentDeltaTime = getTaskDeltaTime(TASK_SELF);

    if (imuConfig()->gyroSync) {
        while (1) {
            if (gyroSyncCheckUpdate() || ((currentDeltaTime + (micros() - currentTime)) >= (targetLooptime + GYRO_WATCHDOG_DELAY))) {
                break;
            }
        }
    }

    taskMainPidLoop();
}

void taskUpdateAccelerometer(void)
{
    imuUpdateAccelerometer(&accelerometerConfig()->accelerometerTrims);
}

void taskHandleSerial(void)
{
#ifdef USE_CLI
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }
#endif

    mspSerialProcess();
}

#ifdef BEEPER
void taskUpdateBeeper(void)
{
    beeperUpdate();          //call periodic beeper handler
}
#endif

void taskUpdateBattery(void)
{
    static uint32_t vbatLastServiced = 0;
    static uint32_t ibatLastServiced = 0;

    if (feature(FEATURE_VBAT)) {
        if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
            vbatLastServiced = currentTime;
            updateBattery();
        }
    }

    if (feature(FEATURE_CURRENT_METER)) {
        int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

        if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
            ibatLastServiced = currentTime;

            throttleStatus_e throttleStatus = calculateThrottleStatus(rxConfig(), rcControlsConfig()->deadband3d_throttle);

            switch(batteryConfig()->currentMeterType) {
                case CURRENT_SENSOR_ADC:
                    updateCurrentMeter(ibatTimeSinceLastServiced);
                    break;
                case CURRENT_SENSOR_VIRTUAL:
                    updateVirtualCurrentMeter(ibatTimeSinceLastServiced, throttleStatus);
                    break;
                default:
                    break;
            }
        }
    }
}

bool taskUpdateRxCheck(uint32_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);
    updateRx(currentTime);
    return shouldProcessRx(currentTime);
}

void taskUpdateRxMain(void)
{
    processRx();
    updateLEDs();

    isRXDataNew = true;

#ifdef BARO
    // updateRcCommands() sets rcCommand[], updateAltHoldState depends on valid rcCommand[] data.
    if (haveUpdatedRcCommandsOnce) {
        if (sensors(SENSOR_BARO)) {
            updateAltHoldState();
        }
    }
#endif

#ifdef SONAR
    // updateRcCommands() sets rcCommand[], updateAltHoldState depends on valid rcCommand[] data.
    if (haveUpdatedRcCommandsOnce) {
        if (sensors(SENSOR_SONAR)) {
            updateSonarAltHoldState();
        }
    }
#endif
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
        updateCompass(&sensorTrims()->magZero);
    }
}
#endif

#ifdef BARO
void taskUpdateBaro(void)
{
    if (sensors(SENSOR_BARO)) {
        uint32_t newDeadline = baroUpdate();
        rescheduleTask(TASK_SELF, newDeadline);
    }
}
#endif

#ifdef SONAR
void taskUpdateSonar(void)
{
    if (sensors(SENSOR_SONAR)) {
        sonarUpdate();
    }
}
#endif

#if defined(BARO) || defined(SONAR)
void taskCalculateAltitude(void)
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
        telemetryProcess(rcControlsConfig()->deadband3d_throttle);
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

#ifdef TRANSPONDER
void taskTransponder(void)
{
    if (feature(FEATURE_TRANSPONDER)) {
        updateTransponder();
    }
}
#endif
