#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/accgyro_common.h"
#include "drivers/light_ledring.h"
#include "drivers/light_led.h"
#include "drivers/gpio_common.h"
#include "drivers/system_common.h"
#include "drivers/serial_common.h"

#include "boardalignment.h"
#include "battery.h"
#include "buzzer.h"
#include "escservo.h"
#include "failsafe.h"
#include "flight_imu.h"
#include "flight_common.h"
#include "flight_mixer.h"
#include "gimbal.h"
#include "gps_common.h"
#include "sensors_common.h"
#include "sensors_sonar.h"
#include "sensors_compass.h"
#include "sensors_acceleration.h"
#include "sensors_barometer.h"
#include "sensors_gyro.h"
#include "serial_cli.h"
#include "serial_common.h"
#include "statusindicator.h"
#include "rc_controls.h"
#include "rc_curves.h"
#include "rx_common.h"
#include "telemetry_common.h"

#include "runtime_config.h"
#include "config.h"
#include "config_profile.h"
#include "config_master.h"

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

uint16_t rssi;                  // range: [0;1023]

extern uint8_t dynP8[3], dynI8[3], dynD8[3];
extern failsafe_t *failsafe;

typedef void (* pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination, rollAndPitchTrims_t *accelerometerTrims);

extern pidControllerFuncPtr pid_controller;

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
uint16_t InflightcalibratingA = 0;

void annexCode(void)
{
    static uint32_t calibratedAccTime;
    int32_t tmp, tmp2;
    int32_t axis, prop1, prop2;

    static uint8_t batteryWarningEnabled = false;

    static uint8_t vbatTimer = 0;

    // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < currentProfile.tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)currentProfile.dynThrPID * (rcData[THROTTLE] - currentProfile.tpa_breakpoint) / (2000 - currentProfile.tpa_breakpoint);
        } else {
            prop2 = 100 - currentProfile.dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - masterConfig.rxConfig.midrc), 500);
        if (axis != 2) {        // ROLL & PITCH
            if (currentProfile.deadband) {
                if (tmp > currentProfile.deadband) {
                    tmp -= currentProfile.deadband;
                } else {
                    tmp = 0;
                }
            }

            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t)currentProfile.controlRateConfig.rollPitchRate * tmp / 500;
            prop1 = (uint16_t)prop1 * prop2 / 100;
        } else {                // YAW
            if (currentProfile.yaw_deadband) {
                if (tmp > currentProfile.yaw_deadband) {
                    tmp -= currentProfile.yaw_deadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = tmp * -masterConfig.yaw_control_direction;
            prop1 = 100 - (uint16_t)currentProfile.controlRateConfig.yawRate * abs(tmp) / 500;
        }
        dynP8[axis] = (uint16_t)currentProfile.pidProfile.P8[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)currentProfile.pidProfile.I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)currentProfile.pidProfile.D8[axis] * prop1 / 100;
        if (rcData[axis] < masterConfig.rxConfig.midrc)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], masterConfig.rxConfig.mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - masterConfig.rxConfig.mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - masterConfig.rxConfig.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if (f.HEADFREE_MODE) {
        float radDiff = degreesToRadians(heading - headFreeModeHold);
        float cosDiff = cosf(radDiff);
        float sinDiff = sinf(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }

    if (feature(FEATURE_VBAT)) {
        if (!(++vbatTimer % VBATFREQ)) {
            updateBatteryVoltage();

            batteryWarningEnabled = shouldSoundBatteryAlarm();
        }
    }

    buzzer(batteryWarningEnabled); // external buzzer routine that handles buzzer events globally now

    if ((!isAccelerationCalibrationComplete() && sensors(SENSOR_ACC)) || (!isGyroCalibrationComplete())) {      // Calibration phasis
        LED0_TOGGLE;
    } else {
        if (f.ACC_CALIBRATED)
            LED0_OFF;
        if (f.ARMED)
            LED0_ON;

        checkTelemetryState();
    }

#ifdef LEDRING
    if (feature(FEATURE_LED_RING)) {
        static uint32_t LEDTime;
        if ((int32_t)(currentTime - LEDTime) >= 0) {
            LEDTime = currentTime + 50000;
            ledringState(f.ARMED, inclination.angle.pitchDeciDegrees, inclination.angle.rollDeciDegrees);
        }
    }
#endif

    if ((int32_t)(currentTime - calibratedAccTime) >= 0) {
        if (!f.SMALL_ANGLE) {
            f.ACC_CALIBRATED = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            calibratedAccTime = currentTime + 500000;
        } else {
            f.ACC_CALIBRATED = 1;
        }
    }

    handleSerial();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        handleTelemetry();
    }

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTime);
    }

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);
    else {
        // TODO MCU temp
    }
}

static void mwArm(void)
{
    if (isGyroCalibrationComplete() && f.ACC_CALIBRATED) {
        // TODO: feature(FEATURE_FAILSAFE) && failsafeCnt < 2
        // TODO: && ( !feature || ( feature && ( failsafecnt > 2) )
        if (!f.ARMED) {         // arm now!
            f.ARMED = 1;
            headFreeModeHold = heading;
        }
    } else if (!f.ARMED) {
        blinkLedAndSoundBeeper(2, 255, 1);
    }
}

static void mwVario(void)
{

}

void loop(void)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
    uint8_t stTmp = 0;
    int i;
    static uint32_t rcTime = 0;
#ifdef BARO
    static int16_t initialThrottleHold;
#endif
    static uint32_t loopTime;
    uint16_t auxState = 0;
    bool isThrottleLow = false;
    bool rcReady = false;

    // calculate rc stuff from serial-based receivers (spek/sbus)
    if (feature(FEATURE_SERIALRX)) {
        rcReady = isSerialRxFrameComplete(&masterConfig.rxConfig);
    }

    if (((int32_t)(currentTime - rcTime) >= 0) || rcReady) { // 50Hz or data driven
        rcReady = false;
        rcTime = currentTime + 20000;
        computeRC(&masterConfig.rxConfig, &rxRuntimeConfig);

        // in 3D mode, we need to be able to disarm by switch at any time
        if (feature(FEATURE_3D)) {
            if (!rcOptions[BOXARM])
                mwDisarm();
        }

        // Read value of AUX channel as rssi
        // 0 is disable, 1-4 is AUX{1..4}
        if (masterConfig.rssi_aux_channel > 0) {
            const int16_t rssiChannelData = rcData[AUX1 + masterConfig.rssi_aux_channel - 1];
            // Range of rssiChannelData is [1000;2000]. rssi should be in [0;1023];
            rssi = (uint16_t)((constrain(rssiChannelData - 1000, 0, 1000) / 1000.0f) * 1023.0f);
        }

        if (feature(FEATURE_FAILSAFE)) {
            failsafe->vTable->updateState();
        }

        // ------------------ STICKS COMMAND HANDLER --------------------
        // checking sticks positions
        for (i = 0; i < 4; i++) {
            stTmp >>= 2;
            if (rcData[i] > masterConfig.rxConfig.mincheck)
                stTmp |= 0x80;  // check for MIN
            if (rcData[i] < masterConfig.rxConfig.maxcheck)
                stTmp |= 0x40;  // check for MAX
        }
        if (stTmp == rcSticks) {
            if (rcDelayCommand < 250)
                rcDelayCommand++;
        } else
            rcDelayCommand = 0;
        rcSticks = stTmp;

        // perform actions
        if (feature(FEATURE_3D) && (rcData[THROTTLE] > (masterConfig.rxConfig.midrc - masterConfig.flight3DConfig.deadband3d_throttle) && rcData[THROTTLE] < (masterConfig.rxConfig.midrc + masterConfig.flight3DConfig.deadband3d_throttle)))
            isThrottleLow = true;
        else if (!feature(FEATURE_3D) && (rcData[THROTTLE] < masterConfig.rxConfig.mincheck))
            isThrottleLow = true;
        if (isThrottleLow) {
            resetErrorAngle();
            resetErrorGyro();
            if (currentProfile.activate[BOXARM] > 0) { // Arming/Disarming via ARM BOX
                if (rcOptions[BOXARM] && f.OK_TO_ARM)
                    mwArm();
                else if (f.ARMED)
                    mwDisarm();
            }
        }

        if (rcDelayCommand == 20) {
            if (f.ARMED) {      // actions during armed
                // Disarm on throttle down + yaw
                if (currentProfile.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE))
                    mwDisarm();
                // Disarm on roll (only when retarded_arm is enabled)
                if (masterConfig.retarded_arm && currentProfile.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO))
                    mwDisarm();
            } else {            // actions during not armed
                i = 0;
                // GYRO calibration
                if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
                    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
                    if (feature(FEATURE_GPS))
                        GPS_reset_home_position();
#ifdef BARO
                    if (sensors(SENSOR_BARO))
                        baroSetCalibrationCycles(10); // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
#endif
                    if (!sensors(SENSOR_MAG))
                        heading = 0; // reset heading to zero after gyro calibration
                // Inflight ACC Calibration
                } else if (feature(FEATURE_INFLIGHT_ACC_CAL) && (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI)) {
                    if (AccInflightCalibrationMeasurementDone) {        // trigger saving into eeprom after landing
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

                // Multiple configuration profiles
                if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO)          // ROLL left  -> Profile 1
                    i = 1;
                else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE)     // PITCH up   -> Profile 2
                    i = 2;
                else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)     // ROLL right -> Profile 3
                    i = 3;
                if (i) {
                    masterConfig.current_profile_index = i - 1;
                    writeEEPROM();
                    readEEPROM();
                    blinkLedAndSoundBeeper(2, 40, i);
                    // TODO alarmArray[0] = i;
                }

                // Arm via YAW
                if (currentProfile.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE))
                    mwArm();
                // Arm via ROLL
                else if (masterConfig.retarded_arm && currentProfile.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI))
                    mwArm();
                // Calibrating Acc
                else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
                // Calibrating Mag
                else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE)
                    f.CALIBRATE_MAG = 1;
                i = 0;
                // Acc Trim
                if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
                    currentProfile.accelerometerTrims.trims.pitch += 2;
                    i = 1;
                } else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
                    currentProfile.accelerometerTrims.trims.pitch -= 2;
                    i = 1;
                } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
                    currentProfile.accelerometerTrims.trims.roll += 2;
                    i = 1;
                } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
                    currentProfile.accelerometerTrims.trims.roll -= 2;
                    i = 1;
                }
                if (i) {
                    copyCurrentProfileToProfileSlot(masterConfig.current_profile_index);
                    writeEEPROM();
                    readEEPROMAndNotify();
                    rcDelayCommand = 0; // allow autorepetition
                }
            }
        }

        if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
            if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > masterConfig.rxConfig.mincheck && !rcOptions[BOXARM]) {   // Copter is airborne and you are turning it off via boxarm : start measurement
                InflightcalibratingA = 50;
                AccInflightCalibrationArmed = false;
            }
            if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
                if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
                    InflightcalibratingA = 50;
                    AccInflightCalibrationActive = true;
            } else if (AccInflightCalibrationMeasurementDone && !f.ARMED) {
                AccInflightCalibrationMeasurementDone = false;
                AccInflightCalibrationSavetoEEProm = true;
            }
        }

        // Check AUX switches
        for (i = 0; i < 4; i++)
            auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
        for (i = 0; i < CHECKBOX_ITEM_COUNT; i++)
            rcOptions[i] = (auxState & currentProfile.activate[i]) > 0;

        if ((rcOptions[BOXANGLE] || (feature(FEATURE_FAILSAFE) && failsafe->vTable->hasTimerElapsed())) && (sensors(SENSOR_ACC))) {
            // bumpless transfer to Level mode
            if (!f.ANGLE_MODE) {
                resetErrorAngle();
                f.ANGLE_MODE = 1;
            }
        } else {
            f.ANGLE_MODE = 0; // failsafe support
        }

        if (rcOptions[BOXHORIZON]) {
            f.ANGLE_MODE = 0;
            if (!f.HORIZON_MODE) {
                resetErrorAngle();
                f.HORIZON_MODE = 1;
            }
        } else {
            f.HORIZON_MODE = 0;
        }

        if ((rcOptions[BOXARM]) == 0)
            f.OK_TO_ARM = 1;
        if (f.ANGLE_MODE || f.HORIZON_MODE) {
            LED1_ON;
        } else {
            LED1_OFF;
        }

#ifdef BARO
        if (sensors(SENSOR_BARO)) {
            // Baro alt hold activate
            if (rcOptions[BOXBARO]) {
                if (!f.BARO_MODE) {
                    f.BARO_MODE = 1;
                    AltHold = EstAlt;
                    initialThrottleHold = rcCommand[THROTTLE];
                    errorAltitudeI = 0;
                    BaroPID = 0;
                }
            } else {
                f.BARO_MODE = 0;
            }
            // Vario signalling activate
            if (feature(FEATURE_VARIO)) {
                if (rcOptions[BOXVARIO]) {
                    if (!f.VARIO_MODE) {
                        f.VARIO_MODE = 1;
                    }
                } else {
                    f.VARIO_MODE = 0;
                }
            }
        }
#endif

#ifdef  MAG
        if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
            if (rcOptions[BOXMAG]) {
                if (!f.MAG_MODE) {
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            } else {
                f.MAG_MODE = 0;
            }
            if (rcOptions[BOXHEADFREE]) {
                if (!f.HEADFREE_MODE) {
                    f.HEADFREE_MODE = 1;
                }
            } else {
                f.HEADFREE_MODE = 0;
            }
            if (rcOptions[BOXHEADADJ]) {
                headFreeModeHold = heading; // acquire new heading
            }
        }
#endif

        if (sensors(SENSOR_GPS)) {
            updateGpsWaypointsAndMode();
        }

        if (rcOptions[BOXPASSTHRU]) {
            f.PASSTHRU_MODE = 1;
        } else {
            f.PASSTHRU_MODE = 0;
        }

        if (masterConfig.mixerConfiguration == MULTITYPE_FLYING_WING || masterConfig.mixerConfiguration == MULTITYPE_AIRPLANE) {
            f.HEADFREE_MODE = 0;
        }
    } else {                    // not in rc loop
        static int taskOrder = 0;    // never call all function in the same loop, to avoid high delay spikes
        switch (taskOrder) {
        case 0:
            taskOrder++;
#ifdef MAG
            if (sensors(SENSOR_MAG) && compassGetADC(&masterConfig.magZero))
                break;
#endif
        case 1:
            taskOrder++;
#ifdef BARO
            if (sensors(SENSOR_BARO) && baroUpdate(currentTime) != BAROMETER_ACTION_NOT_READY)
                break;
#endif
        case 2:
            taskOrder++;
#ifdef BARO
            if (sensors(SENSOR_BARO) && getEstimatedAltitude())
                break;
#endif
        case 3:
            // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
            // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
            // change this based on available hardware
            taskOrder++;
            if (feature(FEATURE_GPS)) {
                gpsThread();
                break;
            }
        case 4:
            taskOrder = 0;
#ifdef SONAR
            if (sensors(SENSOR_SONAR)) {
                Sonar_update();
            }
#endif
            if (feature(FEATURE_VARIO) && f.VARIO_MODE)
                mwVario();
            break;
        }
    }

    currentTime = micros();
    if (masterConfig.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + masterConfig.looptime;

        computeIMU();
        annexCode();
        // Measure loop rate just afer reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;

#ifdef MAG
        if (sensors(SENSOR_MAG)) {
            if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
                int16_t dif = heading - magHold;
                if (dif <= -180)
                    dif += 360;
                if (dif >= +180)
                    dif -= 360;
                dif *= -masterConfig.yaw_control_direction;
                if (f.SMALL_ANGLE)
                    rcCommand[YAW] -= dif * currentProfile.pidProfile.P8[PIDMAG] / 30;    // 18 deg
            } else
                magHold = heading;
        }
#endif

#ifdef BARO
        if (sensors(SENSOR_BARO)) {
            if (f.BARO_MODE) {
                static uint8_t isAltHoldChanged = 0;
                static int16_t AltHoldCorr = 0;
                if (!f.FIXED_WING) {
                    // multirotor alt hold
                    if (currentProfile.alt_hold_fast_change) {
                        // rapid alt changes
                        if (abs(rcCommand[THROTTLE] - initialThrottleHold) > currentProfile.alt_hold_throttle_neutral) {
                            errorAltitudeI = 0;
                            isAltHoldChanged = 1;
                            rcCommand[THROTTLE] += (rcCommand[THROTTLE] > initialThrottleHold) ? -currentProfile.alt_hold_throttle_neutral : currentProfile.alt_hold_throttle_neutral;
                        } else {
                            if (isAltHoldChanged) {
                                AltHold = EstAlt;
                                isAltHoldChanged = 0;
                            }
                            rcCommand[THROTTLE] = constrain(initialThrottleHold + BaroPID, masterConfig.escAndServoConfig.minthrottle + 100, masterConfig.escAndServoConfig.maxthrottle);
                        }
                    } else {
                        // slow alt changes for apfags
                        if (abs(rcCommand[THROTTLE] - initialThrottleHold) > currentProfile.alt_hold_throttle_neutral) {
                            // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
                            AltHoldCorr += rcCommand[THROTTLE] - initialThrottleHold;
                            AltHold += AltHoldCorr / 2000;
                            AltHoldCorr %= 2000;
                            isAltHoldChanged = 1;
                        } else if (isAltHoldChanged) {
                            AltHold = EstAlt;
                            AltHoldCorr = 0;
                            isAltHoldChanged = 0;
                        }
                        rcCommand[THROTTLE] = constrain(initialThrottleHold + BaroPID, masterConfig.escAndServoConfig.minthrottle + 100, masterConfig.escAndServoConfig.maxthrottle);
                    }
                } else {
                    // handle fixedwing-related althold. UNTESTED! and probably wrong
                    // most likely need to check changes on pitch channel and 'reset' althold similar to
                    // how throttle does it on multirotor
                    rcCommand[PITCH] += BaroPID * masterConfig.fixedwing_althold_dir;
                }
            }
        }
#endif

        if (currentProfile.throttle_correction_value && (f.ANGLE_MODE || f.HORIZON_MODE)) {
            rcCommand[THROTTLE] += throttleAngleCorrection;
        }

        if (sensors(SENSOR_GPS)) {
            if ((f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && f.GPS_FIX_HOME) {
                updateGpsStateForHomeAndHoldMode();
            }
        }

        // PID - note this is function pointer set by setPIDController()
        pid_controller(&currentProfile.pidProfile, &currentProfile.controlRateConfig, masterConfig.max_angle_inclination, &currentProfile.accelerometerTrims);

        mixTable();
        writeServos();
        writeMotors();
    }
}
