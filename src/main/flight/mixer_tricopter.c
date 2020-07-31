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

#include "platform.h"

#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/servos.h"

#ifdef USE_TRIFLIGHT

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "pg/adc.h"
#include "build/debug.h"
#include "pg/pg_ids.h"
#include "flight/pid.h"
#include "config/config.h"
#include "io/beeper.h"
#include "sensors/gyro.h"

#define GetCurrentDelay_ms(timestamp_ms) (now_ms - timestamp_ms)
#define GetCurrentTime_ms() (now_ms)
#define InitDelayMeasurement_ms() const uint32_t now_ms = millis()
#define IsDelayElapsed_ms(timestamp_ms, delay_ms) ((uint32_t) (now_ms - timestamp_ms) >= delay_ms)

PG_REGISTER_WITH_RESET_TEMPLATE(triflightConfig_t, triflightConfig, PG_TRIFLIGHT_CONFIG, 0);

PG_RESET_TEMPLATE(triflightConfig_t, triflightConfig,
                  .tri_motor_acc_yaw_correction = 6,
                  .tri_motor_acceleration       = 18,
                  .tri_servo_angle_at_max       = 400,
                  .tri_servo_feedback           = TRI_SERVO_FB_VIRTUAL,
                  .tri_servo_max_adc            = 0,
                  .tri_servo_mid_adc            = 0,
                  .tri_servo_min_adc            = 0,
                  .tri_tail_motor_thrustfactor  = 138,
                  .tri_tail_servo_speed         = 300,
                  .tri_yaw_boost                = 10,
                  .tri_tail_motor_index         = 0,
);

enum {
    DEBUG_TRI_MOTOR_CORRECTION = 0,
    DEBUG_TRI_TAIL_MOTOR = 1,
    DEBUG_TRI_YAW_ERROR = 2,
    DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE = 3,
};

UNIT_TESTED tailMotor_t tailMotor = {.virtualFeedBack = 1000.0f};
UNIT_TESTED tailServo_t tailServo = {.angle = TRI_TAIL_SERVO_ANGLE_MID, .ADCChannel = ADC_RSSI};
UNIT_TESTED tailTune_t tailTune = {.mode = TT_MODE_NONE};

// Tail motor correction per servo angle. Index 0 is angle TRI_CURVE_FIRST_INDEX_ANGLE.
float motorPitchCorrectionCurve[TRI_YAW_FORCE_CURVE_SIZE];
// Yaw output gain per servo angle. Index 0 is angle TRI_CURVE_FIRST_INDEX_ANGLE.
float yawOutputGainCurve[TRI_YAW_FORCE_CURVE_SIZE];
int8_t triServoDirection;

STATIC_UNIT_TESTED float getAngleForYawOutput(const float yawOutput);
STATIC_UNIT_TESTED uint16_t getLinearServoValue(const servoParam_t *const servoConf, const float scaledPIDOutput, const float pidSumLimit);
STATIC_UNIT_TESTED float getServoAngle(const servoParam_t * const servoConf, const uint16_t servoValue);
STATIC_UNIT_TESTED uint16_t getServoValueAtAngle(const servoParam_t *const servoConf, const float angle);
STATIC_UNIT_TESTED void tailTuneModeThrustTorque(thrustTorque_t *const pTT, const bool isThrottleHigh);

bool isRcAxisWithinDeadband(const int32_t axis, const uint8_t minDeadband);
float binarySearchOutput(const float yawOutput, const float motorWoPitchCorr);
float getPitchCorrectionAtTailAngle(const float angle, const float thrustFactor);

#ifndef UNIT_TEST
static uint32_t preventArmingFlags = 0;

static void checkArmingPrevent(void);
static void updateServoAngle(const float dT);
static void predictGyroOnDeceleration(void);
static void tailMotorStep(const int16_t setpoint, const float dT);
static void triTailTuneStep(servoParam_t *pServoConf, int16_t *pServoVal, float dT);
static float feedbackServoStep(const uint16_t tailServoADC);
static void preventArming(const triArmingPreventFlag_e flag, const bool enable);
static float virtualServoStep(float currentAngle, int16_t servoSpeed,
                              float dT, servoParam_t *servoConf,
                              uint16_t servoValue);

extern void tailTuneModeServoSetup(struct servoSetup_t *pSS, servoParam_t *pServoConf, int16_t *pServoVal, float dT);

#endif

// ======== Unit Tested Code ========

UNIT_TESTED float motorToThrust(const float motor) {
    return motor * (1.0f + (motor / tailMotor.outputRange) * 2.0f);
}

STATIC_UNIT_TESTED uint16_t getLinearServoValue(const servoParam_t *const servoConf, const float scaledPIDOutput, const float pidSumLimit) {
    // maxYawOutput is the maximum output we can get at zero motor output with the pitch correction
    const float yawOutput = tailServo.maxYawOutput * scaledPIDOutput / pidSumLimit;
    const float correctedAngle = getAngleForYawOutput(yawOutput);
    const uint16_t linearServoValue = getServoValueAtAngle(servoConf, correctedAngle);

    DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, correctedAngle * 10);

    return linearServoValue;
}

STATIC_UNIT_TESTED uint16_t getServoValueAtAngle(const servoParam_t *const servoConf, const float angle) {
    const float servoMid = servoConf->middle;
    uint16_t servoValue;
    if (angle == TRI_TAIL_SERVO_ANGLE_MID) {
        servoValue = servoMid;
    }
    else {
        const float angleRange = tailServo.maxDeflection;

        float angleDiff;

        int8_t servoRange; // -1 == min-mid, 1 == mid-max

        if (angle < TRI_TAIL_SERVO_ANGLE_MID) {
            angleDiff = TRI_TAIL_SERVO_ANGLE_MID - angle;
            if (triServoDirection > 0) {
                servoRange = -1;
            }
            else {
                servoRange = 1;
            }
        }
        else {
            angleDiff = angle - TRI_TAIL_SERVO_ANGLE_MID;
            if (triServoDirection > 0) {
                servoRange = 1;
            }
            else {
                servoRange = -1;
            }
        }
        if (servoRange < 0) {
            const float servoMin = servoConf->min;
            servoValue = servoMid - angleDiff * (servoMid - servoMin) / angleRange;
        }
        else {
            const float servoMax = servoConf->max;
            servoValue = ROUND(servoMid + angleDiff * (servoMax - servoMid) / angleRange);
        }
    }

    return servoValue;
}


STATIC_UNIT_TESTED float getAngleForYawOutput(const float yawOutput) {
    float angle;

    float motorWoPitchCorr = tailMotor.virtualFeedBack - tailMotor.minOutput - tailMotor.lastCorrection;

    motorWoPitchCorr = MAX(tailMotor.linearMinOutput, motorWoPitchCorr);

    if (yawOutput < (motorToThrust((motorWoPitchCorr + motorPitchCorrectionCurve[0])) * yawOutputGainCurve[0])) {
        // No force that low
        angle = tailServo.angleAtLinearMin;
    }
    else if (yawOutput > (motorToThrust((motorWoPitchCorr + motorPitchCorrectionCurve[TRI_YAW_FORCE_CURVE_SIZE - 1])) *
                          yawOutputGainCurve[TRI_YAW_FORCE_CURVE_SIZE - 1])) {
        // No force that high
        angle = tailServo.angleAtLinearMax;
    }
    else {
        // Binary search: yawForceCurve[lower] <= force, yawForceCurve[higher] > force
        angle = binarySearchOutput(yawOutput, motorWoPitchCorr);

        if (angle < tailServo.angleAtLinearMin) {
            angle = tailServo.angleAtLinearMin;
        }
        else if (angle > tailServo.angleAtLinearMax) {
            angle = tailServo.angleAtLinearMax;
        }
    }

    return angle;
}

STATIC_UNIT_TESTED float getServoAngle(const servoParam_t * const servoConf, const uint16_t servoValue) {
    const int16_t midValue = servoConf->middle;
    const int16_t endValue = servoValue < midValue ? servoConf->min : servoConf->max;
    const float endAngle = servoValue < midValue ? tailServo.angleAtMin : tailServo.angleAtMax;
    const float servoAngle = (float) (endAngle - TRI_TAIL_SERVO_ANGLE_MID) * (float) (servoValue - midValue)
                             / (endValue - midValue) + TRI_TAIL_SERVO_ANGLE_MID;

    return servoAngle;
}

STATIC_UNIT_TESTED void tailTuneModeThrustTorque(thrustTorque_t *const pTT, const bool isThrottleHigh) {
    InitDelayMeasurement_ms();
    switch (pTT->state) {
        case TT_IDLE:
            // Calibration has been requested, only start when throttle is up
            if (isThrottleHigh && ARMING_FLAG(ARMED)) {
                beeper(BEEPER_BAT_LOW);
                pTT->startBeepDelay_ms = 1000;
                pTT->timestamp_ms = GetCurrentTime_ms();
                pTT->timestamp2_ms = GetCurrentTime_ms();
                pTT->lastAdjTime_ms = GetCurrentTime_ms();
                pTT->state = TT_WAIT;
                pTT->servoAvgAngle.sum = 0;
                pTT->servoAvgAngle.numOf = 0;
                pTT->tailTuneGyroLimit = 4.5f;
            }
            break;
        case TT_WAIT:
            if (isThrottleHigh && ARMING_FLAG(ARMED)) {
                // Wait for 5 seconds before activating the tuning.
                // This is so that pilot has time to take off if the tail tune mode was activated on ground.
                if (IsDelayElapsed_ms(pTT->timestamp_ms, 5000)) {
                    DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1011);
                    // Longer beep when starting
                    beeper(BEEPER_BAT_CRIT_LOW);
                    pTT->state = TT_ACTIVE;
                    pTT->timestamp_ms = GetCurrentTime_ms();
                }
                else if (IsDelayElapsed_ms(pTT->timestamp_ms, pTT->startBeepDelay_ms)) {
                    DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1010);
                    // Beep every second until start
                    beeper(BEEPER_BAT_LOW);
                    pTT->startBeepDelay_ms += 1000;
                }
            }
            else {
                pTT->state = TT_IDLE;
            }
            break;
        case TT_ACTIVE:
            if (!(isThrottleHigh
                  && isRcAxisWithinDeadband(ROLL, TRI_TAIL_TUNE_MIN_DEADBAND)
                  && isRcAxisWithinDeadband(PITCH, TRI_TAIL_TUNE_MIN_DEADBAND)
                  && isRcAxisWithinDeadband(YAW, TRI_TAIL_TUNE_MIN_DEADBAND))) {
                pTT->timestamp_ms = GetCurrentTime_ms(); // sticks are NOT good
                DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1100);
            }
#ifndef UNIT_TEST
            if (ABS(gyro.gyroADCf[FD_YAW]) > pTT->tailTuneGyroLimit) {
                pTT->timestamp2_ms = GetCurrentTime_ms(); // gyro is NOT stable
                DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1200);
            }
#endif
            if (IsDelayElapsed_ms(pTT->timestamp_ms, 200)) {
                DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1300);
                DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1300);
                // RC commands have been within deadbands for 250 ms
                if (IsDelayElapsed_ms(pTT->timestamp2_ms, 200)) {
                    DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1400);
                    // Gyro has also been stable for 250 ms
                    if (IsDelayElapsed_ms(pTT->lastAdjTime_ms, 20)) {
                        pTT->lastAdjTime_ms = GetCurrentTime_ms();
                        pTT->servoAvgAngle.sum += triGetCurrentServoAngle();
                        pTT->servoAvgAngle.numOf++;
                        DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1600 + pTT->servoAvgAngle.numOf / 10);
                        if ((pTT->servoAvgAngle.numOf & 0x1f) == 0x00) {
                            // once every 32 times
                            beeperConfirmationBeeps(1);
                        }
                        if (pTT->servoAvgAngle.numOf >= 300) {
                            beeper(BEEPER_READY_BEEP);
                            pTT->state = TT_WAIT_FOR_DISARM;
                            pTT->timestamp_ms = GetCurrentTime_ms();
                        }
                    }
                }
                else if (IsDelayElapsed_ms(pTT->lastAdjTime_ms, 300)) {
                    // Sticks are OK but there has not been any valid samples in 1 s, try to loosen the gyro criteria a little
                    pTT->tailTuneGyroLimit += 0.1f;
                    pTT->lastAdjTime_ms = GetCurrentTime_ms();
                    if (pTT->tailTuneGyroLimit > 10.0f) {
                        // If there are not enough samples by now it is a fail.
                        pTT->state = TT_FAIL;
                    }
                    DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1500 + pTT->tailTuneGyroLimit * 10);
                }
            }
            else {
                DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1250);
            }
            break;
        case TT_WAIT_FOR_DISARM: DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1700 + pTT->tailTuneGyroLimit * 10);
            if (!ARMING_FLAG(ARMED)) {
                float averageServoAngle = pTT->servoAvgAngle.sum / pTT->servoAvgAngle.numOf;
                if (averageServoAngle > 90.5f && averageServoAngle < 120.f) {
                    averageServoAngle -= 90.0f;
                    averageServoAngle *= RAD;
                    triflightConfigMutable()->tri_tail_motor_thrustfactor = 10.0f * cos_approx(averageServoAngle) /
                                                                            sin_approx(averageServoAngle);
                    saveConfigAndNotify();

                    pTT->state = TT_DONE;
                }
                else {
                    pTT->state = TT_FAIL;
                }
                pTT->timestamp_ms = GetCurrentTime_ms();
            }
            else {
                if (IsDelayElapsed_ms(pTT->timestamp_ms, 2000)) {
                    beeper(BEEPER_READY_BEEP);
                    pTT->timestamp_ms = GetCurrentTime_ms();
                }
            }
            break;
        case TT_DONE: DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1800);
            if (IsDelayElapsed_ms(pTT->timestamp_ms, 2000)) {
                beeper(BEEPER_READY_BEEP);
                pTT->timestamp_ms = GetCurrentTime_ms();
            }
            break;
        case TT_FAIL: DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1850);
            if (IsDelayElapsed_ms(pTT->timestamp_ms, 2000)) {
                beeper(BEEPER_ACC_CALIBRATION_FAIL);
                pTT->timestamp_ms = GetCurrentTime_ms();
            }
            break;
    }
}

// ======== Public Interface ========

#ifndef UNIT_TEST

bool triIsEnabledServoUnarmed(void) {
    const bool isEnabledServoUnarmed = (servoConfig()->tri_unarmed_servo != 0) || FLIGHT_MODE(TAILTUNE_MODE);
    return isEnabledServoUnarmed;
}

void triServoMixer(const float scaledYawPid, const float pidSumLimit, const float dT) {
    // Update the tail motor speed from feedback
    tailMotorStep(motor[triflightConfig()->tri_tail_motor_index], dT);

    // Update the servo angle from feedback
    updateServoAngle(dT);

    // Correct the servo output to produce linear yaw output
    *tailServo.pOutput = getLinearServoValue(tailServo.pConf, scaledYawPid, pidSumLimit);

    // Run tail tune mode
    triTailTuneStep(tailServo.pConf, tailServo.pOutput, dT);

    // Check for tail motor deceleration and determine expected produced yaw error
    predictGyroOnDeceleration();

    checkArmingPrevent();
}

#endif

// ======== Internal Code Shared with Init ========

float triGetCurrentServoAngle(void) {
    return tailServo.angle;
}

int16_t triGetMotorCorrection(const uint8_t motorIndex) {
    uint16_t correction = 0;

    if (motorIndex == triflightConfig()->tri_tail_motor_index) {
        // TODO: revise this function, is the speed up lag really needed? Test without.

        // Adjust tail motor speed based on servo angle. Check how much to adjust speed from pitch force curve based on servo angle.
        // Take motor speed up lag into account by shifting the phase of the curve
        // Not taking into account the motor braking lag (yet)
        const float servoAngle = triGetCurrentServoAngle();
        correction = getPitchCorrectionAtTailAngle(DEGREES_TO_RADIANS(servoAngle), tailServo.thrustFactor);

        // Multiply the correction to get more authority (yaw boost)
        if (isAirmodeActivated() && tailServo.feedbackHealthy) {
            correction *= tailMotor.pitchCorrectionGain;
        }
        tailMotor.lastCorrection = correction;

        DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_MOTOR_CORRECTION, 1000 + correction);
    }

    return correction;
}

bool triIsServoSaturated(const float rateError) {
    if (ABS(rateError) > TRI_SERVO_SATURATED_GYRO_ERROR) {
        return true;
    }
    else {
        return false;
    }
}

float getPitchCorrectionAtTailAngle(const float angle, const float thrustFactor) {
    const float pitchCorrection = 1.0f / (sin_approx(angle) - cos_approx(angle) / thrustFactor);
    const float motorCorrection = (tailMotor.outputRange * pitchCorrection) - tailMotor.outputRange;
    return motorCorrection;
}

float binarySearchOutput(const float yawOutput, const float motorWoPitchCorr) {
    int32_t lower = 0;
    int32_t higher = TRI_YAW_FORCE_CURVE_SIZE - 1;

    while (higher > lower + 1) {
        const int32_t mid = (lower + higher) / 2;
        if ((motorToThrust(motorWoPitchCorr + motorPitchCorrectionCurve[mid]) * yawOutputGainCurve[mid]) > yawOutput) {
            higher = mid;
        }
        else {
            lower = mid;
        }
    }

    // Interpolating
    const float outputLow = (motorToThrust(motorWoPitchCorr + motorPitchCorrectionCurve[lower])) * yawOutputGainCurve[lower];
    const float outputHigh = (motorToThrust(motorWoPitchCorr + motorPitchCorrectionCurve[higher])) * yawOutputGainCurve[higher];
    const float angleLow = TRI_CURVE_FIRST_INDEX_ANGLE + lower;
    const float angle = angleLow + (yawOutput - outputLow) / (outputHigh - outputLow);

    return angle;
}

bool isRcAxisWithinDeadband(const int32_t axis, const uint8_t minDeadband) {
    const int32_t tmp = MIN(ABS(rcCommand[axis]), 500);
    bool ret = false;
    uint8_t deadband = minDeadband;

#ifndef UNIT_TEST
    if (axis == YAW) {
        deadband = rcControlsConfig()->yaw_deadband;
    }
    else {
        deadband = rcControlsConfig()->deadband;
    }
    deadband = MAX(minDeadband, deadband);
#endif

    if (tmp <= deadband) {
        ret = true;
    }

    return ret;
}

// ======== Static Non-Unit Testable Code ========

#ifndef UNIT_TEST

static void preventArming(const triArmingPreventFlag_e flag, const bool enable) {
    if (enable) {
        preventArmingFlags |= flag;
    }
    else {
        preventArmingFlags &= ~flag;
    }
}

static void triTailTuneStep(servoParam_t *pServoConf, int16_t *pServoVal, float dT) {
    if (!IS_RC_MODE_ACTIVE(BOXTAILTUNE)) {
        if (FLIGHT_MODE(TAILTUNE_MODE)) {
            preventArming(TRI_ARMING_PREVENT_FLAG_UNARMED_TAIL_TUNE, false);
            DISABLE_FLIGHT_MODE(TAILTUNE_MODE);
            tailTune.mode = TT_MODE_NONE;
        }
    }
    else {
        ENABLE_FLIGHT_MODE(TAILTUNE_MODE);
        if (tailTune.mode == TT_MODE_NONE) {
            if (ARMING_FLAG(ARMED)) {
                tailTune.mode = TT_MODE_THRUST_TORQUE;
                tailTune.tt.state = TT_IDLE;
            }
            else {
                // Prevent accidental arming in servo setup mode
                preventArming(TRI_ARMING_PREVENT_FLAG_UNARMED_TAIL_TUNE, true);
                tailTune.mode = TT_MODE_SERVO_SETUP;
                tailTune.ss.servoVal = pServoConf->middle;
            }
        }
        switch (tailTune.mode) {
            case TT_MODE_THRUST_TORQUE:
                tailTuneModeThrustTorque(&tailTune.tt, (THROTTLE_HIGH == calculateThrottleStatus()));
                break;
            case TT_MODE_SERVO_SETUP:
                tailTuneModeServoSetup(&tailTune.ss, pServoConf, pServoVal, dT);
                break;
            case TT_MODE_NONE:
                break;
        }
    }
}

static void predictGyroOnDeceleration(void) {
    static float previousMotorSpeed = 1000.0f;

    if (triflightConfig()->tri_motor_acc_yaw_correction > 0) {
        const float tailMotorSpeed = tailMotor.virtualFeedBack;
        // Calculate how much the motor speed changed since last time
        float acceleration = (tailMotorSpeed - previousMotorSpeed);

        previousMotorSpeed = tailMotorSpeed;
        float error = 0;
        if (acceleration < 0.0f) {
            // Tests have shown that this is mostly needed when throttle is cut (motor decelerating), so only
            // set the expected gyro error in that case.
            // Set the expected axis error based on tail motor acceleration and configured gain
            error = acceleration * triflightConfig()->tri_motor_acc_yaw_correction * 10.0f;
            error *= sin_approx(DEGREES_TO_RADIANS(triGetCurrentServoAngle()));
        }

        DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_YAW_ERROR, 1000 + error);

        pidSetPredictedError(FD_YAW, error);
    }
}

static void checkArmingPrevent(void) {
    if (preventArmingFlags) {
        ENABLE_ARMING_FLAG(ARMING_DISABLED_TRIFLIGHT);
    }
    else {
        DISABLE_ARMING_FLAG(ARMING_DISABLED_TRIFLIGHT);
    }
}

static void tailMotorStep(const int16_t setpoint, const float dT) {
    static float current = 1000;

    const float dS = dT * tailMotor.acceleration; // Max change of an speed since last check

    if (ABS(current - setpoint) < dS) {
        // At set-point after this moment
        current = setpoint;
    }
    else if (current < setpoint) {
        current += dS;
    }
    else {
        current -= dS;
    }

    // Use a PT1 low-pass filter to add "slowness" to the virtual motor feedback.
    // Cut-off to delay:
    // 2  Hz -> 25 ms
    // 5  Hz -> 14 ms
    // 10 Hz -> 9  ms

    tailMotor.virtualFeedBack = pt1FilterApply(&tailMotor.feedbackFilter, current);

    DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_TAIL_MOTOR, tailMotor.virtualFeedBack);
}


static void updateServoAngle(const float dT) {
    if (triflightConfig()->tri_servo_feedback == TRI_SERVO_FB_VIRTUAL) {
        tailServo.angle = virtualServoStep(tailServo.angle, tailServo.speed, dT, tailServo.pConf, *tailServo.pOutput);
    }
    else {
        // Read new servo feedback signal sample and run it through filter
        const uint16_t ADCRaw = pt1FilterApply(&tailServo.feedbackFilter, adcGetChannel(tailServo.ADCChannel));
        tailServo.angle = feedbackServoStep(ADCRaw);
        tailServo.ADCRaw = ADCRaw;
    }

    if ((tailServo.angle < (TRI_TAIL_SERVO_INVALID_ANGLE_MIN)) ||
        (tailServo.angle > (TRI_TAIL_SERVO_INVALID_ANGLE_MAX))) {
        tailServo.feedbackHealthy = false;
        preventArming(TRI_ARMING_PREVENT_FLAG_INVALID_SERVO_ANGLE, true);
    }
    else {
        tailServo.feedbackHealthy = true;
        preventArming(TRI_ARMING_PREVENT_FLAG_INVALID_SERVO_ANGLE, false);
    }
}

static float virtualServoStep(float currentAngle, int16_t servoSpeed, float dT, servoParam_t *servoConf, uint16_t servoValue) {
    const float angleSetPoint = getServoAngle(servoConf, servoValue);
    const float dA = dT * servoSpeed; // Max change of an angle since last check

    if (ABS(currentAngle - angleSetPoint) < dA) {
        // At set-point after this moment
        currentAngle = angleSetPoint;
    }
    else if (currentAngle < angleSetPoint) {
        currentAngle += dA;
    }
    else {
        // tailServoAngle.virtual > angleSetPoint
        currentAngle -= dA;
    }

    return currentAngle;
}

static float feedbackServoStep(const uint16_t tailServoADC) {
    // Feedback servo
    const int32_t ADCFeedback = tailServoADC;
    const int16_t midValue = triflightConfig()->tri_servo_mid_adc;
    const int16_t endValue = ADCFeedback < midValue ? triflightConfig()->tri_servo_min_adc :
                             triflightConfig()->tri_servo_max_adc;
    const float tailServoMaxAngle = tailServo.maxDeflection;
    const float endAngle = ADCFeedback < midValue ? TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle :
                           TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle;
    const float currentAngle = (endAngle - TRI_TAIL_SERVO_ANGLE_MID) * (ADCFeedback - midValue) /
                               (endValue - midValue) + TRI_TAIL_SERVO_ANGLE_MID;

    return currentAngle;
}

#endif // ifndef UNIT_TEST

#else // !USE_TRIFLIGHT

bool triIsEnabledServoUnarmed(void) {
    return (servoConfig()->tri_unarmed_servo != 0);
}

#endif // USE_TRIFLIGHT
