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
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <fc/core.h>

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "pg/pg_ids.h"

#include "drivers/time.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"
#include "flight/servos.h"

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

STATIC_UNIT_TESTED tailMotor_t tailMotor = {.virtualFeedBack = 1000.0f};
STATIC_UNIT_TESTED tailServo_t tailServo = {.angle = TRI_TAIL_SERVO_ANGLE_MID, .ADCChannel = ADC_RSSI};
STATIC_UNIT_TESTED tailTune_t tailTune = {.mode = TT_MODE_NONE};

// Tail motor correction per servo angle. Index 0 is angle TRI_CURVE_FIRST_INDEX_ANGLE.
static float motorPitchCorrectionCurve[TRI_YAW_FORCE_CURVE_SIZE];
// Yaw output gain per servo angle. Index 0 is angle TRI_CURVE_FIRST_INDEX_ANGLE.
static float yawOutputGainCurve[TRI_YAW_FORCE_CURVE_SIZE];
static int8_t triServoDirection;

STATIC_UNIT_TESTED float getAngleForYawOutput(float yawOutput);
STATIC_UNIT_TESTED uint16_t getLinearServoValue(servoParam_t *servoConf, float scaledPIDOutput, float pidSumLimit);
STATIC_UNIT_TESTED float getServoAngle(servoParam_t *servoConf, uint16_t servoValue);
STATIC_UNIT_TESTED uint16_t getServoValueAtAngle(servoParam_t *servoConf, float angle);
STATIC_UNIT_TESTED void tailTuneModeThrustTorque(thrustTorque_t *pTT, const bool isThrottleHigh);

static float binarySearchOutput(float yawOutput, float motorWoPitchCorr);
static bool isRcAxisWithinDeadband(int32_t axis, uint8_t minDeadband);
static float getPitchCorrectionAtTailAngle(float angle, float thrustFactor);
static AdcChannel getServoFeedbackADCChannel(uint8_t tri_servo_feedback);
static void initYawForceCurve(void);
static int8_t triGetServoDirection(void);

#ifndef UNIT_TEST
static uint32_t preventArmingFlags = 0;

static void checkArmingPrevent(void);
static void updateServoAngle(float dT);
static void predictGyroOnDeceleration(void);
static void tailMotorStep(int16_t setpoint, float dT);
static void tailTuneModeServoSetup(struct servoSetup_t *pSS, servoParam_t *pServoConf, int16_t *pServoVal, float dT);
static void triTailTuneStep(servoParam_t *pServoConf, int16_t *pServoVal, float dT);
static float feedbackServoStep(uint16_t tailServoADC);
static void preventArming(triArmingPreventFlag_e flag, bool enable);
static float virtualServoStep(float currentAngle, int16_t servoSpeed,
                              float dT, servoParam_t *servoConf,
                              uint16_t servoValue);
#endif

// ======== Unit Tested Code ========

STATIC_UNIT_TESTED float motorToThrust(float motor) {
    return motor * (1.0f + (motor / tailMotor.outputRange) * 2.0f);
}

STATIC_UNIT_TESTED uint16_t getLinearServoValue(servoParam_t *servoConf, float scaledPIDOutput, float pidSumLimit) {
    // maxYawOutput is the maximum output we can get at zero motor output with the pitch correction
    const float yawOutput = tailServo.maxYawOutput * scaledPIDOutput / pidSumLimit;
    const float correctedAngle = getAngleForYawOutput(yawOutput);
    const uint16_t linearServoValue = getServoValueAtAngle(servoConf, correctedAngle);

    DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, correctedAngle * 10);

    return linearServoValue;
}

STATIC_UNIT_TESTED uint16_t getServoValueAtAngle(servoParam_t *servoConf, float angle) {
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

            servoValue = lroundf(servoMid + angleDiff * (servoMax - servoMid) / angleRange);
        }
    }

    return servoValue;
}


STATIC_UNIT_TESTED float getAngleForYawOutput(float yawOutput) {
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

STATIC_UNIT_TESTED float getServoAngle(servoParam_t *servoConf, uint16_t servoValue) {
    const int16_t midValue = servoConf->middle;
    const int16_t endValue = servoValue < midValue ? servoConf->min : servoConf->max;
    const float endAngle = servoValue < midValue ? tailServo.angleAtMin : tailServo.angleAtMax;
    const float servoAngle = (float) (endAngle - TRI_TAIL_SERVO_ANGLE_MID) * (float) (servoValue - midValue)
                             / (endValue - midValue) + TRI_TAIL_SERVO_ANGLE_MID;

    return servoAngle;
}

STATIC_UNIT_TESTED void tailTuneModeThrustTorque(thrustTorque_t *pTT, const bool isThrottleHigh) {
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
            if (fabsf(gyro.gyroADCf[FD_YAW]) > pTT->tailTuneGyroLimit) {
                pTT->timestamp2_ms = GetCurrentTime_ms(); // gyro is NOT stable
                DEBUG_SET(DEBUG_TRIFLIGHT, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1200);
            }
            if (IsDelayElapsed_ms(pTT->timestamp_ms, 200)) {
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
                beeper(BEEPER_ACTION_FAIL);
                pTT->timestamp_ms = GetCurrentTime_ms();
            }
            break;
    }
}

// ======== Public Interface ========

void triInitMixer(servoParam_t *pTailServoConfig, int16_t *pTailServoOutput) {
    tailServo.pConf = pTailServoConfig;
    tailServo.pOutput = pTailServoOutput;
    tailServo.thrustFactor = triflightConfig()->tri_tail_motor_thrustfactor / 10.0f;
    tailServo.maxDeflection = triflightConfig()->tri_servo_angle_at_max / 10.0f;
    tailServo.angleAtMin = TRI_TAIL_SERVO_ANGLE_MID - tailServo.maxDeflection;
    tailServo.angleAtMax = TRI_TAIL_SERVO_ANGLE_MID + tailServo.maxDeflection;
    tailServo.speed = triflightConfig()->tri_tail_servo_speed / 10.0f;
    tailServo.ADCChannel = getServoFeedbackADCChannel(triflightConfig()->tri_servo_feedback);

    tailMotor.outputRange = mixGetMotorOutputHigh() - mixGetMotorOutputLow();
    tailMotor.minOutput = mixGetMotorOutputLow();
    tailMotor.linearMinOutput = tailMotor.outputRange * 0.05;
    tailMotor.pitchCorrectionGain = triflightConfig()->tri_yaw_boost / 100.0f;
    tailMotor.acceleration = (float) tailMotor.outputRange / triflightConfig()->tri_motor_acceleration;

    initYawForceCurve();

    triServoDirection = triGetServoDirection();
}

#ifndef UNIT_TEST

void triInitFilters(void) {
    const float dT = pidGetDT();
    pt1FilterInit(&tailMotor.feedbackFilter, pt1FilterGain(TRI_MOTOR_FEEDBACK_LPF_CUTOFF_HZ, dT));
    pt1FilterInit(&tailServo.feedbackFilter, pt1FilterGain(TRI_SERVO_FEEDBACK_LPF_CUTOFF_HZ, dT));
}

bool triIsEnabledServoUnarmed(void) {
    const bool isEnabledServoUnarmed = (servoConfig()->tri_unarmed_servo != 0) || FLIGHT_MODE(TAILTUNE_MODE);

    return isEnabledServoUnarmed;
}

void triServoMixer(float scaledYawPid, float pidSumLimit, float dT) {
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

// ======== Static Code ========


static void initYawForceCurve(void) {
    // DERIVATE(1/(sin(x)-cos(x)/tailServoThrustFactor)) = 0
    // Multiplied by 10 to get decidegrees
    const int16_t minAngle = tailServo.angleAtMin;
    const int16_t maxAngle = tailServo.angleAtMax;

    float maxNegForce = 0;
    float maxPosForce = 0;

    int16_t angle = TRI_CURVE_FIRST_INDEX_ANGLE;

    for (int32_t i = 0; i < TRI_YAW_FORCE_CURVE_SIZE; i++) {
        const float angleRad = DEGREES_TO_RADIANS(angle);

        yawOutputGainCurve[i] = -tailServo.thrustFactor * cosf(angleRad) - sinf(angleRad);

        motorPitchCorrectionCurve[i] = tailMotor.pitchCorrectionGain * getPitchCorrectionAtTailAngle(angleRad, tailServo.thrustFactor);

        // Only calculate the top forces in the configured angle range
        if ((angle >= minAngle) && (angle <= maxAngle)) {
            maxNegForce = MIN(yawOutputGainCurve[i], maxNegForce);
            maxPosForce = MAX(yawOutputGainCurve[i], maxPosForce);
        }
        angle++;
    }

    float minLinearAngle;
    float maxLinearAngle;

    const bool tailCCW = ABS(maxPosForce) < ABS(maxNegForce);

    if (tailCCW || true) { // Only support CCW tail for the time being
        const float maxOutput = ABS(maxPosForce);

        maxLinearAngle = maxAngle;

        const uint32_t indexMax = TRI_YAW_FORCE_CURVE_SIZE - 1;

        tailServo.maxYawOutput = maxOutput * motorToThrust(motorPitchCorrectionCurve[indexMax]);

        minLinearAngle = binarySearchOutput(-tailServo.maxYawOutput, 0);
    }
    else {
        // This would be the case if tail motor is spinning CW
        // Not supported yet
    }

    tailServo.angleAtLinearMin = minLinearAngle;
    tailServo.angleAtLinearMax = maxLinearAngle;
}

float triGetCurrentServoAngle(void) {
    return tailServo.angle;
}

int16_t triGetMotorCorrection(uint8_t motorIndex) {
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

bool triIsServoSaturated(float rateError) {
    if (fabsf(rateError) > TRI_SERVO_SATURATED_GYRO_ERROR) {
        return true;
    }
    else {
        return false;
    }
}


static float getPitchCorrectionAtTailAngle(float angle, float thrustFactor) {
    const float pitchCorrection = 1.0f / (sin_approx(angle) - cos_approx(angle) / thrustFactor);
    const float motorCorrection = (tailMotor.outputRange * pitchCorrection) - tailMotor.outputRange;

    return motorCorrection;
}

static float binarySearchOutput(float yawOutput, float motorWoPitchCorr) {
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

static AdcChannel getServoFeedbackADCChannel(uint8_t tri_servo_feedback) {
    AdcChannel channel;
    switch (tri_servo_feedback) {
        case TRI_SERVO_FB_RSSI:
            channel = ADC_RSSI;
            break;
        case TRI_SERVO_FB_CURRENT:
            channel = ADC_CURRENT;
            break;
#ifdef EXTERNAL1_ADC_PIN
            case TRI_SERVO_FB_EXT1:
                channel = ADC_EXTERNAL1;
                break;
#endif
        default:
            channel = ADC_RSSI;
            break;
    }

    return channel;
}

static int8_t triGetServoDirection(void) {
    const int8_t direction = (int8_t) servoDirection(SERVO_RUDDER, INPUT_STABILIZED_YAW);

    return direction;
}

static bool isRcAxisWithinDeadband(int32_t axis, uint8_t minDeadband) {
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

static void preventArming(triArmingPreventFlag_e flag, bool enable) {
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

static void tailMotorStep(int16_t setpoint, float dT) {
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

static void tailTuneModeServoSetup(struct servoSetup_t *pSS, servoParam_t *pServoConf, int16_t *pServoVal, float dT) {
    InitDelayMeasurement_ms();
    // Check mode select
    if (isRcAxisWithinDeadband(PITCH, TRI_TAIL_TUNE_MIN_DEADBAND) && (rcCommand[ROLL] < -100)) {
        pSS->servoVal = pServoConf->min;
        pSS->pLimitToAdjust = &pServoConf->min;
        beeperConfirmationBeeps(1);
        pSS->state = SS_SETUP;
    }
    else if (isRcAxisWithinDeadband(ROLL, TRI_TAIL_TUNE_MIN_DEADBAND) && (rcCommand[PITCH] > 100)) {
        pSS->servoVal = pServoConf->middle;
        pSS->pLimitToAdjust = &pServoConf->middle;
        beeperConfirmationBeeps(2);
        pSS->state = SS_SETUP;
    }
    else if (isRcAxisWithinDeadband(PITCH, TRI_TAIL_TUNE_MIN_DEADBAND) && (rcCommand[ROLL] > 100)) {
        pSS->servoVal = pServoConf->max;
        pSS->pLimitToAdjust = &pServoConf->max;
        beeperConfirmationBeeps(3);
        pSS->state = SS_SETUP;
    }
    else if (isRcAxisWithinDeadband(ROLL, TRI_TAIL_TUNE_MIN_DEADBAND) && (rcCommand[PITCH] < -100)) {
        pSS->state = SS_CALIB;
        pSS->cal.state = SS_C_IDLE;
    }
    switch (pSS->state) {
        case SS_IDLE:
            break;
        case SS_SETUP:
            if (!isRcAxisWithinDeadband(YAW, TRI_TAIL_TUNE_MIN_DEADBAND)) {
                pSS->servoVal += (float) triServoDirection * -1.0f * (float) rcCommand[YAW] * dT;
                pSS->servoVal = constrainf(pSS->servoVal, 900.0f, 2100.0f);
                *pSS->pLimitToAdjust = pSS->servoVal;
            }
            break;
        case SS_CALIB:
            // State transition
            if ((pSS->cal.done == true) || (pSS->cal.state == SS_C_IDLE)) {
                if (pSS->cal.state == SS_C_IDLE) {
                    pSS->cal.state = SS_C_CALIB_MIN_MID_MAX;
                    pSS->cal.subState = SS_C_MIN;
                    pSS->servoVal = pServoConf->min;
                    pSS->cal.avg.pCalibConfig = &triflightConfigMutable()->tri_servo_min_adc;
                }
                else if (pSS->cal.state == SS_C_CALIB_SPEED) {
                    pSS->state = SS_IDLE;
                    pSS->cal.subState = SS_C_MIN;
                    beeper(BEEPER_READY_BEEP);
                    // Speed calibration should be done as final step so this saves the min, mid, max and speed values.
                    saveConfigAndNotify();
                }
                else {
                    if (pSS->cal.state == SS_C_CALIB_MIN_MID_MAX) {
                        switch (pSS->cal.subState) {
                            case SS_C_MIN:
                                pSS->cal.subState = SS_C_MID;
                                pSS->servoVal = pServoConf->middle;
                                pSS->cal.avg.pCalibConfig = &triflightConfigMutable()->tri_servo_mid_adc;
                                break;
                            case SS_C_MID:
                                if (ABS(triflightConfig()->tri_servo_min_adc - triflightConfig()->tri_servo_mid_adc) < 100) {
                                    // Not enough difference between min and mid feedback values.
                                    // Most likely the feedback signal is not connected.
                                    pSS->state = SS_IDLE;
                                    pSS->cal.subState = SS_C_MIN;
                                    beeper(BEEPER_ACTION_FAIL);
                                    // Save configuration even after speed calibration failed.
                                    // Speed calibration should be done as final step so this saves the min, mid and max values.
                                    saveConfigAndNotify();
                                }
                                else {
                                    pSS->cal.subState = SS_C_MAX;
                                    pSS->servoVal = pServoConf->max;
                                    pSS->cal.avg.pCalibConfig = &triflightConfigMutable()->tri_servo_max_adc;
                                }
                                break;
                            case SS_C_MAX:
                                pSS->cal.state = SS_C_CALIB_SPEED;
                                pSS->cal.subState = SS_C_MIN;
                                pSS->servoVal = pServoConf->min;
                                pSS->cal.waitingServoToStop = true;
                                break;
                        }
                    }
                }
                pSS->cal.timestamp_ms = GetCurrentTime_ms();
                pSS->cal.avg.sum = 0;
                pSS->cal.avg.numOf = 0;
                pSS->cal.done = false;
            }
            switch (pSS->cal.state) {
                case SS_C_IDLE:
                    break;
                case SS_C_CALIB_MIN_MID_MAX:
                    if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 500)) {
                        if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 600)) {
                            *pSS->cal.avg.pCalibConfig = pSS->cal.avg.sum / pSS->cal.avg.numOf;
                            pSS->cal.done = true;
                        }
                        else {
                            pSS->cal.avg.sum += tailServo.ADCRaw;
                            pSS->cal.avg.numOf++;
                        }
                    }
                    break;
                case SS_C_CALIB_SPEED:
                    switch (pSS->cal.subState) {
                        case SS_C_MIN:
                            // Wait for the servo to reach min position
                            if (tailServo.ADCRaw < (triflightConfig()->tri_servo_min_adc + 10)) {
                                if (!pSS->cal.waitingServoToStop) {
                                    pSS->cal.avg.sum += GetCurrentDelay_ms(pSS->cal.timestamp_ms);
                                    pSS->cal.avg.numOf++;

                                    if (pSS->cal.avg.numOf > 5) {
                                        const float avgTime = pSS->cal.avg.sum / pSS->cal.avg.numOf;
                                        const float avgServoSpeed = (2.0f * tailServo.maxDeflection) / avgTime * 1000.0f;

                                        triflightConfigMutable()->tri_tail_servo_speed = avgServoSpeed;
                                        tailServo.speed = triflightConfig()->tri_tail_servo_speed;
                                        pSS->cal.done = true;
                                        pSS->servoVal = pServoConf->middle;
                                    }
                                    pSS->cal.timestamp_ms = GetCurrentTime_ms();
                                    pSS->cal.waitingServoToStop = true;
                                }
                                    // Wait for the servo to fully stop before starting speed measuring
                                else if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 200)) {
                                    pSS->cal.timestamp_ms = GetCurrentTime_ms();
                                    pSS->cal.subState = SS_C_MAX;
                                    pSS->cal.waitingServoToStop = false;
                                    pSS->servoVal = pServoConf->max;
                                }
                            }
                            break;
                        case SS_C_MAX:
                            // Wait for the servo to reach max position
                            if (tailServo.ADCRaw > (triflightConfig()->tri_servo_max_adc - 10)) {
                                if (!pSS->cal.waitingServoToStop) {
                                    pSS->cal.avg.sum += GetCurrentDelay_ms(pSS->cal.timestamp_ms);
                                    pSS->cal.avg.numOf++;
                                    pSS->cal.timestamp_ms = GetCurrentTime_ms();
                                    pSS->cal.waitingServoToStop = true;
                                }
                                else if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 200)) {
                                    pSS->cal.timestamp_ms = GetCurrentTime_ms();
                                    pSS->cal.subState = SS_C_MIN;
                                    pSS->cal.waitingServoToStop = false;
                                    pSS->servoVal = pServoConf->min;
                                }
                            }
                            break;
                        case SS_C_MID:
                            // Should not come here
                            break;
                    }
            }
            break;
    }
    *pServoVal = pSS->servoVal;
}

static void updateServoAngle(float dT) {
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

static float feedbackServoStep(uint16_t tailServoADC) {
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

#endif