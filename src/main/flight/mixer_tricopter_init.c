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

#ifdef USE_TRIFLIGHT

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "pg/adc.h"

#include "config/config.h"
#include "config/feature.h"

#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"

#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"

extern tailMotor_t tailMotor;
extern tailServo_t tailServo;
extern tailTune_t tailTune;

extern float motorPitchCorrectionCurve[];
extern float yawOutputGainCurve[];
extern int8_t triServoDirection;

extern float getPitchCorrectionAtTailAngle(const float angle, const float thrustFactor);
extern float motorToThrust(const float motor);
extern float binarySearchOutput(const float yawOutput, const float motorWoPitchCorr);
extern bool isRcAxisWithinDeadband(const int32_t axis, const uint8_t minDeadband);

static AdcChannel getServoFeedbackADCChannel(uint8_t tri_servo_feedback);
static int8_t triGetServoDirection(void);
static void initYawForceCurve(void);

#define GetCurrentDelay_ms(timestamp_ms) (now_ms - timestamp_ms)
#define GetCurrentTime_ms() (now_ms)
#define InitDelayMeasurement_ms() const uint32_t now_ms = millis()
#define IsDelayElapsed_ms(timestamp_ms, delay_ms) ((uint32_t) (now_ms - timestamp_ms) >= delay_ms)

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

void initYawForceCurve(void) {
    // DERIVATE(1/(sin(x)-cos(x)/tailServoThrustFactor)) = 0
    // Multiplied by 10 to get decidegrees
    const int16_t minAngle = tailServo.angleAtMin;
    const int16_t maxAngle = tailServo.angleAtMax;

    float maxNegForce = 0;
    float maxPosForce = 0;

    int16_t angle = TRI_CURVE_FIRST_INDEX_ANGLE;

    for (int32_t i = 0; i < TRI_YAW_FORCE_CURVE_SIZE; i++) {
        const float angleRad = DEGREES_TO_RADIANS(angle);

        yawOutputGainCurve[i] = -tailServo.thrustFactor * cos_approx(angleRad) - sin_approx(angleRad);

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

#ifndef UNIT_TEST

void triInitFilters(void) {
    const float dT = pidGetDT();
    pt1FilterInit(&tailMotor.feedbackFilter, pt1FilterGain(TRI_MOTOR_FEEDBACK_LPF_CUTOFF_HZ, dT));
    pt1FilterInit(&tailServo.feedbackFilter, pt1FilterGain(TRI_SERVO_FEEDBACK_LPF_CUTOFF_HZ, dT));
}

void triInitADCs(void){
    if ((triflightConfig()->tri_servo_feedback == TRI_SERVO_FB_RSSI) &&
        !featureIsEnabled(FEATURE_RSSI_ADC)) {
        adcConfigMutable()->rssi.enabled = true;
    }
    if ((triflightConfig()->tri_servo_feedback == TRI_SERVO_FB_CURRENT) &&
        (batteryConfig()->currentMeterSource != CURRENT_METER_ADC)) {
        adcConfigMutable()->current.enabled = true;
    }
#ifdef EXTERNAL1_ADC_PIN
    if (triflightConfig()->tri_servo_feedback == TRI_SERVO_FB_EXT1) {
            adcConfigMutable()->external1.enabled = true;
        }
#endif
}

#endif

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

void tailTuneModeServoSetup(struct servoSetup_t *pSS, servoParam_t *pServoConf, int16_t *pServoVal, float dT) {
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

#endif // USE_TRIFLIGHT