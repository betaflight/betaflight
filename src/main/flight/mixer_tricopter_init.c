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

extern tailMotor_t tailMotor;
extern tailServo_t tailServo;
extern tailTune_t tailTune;

extern float motorPitchCorrectionCurve[];
extern float yawOutputGainCurve[];
extern int8_t triServoDirection;

extern float getPitchCorrectionAtTailAngle(float angle, float thrustFactor);
extern float motorToThrust(float motor);
extern float binarySearchOutput(float yawOutput, float motorWoPitchCorr);

static AdcChannel getServoFeedbackADCChannel(uint8_t tri_servo_feedback);
static int8_t triGetServoDirection(void);
static void initYawForceCurve(void);

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