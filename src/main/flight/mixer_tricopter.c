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
#include <stdint.h>
#include <math.h>
#include <float.h>

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

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

#include "pg/motor.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"

#define TRI_TAIL_SERVO_ANGLE_MID    (900)
#define TRI_YAW_FORCE_CURVE_SIZE    (100)
#define TRI_TAIL_SERVO_MAX_ANGLE    (500)

#define TRI_YAW_FORCE_PRECISION     (1000)

#define IsDelayElapsed_us(timestamp_us, delay_us) ((uint32_t)(micros() - timestamp_us) >= delay_us)
#define IsDelayElapsed_ms(timestamp_ms, delay_ms) ((uint32_t)(millis() - timestamp_ms) >= delay_ms)

PG_REGISTER_WITH_RESET_TEMPLATE(triflightConfig_t, triflightConfig, PG_TRIFLIGHT_CONFIG, 0);

PG_RESET_TEMPLATE(triflightConfig_t, triflightConfig,
    .tri_dynamic_yaw_minthrottle   = 100,
    .tri_dynamic_yaw_maxthrottle   = 100,
    .tri_dynamic_yaw_hoverthrottle = 0,
    .tri_motor_acc_yaw_correction  = 6,
    .tri_motor_acceleration        = 18,
    .tri_servo_angle_at_max        = 400,
    .tri_servo_feedback            = TRI_SERVO_FB_RSSI,
    .tri_servo_max_adc             = 0,
    .tri_servo_mid_adc             = 0,
    .tri_servo_min_adc             = 0,
    .tri_tail_motor_index          = 0,
    .tri_tail_motor_thrustfactor   = 138,
    .tri_tail_servo_speed          = 300,
);

static float dT;

static tailTune_t tailTune = {.mode = TT_MODE_NONE};

static int16_t  tailMotorAccelerationDelay_ms = 30;
static int16_t  tailMotorDecelerationDelay_ms = 100;
static int16_t  tailMotorAccelerationDelay_angle;
static int16_t  tailMotorDecelerationDelay_angle;
static int16_t  tailMotorPitchZeroAngle;
static uint16_t tailServoADC         = 0;
static uint16_t tailServoAngle       = TRI_TAIL_SERVO_ANGLE_MID;
static int32_t  tailServoMaxYawForce = 0;
static int16_t  tailServoMaxAngle    = 0;
static int16_t  tailServoSpeed       = 0;
static int32_t  yawForceCurve[TRI_YAW_FORCE_CURVE_SIZE];

int32_t         hoverThrottleSum;
float           tailServoThrustFactor = 0;

// Virtual tail motor speed feedback
static float tailMotorVirtual = 1000.0f;

// Configured output throttle range (max - min)
static int16_t throttleRange = 0;

// Motor acceleration in output units (us) / second
static float motorAcceleration = 0;

// Reset the I term when tail motor deceleration has lasted (ms)
static uint16_t resetITermDecelerationLasted_ms = 0;

static int16_t      * gpTailServo;
static servoParam_t * gpTailServoConf;

static AdcChannel tailServoADCChannel = ADC_RSSI;

static int16_t  dynamicYaw(int16_t PIDoutput);
static uint16_t feedbackServoStep(uint16_t tailServoADC);
static uint16_t getAngleFromYawCurveAtForce(int32_t force);
static uint16_t getLinearServoValue(servoParam_t *servoConf, int16_t constrainedPIDOutput);
static float    getPitchCorrectionAtTailAngle(float angle, float thrustFactor);
static uint16_t getPitchCorrectionMaxPhaseShift(int16_t servoAngle,
                                                int16_t servoSetpointAngle,
                                                int16_t motorAccelerationDelayAngle,
                                                int16_t motorDecelerationDelayAngle,
                                                int16_t motorDirectionChangeAngle);
static uint16_t getServoAngle(servoParam_t * servoConf, uint16_t servoValue);
static uint16_t getServoValueAtAngle(servoParam_t * servoConf, uint16_t angle);
static void     initCurves(void);
static void     tailMotorStep(int16_t setpoint, float dT);
static void     tailTuneModeServoSetup(struct servoSetup_t *pSS, servoParam_t *pServoConf, int16_t *pServoVal);
static void     triTailTuneStep(servoParam_t *pServoConf, int16_t *pServoVal);
static void     tailTuneModeThrustTorque(thrustTorque_t *pTT, const bool isThrottleHigh);
static void     updateServoAngle(void);
static void     updateServoFeedbackADCChannel(uint8_t tri_servo_feedback);
static uint16_t virtualServoStep(uint16_t currentAngle, int16_t servoSpeed, float dT, servoParam_t *servoConf, uint16_t servoValue);

static pt1Filter_t feedbackFilter;
static pt1Filter_t motorFilter;

void triInitMixer(servoParam_t *pTailServoConfig, int16_t *pTailServo)
{
    gpTailServoConf       = pTailServoConfig;
    gpTailServo           = pTailServo;
    tailServoThrustFactor = triflightConfig()->tri_tail_motor_thrustfactor / 10.0f;
    tailServoMaxAngle     = triflightConfig()->tri_servo_angle_at_max;
    tailServoSpeed        = triflightConfig()->tri_tail_servo_speed;

    throttleRange         = motorConfig()->maxthrottle - motorConfig()->minthrottle;
    motorAcceleration     = (float)throttleRange / ((float)triflightConfig()->tri_motor_acceleration * 0.01f);

    // Reset the I term when motor deceleration has lasted 35% of the min to max time
    resetITermDecelerationLasted_ms = (float)triflightConfig()->tri_motor_acceleration * 10.0f * 0.35f;

    initCurves();
    updateServoFeedbackADCChannel(triflightConfig()->tri_servo_feedback);
}

static void initCurves(void)
{
    // DERIVATE(1/(sin(x)-cos(x)/tailServoThrustFactor)) = 0
    // Multiplied by 10 to get decidegrees
    tailMotorPitchZeroAngle = 10.0f * 2.0f * (atanf(((sqrtf(tailServoThrustFactor * tailServoThrustFactor + 1) + 1) / tailServoThrustFactor)));

    tailMotorAccelerationDelay_angle = 10.0f * (tailMotorAccelerationDelay_ms / 1000.0f) * tailServoSpeed;
    tailMotorDecelerationDelay_angle = 10.0f * (tailMotorDecelerationDelay_ms / 1000.0f) * tailServoSpeed;

    const int16_t minAngle = TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle;
    const int16_t maxAngle = TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle;
    int32_t maxNegForce    = 0;
    int32_t maxPosForce    = 0;

    int16_t angle = TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE;
    for (int32_t i = 0; i < TRI_YAW_FORCE_CURVE_SIZE; i++)
    {
        const float angleRad = DEGREES_TO_RADIANS(angle / 10.0f);
        yawForceCurve[i] = TRI_YAW_FORCE_PRECISION * (-tailServoThrustFactor * cosf(angleRad) - sinf(angleRad)) * getPitchCorrectionAtTailAngle(angleRad, tailServoThrustFactor);
        // Only calculate the top forces in the configured angle range
        if ((angle >= minAngle) && (angle <= maxAngle))
        {
            maxNegForce = MIN(yawForceCurve[i], maxNegForce);
            maxPosForce = MAX(yawForceCurve[i], maxPosForce);
        }
        angle += 10;
    }

    tailServoMaxYawForce = MIN(ABS(maxNegForce), ABS(maxPosForce));
}

uint16_t triGetCurrentServoAngle(void)
{
    return tailServoAngle;
}

static uint16_t getLinearServoValue(servoParam_t *servoConf, int16_t constrainedPIDOutput)
{
    const int32_t linearYawForceAtValue = tailServoMaxYawForce * constrainedPIDOutput / TRI_YAW_FORCE_PRECISION;
    
    const int16_t correctedAngle = getAngleFromYawCurveAtForce(linearYawForceAtValue);
    
    return getServoValueAtAngle(servoConf, correctedAngle);
}

void triServoMixer(int16_t PIDoutput)
{
    dT = pidGetDT();

    // Dynamic yaw expects input [-1000, 1000]
    PIDoutput = constrain(PIDoutput, -1000, 1000);

    // Scale the PID output based on tail motor speed (thrust)
    PIDoutput = dynamicYaw(PIDoutput);

    if (triflightConfig()->tri_servo_feedback != TRI_SERVO_FB_VIRTUAL)
    {
        // Read new servo feedback signal sample and run it through filter
        tailServoADC = pt1FilterApply4(&feedbackFilter, 
                                       adcGetChannel(tailServoADCChannel),
                                       TRI_SERVO_FEEDBACK_LPF_CUTOFF_HZ,
                                       dT);
    }

    updateServoAngle();

    *gpTailServo = getLinearServoValue(gpTailServoConf, PIDoutput);

#if 1
    DEBUG_SET(DEBUG_TRIFLIGHT, 0, (uint32_t)adcGetChannel(tailServoADCChannel));
    DEBUG_SET(DEBUG_TRIFLIGHT, 1, (uint32_t)tailServoADC);
    DEBUG_SET(DEBUG_TRIFLIGHT, 2, (uint32_t)tailServoAngle);
#endif

    triTailTuneStep(gpTailServoConf, gpTailServo);

    // Update the tail motor virtual feedback
    tailMotorStep(motor[triflightConfig()->tri_tail_motor_index], dT);
}

int16_t triGetMotorCorrection(uint8_t motorIndex)
{
    uint16_t correction = 0;
    if (motorIndex == triflightConfig()->tri_tail_motor_index)
    {
        // Adjust tail motor speed based on servo angle. Check how much to adjust speed from pitch force curve based on servo angle.
        // Take motor speed up lag into account by shifting the phase of the curve
        // Not taking into account the motor braking lag (yet)
        const uint16_t servoAngle = triGetCurrentServoAngle();
        const uint16_t servoSetpointAngle = getServoAngle(gpTailServoConf, *gpTailServo);

        const uint16_t maxPhaseShift = getPitchCorrectionMaxPhaseShift(servoAngle, servoSetpointAngle, tailMotorAccelerationDelay_angle, tailMotorDecelerationDelay_angle, tailMotorPitchZeroAngle);

        int16_t angleDiff = servoSetpointAngle - servoAngle;
        if (ABS(angleDiff) > maxPhaseShift)
        {
            angleDiff = (int32_t)maxPhaseShift * angleDiff / ABS(angleDiff);
        }

        const int16_t futureServoAngle = constrain(servoAngle + angleDiff, TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle, TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle);
        uint16_t throttleMotorOutput = tailMotorVirtual - motorConfig()->minthrottle;
        /* Increased yaw authority at min throttle, always calculate the pitch
         * correction on at least half motor output. This produces a little bit
         * more forward pitch, but tested to be negligible.
         *
         * TODO: this is not the best way to achieve this, but how could the min_throttle
         * pitch correction be calculated, as the thrust is zero?
         */
        throttleMotorOutput = constrain(throttleMotorOutput, throttleRange / 2, 1000);
        correction = (throttleMotorOutput * getPitchCorrectionAtTailAngle(DEGREES_TO_RADIANS(futureServoAngle / 10.0f), tailServoThrustFactor)) - throttleMotorOutput;
    }

    return correction;
}

static uint16_t getServoValueAtAngle(servoParam_t *servoConf, uint16_t angle)
{
    const int16_t servoMid = servoConf->middle;
    uint16_t servoValue;

    if (angle < TRI_TAIL_SERVO_ANGLE_MID)
    {
        const int16_t servoMin = servoConf->min;
        servoValue = (int32_t)(angle - tailServoMaxAngle) * (servoMid - servoMin) / (TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle) + servoMin;
    }
    else if (angle > TRI_TAIL_SERVO_ANGLE_MID)
    {
        servoValue = (int32_t)(angle - TRI_TAIL_SERVO_ANGLE_MID) * (servoConf->max - servoMid) / tailServoMaxAngle + servoMid;
    }
    else
    {
        servoValue = servoMid;
    }

    return servoValue;
}

static float getPitchCorrectionAtTailAngle(float angle, float thrustFactor)
{
    return 1 / (sin_approx(angle) - cos_approx(angle) / thrustFactor);
}

static uint16_t getAngleFromYawCurveAtForce(int32_t force)
{
    if (force < yawForceCurve[0]) // No force that low
    {
        return TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE;
    }
    else if (!(force < yawForceCurve[TRI_YAW_FORCE_CURVE_SIZE - 1])) // No force that high
    {
        return TRI_TAIL_SERVO_ANGLE_MID + TRI_TAIL_SERVO_MAX_ANGLE;
    }
    // Binary search: yawForceCurve[lower] <= force, yawForceCurve[higher] > force
    int32_t lower = 0, higher = TRI_YAW_FORCE_CURVE_SIZE - 1;
    while (higher > lower + 1)
    {
        const int32_t mid = (lower + higher) / 2;
        if (yawForceCurve[mid] > force)
        {
            higher = mid;
        }
        else
        {
            lower = mid;
        }
    }
    // Interpolating
    return TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE + lower * 10 + (int32_t)(force - yawForceCurve[lower]) * 10 / (yawForceCurve[higher] - yawForceCurve[lower]);
}

static uint16_t getServoAngle(servoParam_t *servoConf, uint16_t servoValue)
{
    const int16_t midValue   = servoConf->middle;
    const int16_t endValue   = servoValue < midValue ? servoConf->min : servoConf->max;
    const int16_t endAngle   = servoValue < midValue ? TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle : TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle;
    const int16_t servoAngle = (int32_t)(endAngle - TRI_TAIL_SERVO_ANGLE_MID) * (servoValue - midValue) / (endValue - midValue) + TRI_TAIL_SERVO_ANGLE_MID;
    
    return servoAngle;
}

static uint16_t getPitchCorrectionMaxPhaseShift(int16_t servoAngle,
                                                int16_t servoSetpointAngle,
                                                int16_t motorAccelerationDelayAngle,
                                                int16_t motorDecelerationDelayAngle,
                                                int16_t motorDirectionChangeAngle)
{
    uint16_t maxPhaseShift;

    if (((servoAngle > servoSetpointAngle) && (servoAngle >= (motorDirectionChangeAngle + motorAccelerationDelayAngle))) ||
        ((servoAngle < servoSetpointAngle) && (servoAngle <= (motorDirectionChangeAngle - motorAccelerationDelayAngle))))
    {
        // Motor is braking
        maxPhaseShift = ABS(servoAngle - motorDirectionChangeAngle) >= motorDecelerationDelayAngle ?
                                                                                                   motorDecelerationDelayAngle:
                                                                                                   ABS(servoAngle - motorDirectionChangeAngle);
    }
    else
    {
        // Motor is accelerating
        maxPhaseShift = motorAccelerationDelayAngle;
    }

    return maxPhaseShift;
}

static uint16_t virtualServoStep(uint16_t currentAngle, int16_t servoSpeed, float dT, servoParam_t *servoConf, uint16_t servoValue)
{
    const uint16_t angleSetPoint = getServoAngle(servoConf, servoValue);
    const uint16_t dA            = dT * servoSpeed * 10; // Max change of an angle since last check

    if ( ABS(currentAngle - angleSetPoint) < dA )
    {
        // At set-point after this moment
        currentAngle = angleSetPoint;
    }
    else if (currentAngle < angleSetPoint)
    {
        currentAngle += dA;
    }
    else // tailServoAngle.virtual > angleSetPoint
    {
        currentAngle -= dA;
    }

    return currentAngle;
}

static uint16_t feedbackServoStep(uint16_t tailServoADC)
{
    // Feedback servo
    const int32_t ADCFeedback       = tailServoADC;
    const int16_t midValue          = triflightConfig()->tri_servo_mid_adc;
    const int16_t endValue          = ADCFeedback < midValue ? triflightConfig()->tri_servo_min_adc :triflightConfig()->tri_servo_max_adc;
    const int16_t tailServoMaxAngle = triflightConfig()->tri_servo_angle_at_max;
    const int16_t endAngle          = ADCFeedback < midValue ? TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle : TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle;
    
    return ((endAngle - TRI_TAIL_SERVO_ANGLE_MID) * (ADCFeedback - midValue) / (endValue - midValue) + TRI_TAIL_SERVO_ANGLE_MID);
}

static void updateServoAngle(void)
{
    if (triflightConfig()->tri_servo_feedback == TRI_SERVO_FB_VIRTUAL)
    {
        tailServoAngle = virtualServoStep(tailServoAngle, tailServoSpeed, dT, gpTailServoConf, *gpTailServo);
    }
    else
    {
        tailServoAngle = feedbackServoStep(tailServoADC);
    }
}

static void updateServoFeedbackADCChannel(uint8_t tri_servo_feedback)
{
    switch (tri_servo_feedback)
    {
    case TRI_SERVO_FB_RSSI:
        tailServoADCChannel = ADC_RSSI;
        break;
    case TRI_SERVO_FB_CURRENT:
        tailServoADCChannel = ADC_CURRENT;
        break;
    default:
        tailServoADCChannel = ADC_RSSI;
        break;
    }
}

static int16_t dynamicYaw(int16_t PIDoutput)
{
    static int32_t range     = 0;
    static int32_t lowRange  = 0;
    static int32_t highRange = 0;

    int16_t gain;

    if (triflightConfig()->tri_dynamic_yaw_hoverthrottle == 0)
        return PIDoutput;

    if (range == 0 && lowRange == 0 && highRange == 0)
    {
        range     = (int32_t)(getMotorOutputHigh() - getMotorOutputLow());
        lowRange  = range * (triflightConfig()->tri_dynamic_yaw_hoverthrottle - (int32_t)getMotorOutputLow()) / range;
        highRange = range - lowRange;
    }

#if 0
    DEBUG_SET(DEBUG_TRIFLIGHT, 0, range);
    DEBUG_SET(DEBUG_TRIFLIGHT, 1, lowRange);
    DEBUG_SET(DEBUG_TRIFLIGHT, 2, highRange);
#endif

    // Select the yaw gain based on tail motor speed
    if (tailMotorVirtual < triflightConfig()->tri_dynamic_yaw_hoverthrottle)
    {
        /* Below hover point, gain is increasing the output.
         * e.g. 150 (%) increases the yaw output at min throttle by 150 % (1.5x)
         * e.g. 250 (%) increases the yaw output at min throttle by 250 % (2.5x)
         */
        gain = triflightConfig()->tri_dynamic_yaw_minthrottle - 100;
    }
    else
    {
        /* Above hover point, gain is decreasing the output.
         * e.g. 75 (%) reduces the yaw output at max throttle by 25 % (0.75x)
         * e.g. 20 (%) reduces the yaw output at max throttle by 80 % (0.2x)
         */
        gain = 100 - triflightConfig()->tri_dynamic_yaw_maxthrottle;
    }

    int16_t distanceFromMid = (tailMotorVirtual - triflightConfig()->tri_dynamic_yaw_hoverthrottle);

    int32_t scaledPIDoutput;

    if (lowRange == 0 || highRange == 0)
    scaledPIDoutput = PIDoutput;
    else
    {
        if (tailMotorVirtual < triflightConfig()->tri_dynamic_yaw_hoverthrottle)
            scaledPIDoutput = PIDoutput - distanceFromMid * gain * PIDoutput / (lowRange * 100);
        else
            scaledPIDoutput = PIDoutput - distanceFromMid * gain * PIDoutput / (highRange * 100);
    }

#if 0
    DEBUG_SET(DEBUG_TRIFLIGHT, 0, tailMotorVirtual);
    DEBUG_SET(DEBUG_TRIFLIGHT, 1, gain);
    DEBUG_SET(DEBUG_TRIFLIGHT, 2, PIDoutput);
    DEBUG_SET(DEBUG_TRIFLIGHT, 3, scaledPIDoutput);
#endif

    return constrain(scaledPIDoutput, -1000, 1000);
}

static void tailMotorStep(int16_t setpoint, float dT)
{
    static float current = 1000;
    float dS; // Max change of an speed since last check

    dS = dT * motorAcceleration;

    if ( ABS(current - setpoint) < dS )
    {
        // At set-point after this moment
        current = setpoint;
    }
    else if (current < setpoint)
    {
        current += dS;
    }
    else
    {
        current -= dS;
    }

    // Use a PT1 low-pass filter to add "slowness" to the virtual motor feedback.
    /* Cut-off to delay:
     * 2  Hz -> 25 ms
     * 5  Hz -> 14 ms
     * 10 Hz -> 9  ms
     */
    
    tailMotorVirtual = pt1FilterApply4(&motorFilter, 
                                       current,
                                       TRI_MOTOR_FEEDBACK_LPF_CUTOFF_HZ,
                                       dT);
 
#if 0
    DEBUG_SET(DEBUG_TRIFLIGHT, 0, setpoint);
    DEBUG_SET(DEBUG_TRIFLIGHT, 1, current);
    DEBUG_SET(DEBUG_TRIFLIGHT, 2, tailMotorVirtual);
#endif

}

_Bool triIsEnabledServoUnarmed(void)
{
    const _Bool isEnabledServoUnarmed = (servoConfig()->tri_unarmed_servo != 0) || FLIGHT_MODE(TAILTUNE_MODE);

    return isEnabledServoUnarmed;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

bool isRcAxisWithinDeadband(int32_t axis)
{
    int32_t tmp = MIN(ABS(rcCommand[axis]), 500);
    bool ret = false;
    if (axis == ROLL || axis == PITCH)
    {
        if (tmp <= rcControlsConfig()->deadband)
        {
            ret = true;
        }
    }
    else
    {
        if (tmp <= rcControlsConfig()->yaw_deadband)
        {
            ret = true;
        }
    }

    return ret;
}

static void triTailTuneStep(servoParam_t *pServoConf, int16_t *pServoVal)
{
    if (!IS_RC_MODE_ACTIVE(BOXTAILTUNE))
    {
        if (FLIGHT_MODE(TAILTUNE_MODE))
        {
            DISABLE_ARMING_FLAG(ARMING_DISABLED_TAILTUNE);
            DISABLE_FLIGHT_MODE(TAILTUNE_MODE);
            tailTune.mode = TT_MODE_NONE;
        }
        return;
    }
    else
    {
        ENABLE_FLIGHT_MODE(TAILTUNE_MODE);
    }

    if (tailTune.mode == TT_MODE_NONE)
    {
        if (ARMING_FLAG(ARMED))
        {
            tailTune.mode     = TT_MODE_THRUST_TORQUE;
            tailTune.tt.state = TT_IDLE;
        }
        else
        {
            // Prevent accidental arming in servo setup mode
            ENABLE_ARMING_FLAG(ARMING_DISABLED_TAILTUNE);
            
            tailTune.mode        = TT_MODE_SERVO_SETUP;
            tailTune.ss.servoVal = pServoConf->middle;
        }
    }

    switch (tailTune.mode)
    {
    case TT_MODE_THRUST_TORQUE:
        tailTuneModeThrustTorque(&tailTune.tt, (THROTTLE_HIGH == calculateThrottleStatus()));
        break;
    case TT_MODE_SERVO_SETUP:
        tailTuneModeServoSetup(&tailTune.ss, pServoConf, pServoVal);
        break;
    case TT_MODE_NONE:
        break;
    }
}

static void tailTuneModeThrustTorque(thrustTorque_t *pTT, const bool isThrottleHigh)
{
    switch(pTT->state)
    {
    case TT_IDLE:
        // Calibration has been requested, only start when throttle is up
        if (isThrottleHigh && ARMING_FLAG(ARMED))
        {
            beeper(BEEPER_BAT_LOW);
            
            pTT->startBeepDelay_ms   = 1000;
            pTT->timestamp_ms        = millis();
            pTT->lastAdjTime_ms      = millis();
            pTT->state               = TT_WAIT;
            pTT->servoAvgAngle.sum   = 0;
            pTT->servoAvgAngle.numOf = 0;
            hoverThrottleSum         = 0;
        }
        break;
    case TT_WAIT:
        if (isThrottleHigh && ARMING_FLAG(ARMED))
        {
            /* Wait for 5 seconds before activating the tuning.
            This is so that pilot has time to take off if the tail tune mode was activated on ground. */
            if (IsDelayElapsed_ms(pTT->timestamp_ms, 5000))
            {
                // Longer beep when starting
                beeper(BEEPER_BAT_CRIT_LOW);
                
                pTT->state        = TT_ACTIVE;
                pTT->timestamp_ms = millis();
            }
            else if (IsDelayElapsed_ms(pTT->timestamp_ms, pTT->startBeepDelay_ms))
            {
                // Beep every second until start
                beeper(BEEPER_BAT_LOW);
                
                pTT->startBeepDelay_ms += 1000;
            }
        }
        else
        {
            pTT->state = TT_IDLE;
        }
        break;
    case TT_ACTIVE:
        if (isThrottleHigh &&
            isRcAxisWithinDeadband(ROLL)  &&
            isRcAxisWithinDeadband(PITCH) &&
            isRcAxisWithinDeadband(YAW)   &&
            (fabsf(gyro.gyroADCf[FD_YAW]) <= 10.0f)) // deg/s
        {
            if (IsDelayElapsed_ms(pTT->timestamp_ms, 250))
            {
                // RC commands have been within deadbands for 250 ms
                if (IsDelayElapsed_ms(pTT->lastAdjTime_ms, 10))
                {
                    pTT->lastAdjTime_ms = millis();

                    pTT->servoAvgAngle.sum += triGetCurrentServoAngle();
                    pTT->servoAvgAngle.numOf++;

                    hoverThrottleSum += (motor[triflightConfig()->tri_tail_motor_index]);

                    beeperConfirmationBeeps(1);

                    if (pTT->servoAvgAngle.numOf >= 300)
                    {
                        beeper(BEEPER_READY_BEEP);
                        
                        pTT->state        = TT_WAIT_FOR_DISARM;
                        pTT->timestamp_ms = millis();
                    }
                }
            }
        }
        else
        {
            pTT->timestamp_ms = millis();
        }
        break;
    case TT_WAIT_FOR_DISARM:
        if (!ARMING_FLAG(ARMED))
        {
            float averageServoAngle = pTT->servoAvgAngle.sum / 10.0f / pTT->servoAvgAngle.numOf;
            
            if (averageServoAngle > 90.5f && averageServoAngle < 120.f)
            {
                averageServoAngle -= 90.0f;
                averageServoAngle *= RAD;
                
                triflightConfigMutable()->tri_tail_motor_thrustfactor = 10.0f * cos_approx(averageServoAngle) / sin_approx(averageServoAngle);

                triflightConfigMutable()->tri_dynamic_yaw_hoverthrottle = hoverThrottleSum / (int16_t)pTT->servoAvgAngle.numOf;

                saveConfigAndNotify();

                pTT->state = TT_DONE;
            }
            else
            {
                pTT->state = TT_FAIL;
            }
            pTT->timestamp_ms = millis();
        }
        else
        {
            if (IsDelayElapsed_ms(pTT->timestamp_ms, 2000))
            {
                beeper(BEEPER_READY_BEEP);
                
                pTT->timestamp_ms = millis();
            }
        }
        break;
    case TT_DONE:
        if (IsDelayElapsed_ms(pTT->timestamp_ms, 2000))
        {
            beeper(BEEPER_READY_BEEP);
            
            pTT->timestamp_ms = millis();
        }
        break;
    case TT_FAIL:
        if (IsDelayElapsed_ms(pTT->timestamp_ms, 2000))
        {
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
            
            pTT->timestamp_ms = millis();
        }
        break;
    }
}

static void tailTuneModeServoSetup(struct servoSetup_t *pSS, servoParam_t *pServoConf, int16_t *pServoVal)
{
    // Check mode select
    if (isRcAxisWithinDeadband(PITCH) && (rcCommand[ROLL] < -100))
    {
        pSS->servoVal       = pServoConf->min;
        pSS->pLimitToAdjust = &pServoConf->min;
        pSS->state          = SS_SETUP;

        beeperConfirmationBeeps(1);
    }
    else if (isRcAxisWithinDeadband(ROLL) && (rcCommand[PITCH] > 100))
    {
        pSS->servoVal       = pServoConf->middle;
        pSS->pLimitToAdjust = &pServoConf->middle;
        pSS->state          = SS_SETUP;

        beeperConfirmationBeeps(2);
    }
    else if (isRcAxisWithinDeadband(PITCH) && (rcCommand[ROLL] > 100))
    {
        pSS->servoVal       = pServoConf->max;
        pSS->pLimitToAdjust = &pServoConf->max;
        pSS->state          = SS_SETUP;

        beeperConfirmationBeeps(3);
    }
    else if (isRcAxisWithinDeadband(ROLL) && (rcCommand[PITCH] < -100))
    {
        pSS->state     = SS_CALIB;
        pSS->cal.state = SS_C_IDLE;
    }

    switch (pSS->state)
    {
    case SS_IDLE:
        break;
    case SS_SETUP:
        if (!isRcAxisWithinDeadband(YAW))
        {
            pSS->servoVal += -1.0f * (float)rcCommand[YAW] * dT;
            
            constrain(pSS->servoVal, 950, 2050);
            
            *pSS->pLimitToAdjust = pSS->servoVal;
        }
        break;
    case SS_CALIB:
        // State transition
        if ((pSS->cal.done == true) || (pSS->cal.state == SS_C_IDLE))
        {
            if (pSS->cal.state == SS_C_IDLE)
            {
                pSS->cal.state            = SS_C_CALIB_MIN_MID_MAX;
                pSS->cal.subState         = SS_C_MIN;
                pSS->servoVal             = pServoConf->min;
                pSS->cal.avg.pCalibConfig = &triflightConfigMutable()->tri_servo_min_adc;
            }
            else if (pSS->cal.state == SS_C_CALIB_SPEED)
            {
                pSS->state = SS_IDLE;
                pSS->cal.subState = SS_C_IDLE;

                beeper(BEEPER_READY_BEEP);

                // Speed calibration should be done as final step so this saves the min, mid, max and speed values.
                saveConfigAndNotify();
            }
            else
            {
                if (pSS->cal.state == SS_C_CALIB_MIN_MID_MAX)
                {
                    switch (pSS->cal.subState)
                    {
                    case SS_C_MIN:
                        pSS->cal.subState         = SS_C_MID;
                        pSS->servoVal             = pServoConf->middle;
                        pSS->cal.avg.pCalibConfig = &triflightConfigMutable()->tri_servo_mid_adc;
                        break;
                    case SS_C_MID:
                        if (ABS(triflightConfigMutable()->tri_servo_min_adc - triflightConfigMutable()->tri_servo_mid_adc) < 100)
                        {
                            /* Not enough difference between min and mid feedback values.
                             * Most likely the feedback signal is not connected.
                             */
                            pSS->state        = SS_IDLE;
                            pSS->cal.subState = SS_C_IDLE;

                            beeper(BEEPER_ACC_CALIBRATION_FAIL);

                            /* Save configuration even after speed calibration failed.
                             * Speed calibration should be done as final step so this saves the min, mid and max values.
                             */
                            saveConfigAndNotify();
                        }
                        else
                        {
                            pSS->cal.subState         = SS_C_MAX;
                            pSS->servoVal             = pServoConf->max;
                            pSS->cal.avg.pCalibConfig = &triflightConfigMutable()->tri_servo_max_adc;
                        }
                        break;
                    case SS_C_MAX:
                        pSS->cal.state              = SS_C_CALIB_SPEED;
                        pSS->cal.subState           = SS_C_MIN;
                        pSS->servoVal               = pServoConf->min;
                        pSS->cal.waitingServoToStop = true;
                        break;
                    }
#if 0
                    DEBUG_SET(DEBUG_TRIFLIGHT, 0, (uint32_t)triflightConfigMutable()->tri_servo_min_adc);
                    DEBUG_SET(DEBUG_TRIFLIGHT, 1, (uint32_t)triflightConfigMutable()->tri_servo_mid_adc);
                    DEBUG_SET(DEBUG_TRIFLIGHT, 2, (uint32_t)triflightConfigMutable()->tri_servo_max_adc);
#endif
                }
            }

            pSS->cal.timestamp_ms = millis();
            pSS->cal.avg.sum      = 0;
            pSS->cal.avg.numOf    = 0;
            pSS->cal.done         = false;
        }

        switch (pSS->cal.state)
        {
        case SS_C_IDLE:
            break;
        case SS_C_CALIB_MIN_MID_MAX:
            if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 500))
            {
                if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 600))
                {
                    *pSS->cal.avg.pCalibConfig = pSS->cal.avg.sum / pSS->cal.avg.numOf;
                    pSS->cal.done              = true;
                }
                else
                {
                    pSS->cal.avg.sum += tailServoADC;
                    pSS->cal.avg.numOf++;
                }
            }
            break;
        case SS_C_CALIB_SPEED:
            switch (pSS->cal.subState)
            {
            case SS_C_MIN:
                // Wait for the servo to reach min position
                if (tailServoADC < (triflightConfigMutable()->tri_servo_min_adc + 10))
                {
                    if (!pSS->cal.waitingServoToStop)
                    {
                        pSS->cal.avg.sum += millis() - pSS->cal.timestamp_ms;
                        pSS->cal.avg.numOf++;

                        if (pSS->cal.avg.numOf > 5)
                        {
                            const float avgTime       = pSS->cal.avg.sum / pSS->cal.avg.numOf;
                            const float avgServoSpeed = (2.0f * tailServoMaxAngle / 10.0f) / avgTime * 1000.0f;
                            
                            triflightConfigMutable()->tri_tail_servo_speed = avgServoSpeed;
                            tailServoSpeed                                 = triflightConfig()->tri_tail_servo_speed;
                            
                            pSS->cal.done = true;
                            pSS->servoVal = pServoConf->middle;
                        }

                        pSS->cal.timestamp_ms       = millis();
                        pSS->cal.waitingServoToStop = true;
                    }
                    // Wait for the servo to fully stop before starting speed measuring
                    else if  (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 200))
                    {
                        pSS->cal.timestamp_ms       = millis();
                        pSS->cal.subState           = SS_C_MAX;
                        pSS->cal.waitingServoToStop = false;
                        pSS->servoVal               = pServoConf->max;
                    }
                }
                break;
            case SS_C_MAX:
                // Wait for the servo to reach max position
                if (tailServoADC > (triflightConfigMutable()->tri_servo_max_adc - 10))
                {
                    if (!pSS->cal.waitingServoToStop)
                    {
                        pSS->cal.avg.sum += millis() - pSS->cal.timestamp_ms;
                        pSS->cal.avg.numOf++;
                        
                        pSS->cal.timestamp_ms       = millis();
                        pSS->cal.waitingServoToStop = true;
                    }
                    else if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 200))
                    {
                        pSS->cal.timestamp_ms       = millis();
                        pSS->cal.subState           = SS_C_MIN;
                        pSS->cal.waitingServoToStop = false;
                        pSS->servoVal               = pServoConf->min;
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
