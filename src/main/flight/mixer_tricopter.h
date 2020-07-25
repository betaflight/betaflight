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

#ifndef MIXER_TRICOPTER_H_
#define MIXER_TRICOPTER_H_

#include "common/filter.h"
#include "drivers/adc.h"
#include "flight/servos.h"

#define LINEAR_MIN_OUTPUT_MIN        (5)
#define LINEAR_MIN_OUTPUT_MAX        (50)

#define MOTOR_ACC_YAW_CORRECTION_MIN (0)
#define MOTOR_ACC_YAW_CORRECTION_MAX (200)

#define MOTOR_ACCELERATION_MIN       (1)
#define MOTOR_ACCELERATION_MAX       (100)

#define TAIL_SERVO_ANGLE_MAX_MIN     (0)
#define TAIL_SERVO_ANGLE_MAX_MAX     (400)

#define TAIL_SERVO_FDBK_MIN         (0)
#define TAIL_SERVO_FDBK_MAX         (2)

#define TAIL_SERVO_MAX_ADC_MIN      (0)
#define TAIL_SERVO_MAX_ADC_MAX      (65535)

#define TAIL_SERVO_MID_ADC_MIN      (0)
#define TAIL_SERVO_MID_ADC_MAX      (65535)

#define TAIL_SERVO_MIN_ADC_MIN      (0)
#define TAIL_SERVO_MIN_ADC_MAX      (65535)

#define TAIL_THRUST_FACTOR_MIN      (10)
#define TAIL_THRUST_FACTOR_MAX      (400)

#define TAIL_SERVO_SPEED_MIN        (0)
#define TAIL_SERVO_SPEED_MAX        (1000)

#define YAW_BOOST_MIN               (10)
#define YAW_BOOST_MAX               (700)

#define TRI_TAIL_SERVO_ANGLE_MID                (90.0f)
#define TRI_TAIL_SERVO_MAX_ANGLE                (40.0f)

#define TRI_CURVE_FIRST_INDEX_ANGLE             (TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE)
#define TRI_MOTOR_FEEDBACK_LPF_CUTOFF_HZ        (5)
#define TRI_SERVO_FEEDBACK_LPF_CUTOFF_HZ        (70)
#define TRI_SERVO_SATURATED_GYRO_ERROR          (75.0f)
#define TRI_SERVO_SATURATION_DPS_ERROR_LIMIT    (100.0f)
#define TRI_TAIL_SERVO_INVALID_ANGLE_MAX        (TRI_TAIL_SERVO_ANGLE_MID + TRI_TAIL_SERVO_MAX_ANGLE + 3.0f)
#define TRI_TAIL_SERVO_INVALID_ANGLE_MIN        (TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE - 3.0f)
#define TRI_TAIL_TUNE_MIN_DEADBAND              (12)
#define TRI_YAW_FORCE_CURVE_SIZE                (80 + 1)

typedef struct triflightConfig_s {
    uint16_t tri_motor_acc_yaw_correction;
    uint16_t tri_motor_acceleration;
    int16_t  tri_servo_angle_at_max;
    uint8_t  tri_servo_feedback;
    uint16_t tri_servo_max_adc;
    uint16_t tri_servo_mid_adc;
    uint16_t tri_servo_min_adc;
    int16_t  tri_tail_motor_thrustfactor;
    int16_t  tri_tail_servo_speed;
    uint16_t tri_yaw_boost;
    uint8_t  tri_tail_motor_index;
} triflightConfig_t;

PG_DECLARE(triflightConfig_t, triflightConfig);

// Servo feedback sources. */
typedef enum {
    TRI_SERVO_FB_VIRTUAL = 0,  // Virtual servo, no physical feedback signal from servo
    TRI_SERVO_FB_RSSI,         // Feedback signal from RSSI ADC
    TRI_SERVO_FB_CURRENT,      // Feedback signal from CURRENT ADC
    TRI_SERVO_FB_EXT1,         // Feedback signal from EXT1 ADC
} triServoFeedbackSource_e;

float   triGetCurrentServoAngle(void);
int16_t triGetMotorCorrection(uint8_t motorIndex);
void    triInitFilters();
void    triInitMixer(servoParam_t *pTailServoConfig, int16_t *pTailServo);
bool    triIsEnabledServoUnarmed(void);
bool    triIsServoSaturated(float rateError);
void    triServoMixer(float scaledYawPid, float pidSumLimit, float dT);

typedef enum {
    TRI_ARMING_PREVENT_FLAG_INVALID_SERVO_ANGLE = 0x01,
    TRI_ARMING_PREVENT_FLAG_UNARMED_TAIL_TUNE   = 0x02
} triArmingPreventFlag_e;

typedef enum {
    TT_IDLE = 0,
    TT_WAIT,
    TT_ACTIVE,
    TT_WAIT_FOR_DISARM,
    TT_DONE,
    TT_FAIL,
} tailTuneState_e;

typedef enum {
    SS_IDLE = 0,
    SS_SETUP,
    SS_CALIB,
} servoSetupState_e;

typedef enum {
    SS_C_IDLE = 0,
    SS_C_CALIB_MIN_MID_MAX,
    SS_C_CALIB_SPEED,
} servoSetupCalibState_e;

typedef enum {
    SS_C_MIN = 0,
    SS_C_MID,
    SS_C_MAX,
} servoSetupCalibSubState_e;

typedef enum {
    TT_MODE_NONE = 0,
    TT_MODE_THRUST_TORQUE,
    TT_MODE_SERVO_SETUP,
} tailtuneMode_e;

typedef struct servoAvgAngle_s {
    float sum;
    uint16_t numOf;
} servoAvgAngle_t;

typedef struct thrustTorque_s {
    tailTuneState_e state;
    uint32_t startBeepDelay_ms;
    uint32_t timestamp_ms;
    uint32_t timestamp2_ms;
    uint32_t lastAdjTime_ms;
    servoAvgAngle_t servoAvgAngle;
    float tailTuneGyroLimit;
} thrustTorque_t;

typedef struct tailServo_s {
    pt1Filter_t feedbackFilter;
    bool feedbackHealthy;
    float maxYawOutput;
    float thrustFactor;
    servoParam_t *pConf;       // Pointer to the tail servo configuration
    int16_t *pOutput;          // Pointer to the servo output (setpoint) that controls the PWM output
    AdcChannel ADCChannel;
    float maxDeflection;
    int16_t speed;
    float angleAtMin;
    float angleAtMax;
    float angleAtLinearMin;
    float angleAtLinearMax;
    float angle;               // Current measured angle
    uint16_t ADCRaw;
} tailServo_t;

typedef struct tailMotor_s {
    pt1Filter_t feedbackFilter;
    float virtualFeedBack;
    float acceleration;        // Motor acceleration in output units (us) / second
    float pitchCorrectionGain; // Gain added to the calculated tail motor pitch correction to gain more yaw output
    int16_t lastCorrection;
    uint16_t minOutput;
    uint16_t linearMinOutput;  // Minimum motor output for linear calculation.
    uint16_t outputRange;
} tailMotor_t;

typedef struct tailTune_s {
    tailtuneMode_e mode;
    thrustTorque_t tt;
    struct servoSetup_t {
        servoSetupState_e state;
        float servoVal;
        int16_t *pLimitToAdjust;
        struct servoCalib_t {
            bool done;
            bool waitingServoToStop;
            servoSetupCalibState_e state;
            servoSetupCalibSubState_e subState;
            uint32_t timestamp_ms;
            struct average_t {
                uint16_t *pCalibConfig;
                uint32_t sum;
                uint16_t numOf;
            } avg;
        } cal;
    } ss;
} tailTune_t;

#endif // MIXER_TRICOPTER_H_ 
