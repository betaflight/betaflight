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

#define DYNAMIC_YAW_MINTHROTTLE_MIN   (0)
#define DYNMAIC_YAW_MINTHROTTLE_MAX   (500)

#define DYNAMIC_YAW_MAXTHROTTLE_MIN   (0)
#define DYNAMIC_YAW_MAXTHROTTLE_MAX   (100)

#define DYNAMIC_YAW_HOVERTHROTTLE_MIN (0)
#define DYNAMIC_YAW_HOVERTHROTTLE_MAX (2000)

#define MOTOR_ACC_YAW_CORRECTION_MIN  (0)
#define MOTOR_ACC_YAW_CORRECTION_MAX  (200)

#define MOTOR_ACCELERATION_MIN        (1)
#define MOTOR_ACCELERATION_MAX        (100)

#define TAIL_MOTOR_INDEX_MIN          (0)
#define TAIL_MOTOR_INDEX_MAX          (2)

#define TAIL_SERVO_ANGLE_MAX_MIN      (0)
#define TAIL_SERVO_ANGLE_MAX_MAX      (400)

#define TAIL_SERVO_FDBK_MIN           (0)
#define TAIL_SERVO_FDBK_MAX           (2)

#define TAIL_SERVO_MAX_ADC_MIN        (0)
#define TAIL_SERVO_MAX_ADC_MAX        (65535)

#define TAIL_SERVO_MID_ADC_MIN        (0)
#define TAIL_SERVO_MID_ADC_MAX        (65535)

#define TAIL_SERVO_MIN_ADC_MIN        (0)
#define TAIL_SERVO_MIN_ADC_MAX        (65535)

#define TAIL_THRUST_FACTOR_MIN        (10)
#define TAIL_THRUST_FACTOR_MAX        (400)

#define TAIL_SERVO_SPEED_MIN          (0)
#define TAIL_SERVO_SPEED_MAX          (1000)

#define TRI_MOTOR_FEEDBACK_LPF_CUTOFF_HZ        (5)
#define TRI_SERVO_FEEDBACK_LPF_CUTOFF_HZ        (70)

typedef struct triflightConfig_s {
    int16_t tri_dynamic_yaw_minthrottle;
    int16_t tri_dynamic_yaw_maxthrottle;
    int16_t tri_dynamic_yaw_hoverthrottle;
    uint16_t tri_motor_acc_yaw_correction;
    uint16_t tri_motor_acceleration;
    int16_t  tri_servo_angle_at_max;
    uint8_t  tri_servo_feedback;
    uint16_t tri_servo_max_adc;
    uint16_t tri_servo_mid_adc;
    uint16_t tri_servo_min_adc;
    uint8_t  tri_tail_motor_index;
    int16_t  tri_tail_motor_thrustfactor;
    int16_t  tri_tail_servo_speed;
} triflightConfig_t;

PG_DECLARE(triflightConfig_t, triflightConfig);

// Servo feedback sources
typedef enum {
    TRI_SERVO_FB_VIRTUAL = 0,  // Virtual servo, no physical feedback signal from servo
    TRI_SERVO_FB_RSSI,         // Feedback signal from RSSI ADC
    TRI_SERVO_FB_CURRENT,      // Feedback signal from CURRENT ADC
} triServoFeedbackSource_e;

uint16_t triGetCurrentServoAngle(void);
int16_t  triGetMotorCorrection(uint8_t motorIndex);
void     triServoMixer(int16_t PIDoutput);
void     triInitMixer(servoParam_t *pTailServoConfig, int16_t *pTailServo);
_Bool    triIsEnabledServoUnarmed(void);

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
    uint32_t sum;
    uint16_t numOf;
} servoAvgAngle_t;

typedef struct thrustTorque_s {
    tailTuneState_e state;
    uint32_t startBeepDelay_ms;
    uint32_t timestamp_ms;
    uint32_t lastAdjTime_ms;
    servoAvgAngle_t servoAvgAngle;
} thrustTorque_t;

typedef struct tailTune_s {
    tailtuneMode_e mode;
    thrustTorque_t tt;
    struct servoSetup_t {
        servoSetupState_e state;
        float servoVal;
        int16_t *pLimitToAdjust;
        struct servoCalib_t {
            _Bool done;
            _Bool waitingServoToStop;
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
