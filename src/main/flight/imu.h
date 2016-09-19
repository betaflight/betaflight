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

#pragma once

extern int16_t throttleAngleCorrection;
extern uint32_t accTimeSum;
extern int accSumCount;
extern float accVelScale;
extern int16_t accSmooth[XYZ_AXIS_COUNT];
extern int32_t accSum[XYZ_AXIS_COUNT];

#define DEGREES_TO_DECIDEGREES(angle) (angle * 10)
#define DECIDEGREES_TO_DEGREES(angle) (angle / 10)
#define DECIDEGREES_TO_RADIANS(angle) ((angle / 10.0f) * 0.0174532925f)
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)

typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;

extern attitudeEulerAngles_t attitude;

typedef struct imuConfig_s {
    // IMU configuration
    uint8_t gyro_sync_denom;                // Gyro sample divider
    uint8_t pid_process_denom;              // Processing denominator for PID controller vs gyro sampling rate
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;                    // Angle used for mag hold threshold.
    uint16_t max_angle_inclination;         // max inclination allowed in angle (level) mode. default 500 (50 degrees).
} imuConfig_t;

PG_DECLARE(imuConfig_t, imuConfig);

typedef struct throttleCorrectionConfig_s {
    uint16_t throttle_correction_angle;     // the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
    uint8_t throttle_correction_value;      // the correction that will be applied at throttle_correction_angle.
} throttleCorrectionConfig_t;

PG_DECLARE_PROFILE(throttleCorrectionConfig_t, throttleCorrectionConfig);

typedef struct imuRuntimeConfig_s {
    uint8_t acc_cut_hz;
    uint8_t acc_unarmedcal;
    float dcm_ki;
    float dcm_kp;
    uint8_t small_angle;
} imuRuntimeConfig_t;

void imuInit(void);

void imuConfigure(
    imuRuntimeConfig_t *initialImuRuntimeConfig,
    accDeadband_t *initialAccDeadband,
    float accz_lpf_cutoff,
    uint16_t throttle_correction_angle
);

void imuUpdateAccelerometer(rollAndPitchTrims_t *accelerometerTrims);
void imuUpdateAttitude(void);
float calculateThrottleAngleScale(uint16_t throttle_correction_angle);
int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value);
float calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_cutoff);

int16_t imuCalculateHeading(t_fp_vector *vec);

float getCosTiltAngle(void);

void imuResetAccelerationSum(void);

bool imuIsAircraftArmable(uint8_t arming_angle);

