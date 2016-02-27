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

typedef struct accDeadband_s {
    uint8_t xy;                 // set the acc deadband for xy-Axis
    uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
} accDeadband_t;

typedef struct imuRuntimeConfig_s {
    uint8_t acc_cut_hz;
    uint8_t acc_unarmedcal;
    float dcm_ki;
    float dcm_kp;
    uint8_t small_angle;
} imuRuntimeConfig_t;

typedef enum {
    ACCPROC_READ = 0,
    ACCPROC_CHUNK_1,
    ACCPROC_CHUNK_2,
    ACCPROC_CHUNK_3,
    ACCPROC_CHUNK_4,
    ACCPROC_CHUNK_5,
    ACCPROC_CHUNK_6,
    ACCPROC_CHUNK_7,
    ACCPROC_COPY
} accProcessorState_e;

typedef struct accProcessor_s {
    accProcessorState_e state;
} accProcessor_t;

void imuConfigure(
    imuRuntimeConfig_t *initialImuRuntimeConfig,
    pidProfile_t *initialPidProfile,
    accDeadband_t *initialAccDeadband,
    uint16_t throttle_correction_angle
);

float getCosTiltAngle(void);
void calculateEstimatedAltitude(uint32_t currentTime);
void imuUpdateAccelerometer(rollAndPitchTrims_t *accelerometerTrims);
void imuUpdateGyro(void);
void imuUpdateAttitude(void);
float calculateThrottleAngleScale(uint16_t throttle_correction_angle);
int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value);
float calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_hz);

int16_t imuCalculateHeading(t_fp_vector *vec);

void imuResetAccelerationSum(void);
void imuUpdateGyro(void);
void imuUpdateAcc(rollAndPitchTrims_t *accelerometerTrims);
