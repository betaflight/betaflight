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

#define GRAVITY_CMSS    980.665f

extern int16_t throttleAngleCorrection;
extern int16_t smallAngle;

extern t_fp_vector imuAccelInBodyFrame;
extern t_fp_vector imuMeasuredGravityBF;
extern t_fp_vector imuMeasuredRotationBF;

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

typedef struct imuRuntimeConfig_s {
    float dcm_kp_acc;
    float dcm_ki_acc;
    float dcm_kp_mag;
    float dcm_ki_mag;
    uint8_t small_angle;
} imuRuntimeConfig_t;

struct pidProfile_s;
void imuConfigure(imuRuntimeConfig_t *initialImuRuntimeConfig, struct pidProfile_s *initialPidProfile);

void imuUpdateAttitude(void);
void imuUpdateAccelerometer(void);
float calculateThrottleTiltCompensationFactor(uint8_t throttleTiltCompensationStrength);
float calculateCosTiltAngle(void);
bool isImuReady(void);
bool isImuHeadingValid(void);

void imuTransformVectorBodyToEarth(t_fp_vector * v);
void imuTransformVectorEarthToBody(t_fp_vector * v);

int16_t imuCalculateHeading(t_fp_vector *vec);
void imuInit(void);
