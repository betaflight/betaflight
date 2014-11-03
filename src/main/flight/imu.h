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

typedef struct imuRuntimeConfig_s {
    uint8_t acc_lpf_factor;
    uint8_t acc_unarmedcal;
    float gyro_cmpf_factor;
    float gyro_cmpfm_factor;
    int8_t small_angle;
} imuRuntimeConfig_t;

void configureImu(imuRuntimeConfig_t *initialImuRuntimeConfig, pidProfile_t *initialPidProfile, accDeadband_t *initialAccDeadband);

void calculateEstimatedAltitude(uint32_t currentTime);
void computeIMU(rollAndPitchTrims_t *accelerometerTrims, uint8_t mixerConfiguration);
void calculateThrottleAngleScale(uint16_t throttle_correction_angle);
int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value);
void calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_cutoff);

void accSum_reset(void);
