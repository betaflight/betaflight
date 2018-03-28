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

#include "common/axis.h"
#include "common/time.h"
#include "pg/pg.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"

typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_MPU9250,
    GYRO_ICM20601,
    GYRO_ICM20602,
    GYRO_ICM20608G,
    GYRO_ICM20649,
    GYRO_ICM20689,
    GYRO_BMI160,
    GYRO_IMUF9001,
    GYRO_FAKE
} gyroSensor_e;

typedef struct gyro_s {
    uint32_t targetLooptime;
    float gyroADCf[XYZ_AXIS_COUNT];
} gyro_t;

extern gyro_t gyro;

typedef enum {
    GYRO_OVERFLOW_CHECK_NONE = 0,
    GYRO_OVERFLOW_CHECK_YAW,
    GYRO_OVERFLOW_CHECK_ALL_AXES
} gyroOverflowCheck_e;

#if defined(USE_GYRO_IMUF9001)
typedef enum {
    IMUF_RATE_32K = 0,
    IMUF_RATE_16K = 1,
    IMUF_RATE_8K = 2,
    IMUF_RATE_4K = 3,
    IMUF_RATE_2K = 4,
    IMUF_RATE_1K = 5
} imufRate_e;
#endif

typedef struct gyroConfig_s {
    sensor_align_e gyro_align;              // gyro alignment
    uint8_t  gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t  gyro_sync_denom;                  // Gyro sample divider
    uint8_t  gyro_lpf;                         // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint8_t  gyro_soft_lpf_type;
    uint8_t  gyro_soft_lpf_hz;
    bool     gyro_high_fsr;
    bool     gyro_use_32khz;
    uint8_t  gyro_to_use;
    uint16_t gyro_soft_lpf_hz_2;
    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    gyroOverflowCheck_e checkOverflow;

    int16_t  gyro_offset_yaw;
#if defined(USE_GYRO_IMUF9001)
    uint16_t imuf_mode;
    uint16_t imuf_rate;
    uint16_t imuf_pitch_q;
    uint16_t imuf_pitch_w;
    uint16_t imuf_roll_q;
    uint16_t imuf_roll_w;
    uint16_t imuf_yaw_q;
    uint16_t imuf_yaw_w;
    uint16_t imuf_pitch_lpf_cutoff_hz;
    uint16_t imuf_roll_lpf_cutoff_hz;
    uint16_t imuf_yaw_lpf_cutoff_hz;
#else
    uint16_t gyro_filter_q;
    uint16_t gyro_filter_r;
    uint16_t gyro_filter_p;
#endif
    uint8_t  gyro_stage2_filter_type;
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

bool gyroInit(void);

void gyroInitFilters(void);
#ifdef USE_DMA_SPI_DEVICE
void gyroDmaSpiFinishRead(void);
void gyroDmaSpiStartRead(void);
#endif
void gyroUpdate(timeUs_t currentTimeUs);
bool gyroGetAccumulationAverage(float *accumulation);
const busDevice_t *gyroSensorBus(void);
struct mpuConfiguration_s;
const struct mpuConfiguration_s *gyroMpuConfiguration(void);
struct mpuDetectionResult_s;
const struct mpuDetectionResult_s *gyroMpuDetectionResult(void);
void gyroStartCalibration(bool isFirstArmingCalibration);
bool isFirstArmingGyroCalibrationRunning(void);
bool isGyroCalibrationComplete(void);
void gyroReadTemperature(void);
int16_t gyroGetTemperature(void);
int16_t gyroRateDps(int axis);
bool gyroOverflowDetected(void);
uint16_t gyroAbsRateDps(int axis);
#ifdef USE_GYRO_IMUF9001
uint32_t lastImufExtiTime;
bool gyroIsSane(void);
uint16_t returnGyroAlignmentForImuf9001(void);
#endif
