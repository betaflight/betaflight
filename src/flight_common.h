#pragma once

enum {
    AI_ROLL = 0,
    AI_PITCH,
} angle_index_t;

#define ANGLE_INDEX_COUNT 2

extern int16_t angle[ANGLE_INDEX_COUNT]; // see angle_index_t

enum {
    GI_ROLL = 0,
    GI_PITCH,
    GI_YAW
} gyro_index_t;

#define GYRO_INDEX_COUNT 3

extern int16_t gyroData[GYRO_INDEX_COUNT]; // see gyro_index_t
extern int16_t gyroZero[GYRO_INDEX_COUNT]; // see gyro_index_t

extern int16_t gyroADC[XYZ_AXIS_COUNT], accADC[XYZ_AXIS_COUNT], accSmooth[XYZ_AXIS_COUNT];
extern int32_t accSum[XYZ_AXIS_COUNT];

void mwDisarm(void);

