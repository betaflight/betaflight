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


typedef enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PID_ITEM_COUNT
} pidIndex_e;

typedef struct pidProfile_s {
    uint8_t P8[PID_ITEM_COUNT];
    uint8_t I8[PID_ITEM_COUNT];
    uint8_t D8[PID_ITEM_COUNT];

    float P_f[3];                           // float p i and d factors for the new baseflight pid
    float I_f[3];
    float D_f[3];
    float A_level;
    float H_level;
    uint8_t H_sensitivity;
} pidProfile_t;

typedef enum {
    AI_ROLL = 0,
    AI_PITCH,
} angle_index_t;

#define ANGLE_INDEX_COUNT 2

// See http://en.wikipedia.org/wiki/Flight_dynamics
typedef enum {
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flight_dynamics_index_t;

#define FLIGHT_DYNAMICS_INDEX_COUNT 3

typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} flightDynamicsTrims_def_t;

typedef union {
    int16_t raw[3];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;

typedef struct rollAndPitchInclination_s {
    // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
} rollAndPitchInclination_t_def;

typedef union {
    int16_t raw[ANGLE_INDEX_COUNT];
    rollAndPitchInclination_t_def values;
} rollAndPitchInclination_t;


#define DEGREES_TO_DECIDEGREES(angle) (angle * 10)
#define DECIDEGREES_TO_DEGREES(angle) (angle / 10.0f)

extern rollAndPitchInclination_t inclination;

extern int16_t gyroData[FLIGHT_DYNAMICS_INDEX_COUNT];
extern int16_t gyroZero[FLIGHT_DYNAMICS_INDEX_COUNT];

extern int16_t gyroADC[XYZ_AXIS_COUNT], accADC[XYZ_AXIS_COUNT], accSmooth[XYZ_AXIS_COUNT];
extern int32_t accSum[XYZ_AXIS_COUNT];
extern int16_t axisPID[XYZ_AXIS_COUNT];

extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];

extern int16_t heading, magHold;

extern int32_t AltHold;
extern int32_t vario;

void setPIDController(int type);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void resetErrorAngle(void);
void resetErrorGyro(void);


