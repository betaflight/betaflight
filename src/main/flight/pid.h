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

#define PID_MAX_I 256
#define PID_MAX_D 512
#define PID_MAX_TOTAL_PID 1000

#define GYRO_I_MAX 256                      // Gyro I limiter
#define YAW_P_LIMIT_MIN 100                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_MAX 500                 // Maximum value for yaw P limiter

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

typedef enum {
    PID_CONTROLLER_MW23 = 0,
    PID_CONTROLLER_MWREWRITE,
    PID_CONTROLLER_LUX_FLOAT,
    PID_COUNT
} pidControllerType_e;

#define IS_PID_CONTROLLER_FP_BASED(pidController) (pidController == PID_CONTROLLER_LUX_FLOAT)

typedef enum {
    PID_DELTA_FROM_MEASUREMENT = 0,
    PID_DELTA_FROM_ERROR
} pidDeltaMethod_e;

typedef struct pidProfile_s {
    uint8_t  P8[PID_ITEM_COUNT];
    uint8_t  I8[PID_ITEM_COUNT];
    uint8_t  D8[PID_ITEM_COUNT];
    uint8_t  pidController;
    uint16_t yaw_p_limit;                   // set P term limit (fixed value was 300)
    uint16_t dterm_lpf;                     // dterm filtering
    uint16_t yaw_lpf;                       // additional yaw filter when yaw axis too noisy
    uint8_t  deltaMethod;
} pidProfile_t;

PG_DECLARE_PROFILE(pidProfile_t, pidProfile);

struct controlRateConfig_s;
union rollAndPitchTrims_u;
struct rxConfig_s;
typedef void (*pidControllerFuncPtr)(const pidProfile_t *pidProfile, const struct controlRateConfig_s *controlRateConfig,
        uint16_t max_angle_inclination, const union rollAndPitchTrims_u *angleTrim, const struct rxConfig_s *rxConfig);            // pid controller function prototype

extern int16_t axisPID[FD_INDEX_COUNT];
extern int32_t axisPID_P[FD_INDEX_COUNT], axisPID_I[FD_INDEX_COUNT], axisPID_D[FD_INDEX_COUNT];

float pidScaleITermToRcInput(int axis);
void pidFilterIsSetCheck(const pidProfile_t *pidProfile);

void pidSetController(pidControllerType_e type);
void pidResetITermAngle(void);
void pidResetITerm(void);

