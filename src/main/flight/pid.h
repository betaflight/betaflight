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

#define GYRO_I_MAX 256                      // Gyro I limiter
#define RCconstPI   0.159154943092f         // 0.5f / M_PI;
#define MAIN_CUT_HZ 12.0f                   // (default 12Hz, Range 1-50Hz)
#define OLD_YAW 0                           // [0/1] 0 = MultiWii 2.3 yaw, 1 = older yaw.
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

#define IS_PID_CONTROLLER_FP_BASED(pidController) (pidController == 2)

typedef struct pidProfile_s {
    uint8_t pidController;                  // 0 = multiwii original, 1 = rewrite from http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671, 1, 2 = Luggi09s new baseflight pid

    uint8_t P8[PID_ITEM_COUNT];
    uint8_t I8[PID_ITEM_COUNT];
    uint8_t D8[PID_ITEM_COUNT];
    uint8_t PIDweight[PID_ITEM_COUNT];

    float P_f[3];                           // float p i and d factors for lux float pid controller
    float I_f[3];
    float D_f[3];
    float A_level;
    float H_level;
    uint8_t H_sensitivity;
    uint16_t yaw_p_limit;                   // set P term limit (fixed value was 300)
} pidProfile_t;

#define DEGREES_TO_DECIDEGREES(angle) (angle * 10)
#define DECIDEGREES_TO_DEGREES(angle) (angle / 10.0f)

extern int16_t axisPID[XYZ_AXIS_COUNT];
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];

void pidSetController(int type);
void pidResetErrorAngle(void);
void pidResetErrorGyro(void);


