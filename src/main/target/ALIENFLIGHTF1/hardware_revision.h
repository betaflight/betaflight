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

typedef enum awf1HardwareRevision_t {
    AFF1_UNKNOWN = 0,
    AFF1_REV_1, // MPU6050 (I2C)
} awf1HardwareRevision_e;

typedef enum awf4HardwareMotorType_t {
    MOTOR_UNKNOWN = 0,
    MOTOR_BRUSHED,
    MOTOR_BRUSHLESS
} awf4HardwareMotorType_e;

extern uint8_t hardwareRevision;
extern uint8_t hardwareMotorType;

void updateHardwareRevision(void);
void detectHardwareRevision(void);

struct extiConfig_s;
const struct extiConfig_s *selectMPUIntExtiConfigByHardwareRevision(void);
