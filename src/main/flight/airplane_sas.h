/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "pid.h"

typedef struct psas_pitch_ctrl_s {
    float pilot;
    float damping;
    float stability;
    float I;
    float accelP;
    float Sum;
} psas_pitch_ctrl_t;

typedef struct psas_roll_ctrl_s {
    float pilot;
    float damping;
    float Sum;
} psas_roll_ctrl_t;

typedef struct psas_yaw_ctrl_s {
    float pilot;
    float damping;
    float stability;
    float rollToYawCrossLink;
    float Sum;
} psas_yaw_ctrl_t;

typedef struct psas_control_s {
    psas_pitch_ctrl_t pitch;
    psas_roll_ctrl_t roll;
    psas_yaw_ctrl_t yaw;
} psas_control_t;

extern psas_control_t psasControl;

void psasInit(const pidProfile_t *pidProfile);
bool psasHandleMode(const pidProfile_t *pidProfile);
