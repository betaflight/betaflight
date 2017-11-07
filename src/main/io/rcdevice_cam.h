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

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "io/rcdevice.h"
#include "fc/rc_modes.h"

#define FIVE_KEY_CABLE_JOYSTICK_MIN 1080
#define FIVE_KEY_CABLE_JOYSTICK_MAX 1920
#define FIVE_KEY_CABLE_JOYSTICK_MID_START 1350
#define FIVE_KEY_CABLE_JOYSTICK_MID_END 1650

typedef struct rcdeviceSwitchState_s {
    bool isActivated;
} rcdeviceSwitchState_t;

extern runcamDevice_t *camDevice;
extern bool rcdeviceInMenu;

bool rcdeviceInit(void);
void rcdeviceUpdate(timeUs_t currentTimeUs);

bool rcdeviceIsEnabled(void);

// used for unit test
rcdeviceSwitchState_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
