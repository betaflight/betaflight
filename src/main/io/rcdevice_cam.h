/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
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

void rcdeviceInit(void);
void rcdeviceUpdate(timeUs_t currentTimeUs);

bool rcdeviceIsEnabled(void);

void rcdeviceSend5KeyOSDCableSimualtionEvent(rcdeviceCamSimulationKeyEvent_e key);
