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

#include "drivers/timer.h"

typedef enum {
    DSHOT_BITBANG_OFF,
    DSHOT_BITBANG_ON,
    DSHOT_BITBANG_AUTO,
} dshotBitbangMode_e;

typedef enum {
    DSHOT_BITBANG_OK,
    DSHOT_BITBANG_MOTOR_PIN_CONFLICT,
    DSHOT_BITBANG_NO_PACER
} dshotBitbangStatus_e;

struct motorDevConfig_s;
struct motorDevice_s;
struct motorDevice_s *dshotBitbangDevInit(const struct motorDevConfig_s *motorConfig, uint8_t motorCount);
dshotBitbangStatus_e dshotBitbangGetStatus();
const timerHardware_t* dshotBitbangGetPacerTimer(int index);
