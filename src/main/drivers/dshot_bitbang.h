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

#include "platform.h"

#include "common/time.h"

#include "drivers/dma.h"
#include "drivers/io_types.h"
#include "drivers/motor_types.h"
#include "drivers/timer.h"

#include "pg/motor.h"

typedef enum {
    DSHOT_BITBANG_STATUS_OK,
    DSHOT_BITBANG_STATUS_MOTOR_PIN_CONFLICT,
    DSHOT_BITBANG_STATUS_NO_PACER,
    DSHOT_BITBANG_STATUS_TOO_MANY_PORTS,
} dshotBitbangStatus_e;

bool dshotBitbangDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig);
dshotBitbangStatus_e dshotBitbangGetStatus(void);
const timerHardware_t *dshotBitbangTimerGetAllocatedByNumberAndChannel(int8_t timerNumber, uint16_t timerChannel);
const resourceOwner_t *dshotBitbangTimerGetOwner(const timerHardware_t *timer);
