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

#include "platform.h"

#if defined(USE_GYRO_CLKIN)

#include "drivers/gyro_clkin.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/resource.h"
#include "drivers/timer.h"

static timerChannel_t pwmChannel;

bool gyroClkInInit(ioTag_t tag, uint32_t freqHz, uint8_t resourceIndex)
{
    if (!tag || freqHz == 0) {
        return false;
    }

    const timerHardware_t *timer = timerAllocate(tag, OWNER_GYRO_CLKIN, resourceIndex);
    if (!timer) {
        return false;
    }

    const IO_t io = IOGetByTag(tag);
    IOInit(io, OWNER_GYRO_CLKIN, resourceIndex);
    IOConfigGPIOAF(io, IOCFG_AF_PP, timer->alternateFunction);

    const uint32_t clock = timerClock(timer);
    const uint16_t period = clock / freqHz;
    const uint16_t value = period / 2;

    pwmOutputConfig(&pwmChannel, timer, clock, period - 1, value - 1, 0);
    pwmWriteChannel(&pwmChannel, value - 1);

    return true;
}

#endif
