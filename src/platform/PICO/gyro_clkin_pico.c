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

#include <stdint.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "drivers/gyro_clkin.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/resource.h"

bool gyroClkInInit(ioTag_t tag, uint32_t freqHz, uint8_t resourceIndex)
{
    if (!tag || freqHz == 0) {
        return false;
    }

    const IO_t io = IOGetByTag(tag);
    if (!io) {
        return false;
    }

    IOInit(io, OWNER_GYRO_CLKIN, resourceIndex);

    const uint8_t pin = IO_GPIOPinIdx(io);
    const uint8_t slice = pwm_gpio_to_slice_num(pin);
    const uint8_t channel = pwm_gpio_to_channel(pin);

    // PWM block clock is sys_clk; pick clkdiv=1 and derive wrap directly. The
    // 16-bit wrap register caps the achievable lower bound (~sys_clk / 65536),
    // which is well below the typical 32 kHz CLKIN rate even at 150 MHz sys_clk.
    const uint32_t sysClk = clock_get_hz(clk_sys);
    const uint32_t wrap = (sysClk / freqHz) - 1;
    if (wrap > UINT16_MAX) {
        return false;
    }

    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_wrap(slice, (uint16_t)wrap);
    pwm_set_chan_level(slice, channel, (uint16_t)((wrap + 1) / 2));
    pwm_set_enabled(slice, true);

    return true;
}

#endif
