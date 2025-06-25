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

#include <stdbool.h>
#include <stdint.h>
#include "platform.h"

#if defined(USE_BEEPER) && defined(USE_PWM_OUTPUT)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

static bool beeperInit;
static bool beeperEnabled;
static int beeperGPIO;
static uint pwmSlice;

void pwmWriteBeeper(bool on)
{
    if (beeperInit) {
        gpio_set_outover(beeperGPIO, on ? GPIO_OVERRIDE_NORMAL : GPIO_OVERRIDE_LOW);
        pwm_set_enabled(pwmSlice, on);
        beeperEnabled = on;
    }
}

void pwmToggleBeeper(void)
{
    pwmWriteBeeper(!beeperEnabled);
}

void beeperPwmInit(const ioTag_t tag, uint16_t frequency)
{
    IO_t beeperIO = IOGetByTag(tag);
    if (beeperIO) {
        beeperGPIO = IO_GPIOPinIdx(beeperIO);
        IOInit(beeperIO, OWNER_BEEPER, 0);

        // f = sysclk / div / wrap
        uint32_t clock_divide = 64; // gives a decent range of frequencies (down to 36Hz) for RP2350 at 150MHz
        float wrap_f = ((float)SystemCoreClock) / frequency / clock_divide;
        uint16_t wrap = (uint16_t)(wrap_f > 65535 ? 65535 : wrap_f);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv_int(&cfg, clock_divide);
        pwm_config_set_wrap(&cfg, wrap);
        pwmSlice = pwm_gpio_to_slice_num(beeperGPIO);
        pwm_init(pwmSlice, &cfg, false);
        gpio_set_function(beeperGPIO, GPIO_FUNC_PWM);
        pwm_set_gpio_level(beeperGPIO, wrap >> 1); // 50% square wave
        beeperEnabled = false;
        beeperInit = true;
    } else {
        beeperInit = false;
    }
}

#endif
