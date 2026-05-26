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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"
#include "drivers/pwm_output.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/sound_beeper.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/ledc_ll.h"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop

#include "soc/ledc_struct.h"
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

static IO_t beeperIO = IO_NONE;
static uint8_t beeperLedcChannel = 0;
static bool beeperActive = false;
static uint32_t beeperDuty = 0;

void beeperPwmInit(const ioTag_t tag, uint16_t frequency)
{
    if (!tag || frequency == 0) return;

    // Initialize LEDC if needed
    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
    ledc_ll_enable_bus_clock(true);

    beeperIO = IOGetByTag(tag);
    if (!beeperIO) return;

    IOInit(beeperIO, OWNER_BEEPER, 0);
    IOConfigGPIO(beeperIO, IOCFG_OUT_PP);

    // Use timer 2 for beeper, channel 7 (last available)
    beeperLedcChannel = 7;

    // Calculate for desired frequency with 10-bit resolution
    uint8_t resolution = 10;  // 1024 counts
    uint32_t divider = (80000000 << 4) / (frequency * (1 << resolution));

    ledc_ll_set_clock_divider(&LEDC, LEDC_LOW_SPEED_MODE, 2, divider);
    ledc_ll_set_duty_resolution(&LEDC, LEDC_LOW_SPEED_MODE, 2, resolution);
    ledc_ll_timer_rst(&LEDC, LEDC_LOW_SPEED_MODE, 2);
    ledc_ll_ls_timer_update(&LEDC, LEDC_LOW_SPEED_MODE, 2);

    ledc_ll_bind_channel_timer(&LEDC, LEDC_LOW_SPEED_MODE, beeperLedcChannel, 2);

    // 50% duty for square wave
    beeperDuty = (1 << resolution) / 2;

    uint32_t pin = IO_Pin(beeperIO);
    esp_rom_gpio_pad_select_gpio(pin);
    gpio_ll_output_enable(&GPIO, pin);
    esp_rom_gpio_connect_out_signal(pin, LEDC_LS_SIG_OUT0_IDX + beeperLedcChannel, false, false);

    // Start silent
    ledc_ll_set_duty_int_part(&LEDC, LEDC_LOW_SPEED_MODE, beeperLedcChannel, 0);
    ledc_ll_set_duty_start(&LEDC, LEDC_LOW_SPEED_MODE, beeperLedcChannel);
    ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, beeperLedcChannel);
}

void pwmWriteBeeper(bool onoff)
{
    if (!beeperIO) return;

    beeperActive = onoff;
    uint32_t duty = onoff ? beeperDuty : 0;

    ledc_ll_set_duty_int_part(&LEDC, LEDC_LOW_SPEED_MODE, beeperLedcChannel, duty);
    ledc_ll_set_duty_start(&LEDC, LEDC_LOW_SPEED_MODE, beeperLedcChannel);
    ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, beeperLedcChannel);
}

void pwmToggleBeeper(void)
{
    pwmWriteBeeper(!beeperActive);
}
