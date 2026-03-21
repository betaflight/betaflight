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
#include "drivers/servo_impl.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/ledc_ll.h"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop

#include "soc/ledc_struct.h"
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#define SERVO_MAX_COUNT    MAX_SUPPORTED_SERVOS
#define SERVO_PWM_FREQ     50     // 50Hz standard servo
#define SERVO_RESOLUTION   16     // 16-bit duty resolution

typedef struct {
    IO_t io;
    uint8_t ledcChannel;
    bool enabled;
} servoOutput_t;

static servoOutput_t servos[SERVO_MAX_COUNT];
static uint8_t servoCount = 0;
static float usToCountsFactor = 0;

void servoDevInit(const servoDevConfig_t *servoDevConfig)
{
    // Initialize LEDC if not already done
    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
    ledc_ll_enable_bus_clock(true);

    // Configure timer 1 for 50Hz servo frequency
    // 80MHz / (divider * 2^16) = 50Hz
    // Use Q18.4 format for divider
    uint32_t divider = (80000000 << 4) / (SERVO_PWM_FREQ * (1 << SERVO_RESOLUTION));

    ledc_ll_set_clock_divider(&LEDC, LEDC_LOW_SPEED_MODE, 1, divider);
    ledc_ll_set_duty_resolution(&LEDC, LEDC_LOW_SPEED_MODE, 1, SERVO_RESOLUTION);
    ledc_ll_timer_rst(&LEDC, LEDC_LOW_SPEED_MODE, 1);
    ledc_ll_ls_timer_update(&LEDC, LEDC_LOW_SPEED_MODE, 1);

    // Counts per microsecond: 2^16 / (1000000/50) = 65536/20000
    usToCountsFactor = (float)(1 << SERVO_RESOLUTION) / (1000000.0f / SERVO_PWM_FREQ);

    servoCount = 0;

    for (int i = 0; i < SERVO_MAX_COUNT; i++) {
        const ioTag_t tag = servoDevConfig->ioTags[i];
        if (!tag) continue;

        IO_t io = IOGetByTag(tag);
        if (!io) continue;

        IOInit(io, OWNER_SERVO, i);
        IOConfigGPIO(io, IOCFG_OUT_PP);

        uint32_t pin = IO_Pin(io);
        uint8_t ch = 4 + i;  // LEDC channels 4-7

        servos[i].io = io;
        servos[i].ledcChannel = ch;
        servos[i].enabled = true;

        ledc_ll_bind_channel_timer(&LEDC, LEDC_LOW_SPEED_MODE, ch, 1);

        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_output_enable(&GPIO, pin);
        esp_rom_gpio_connect_out_signal(pin, LEDC_LS_SIG_OUT0_IDX + ch, false, false);

        // Set initial position to center (1500us)
        uint32_t duty = lrintf(servoDevConfig->servoCenterPulse * usToCountsFactor);
        ledc_ll_set_duty_int_part(&LEDC, LEDC_LOW_SPEED_MODE, ch, duty);
        ledc_ll_set_duty_start(&LEDC, LEDC_LOW_SPEED_MODE, ch);
        ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, ch);

        servoCount++;
    }
}

void servoWrite(uint8_t index, float value)
{
    if (index >= servoCount || !servos[index].enabled) return;

    // Clamp to valid servo range
    float us = value;
    if (us < PWM_SERVO_MIN) us = PWM_SERVO_MIN;
    if (us > PWM_SERVO_MAX) us = PWM_SERVO_MAX;

    uint32_t duty = lrintf(us * usToCountsFactor);
    uint8_t ch = servos[index].ledcChannel;

    ledc_ll_set_duty_int_part(&LEDC, LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_ll_set_duty_start(&LEDC, LEDC_LOW_SPEED_MODE, ch);
    ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, ch);
}
