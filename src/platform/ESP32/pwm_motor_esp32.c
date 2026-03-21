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
#include "drivers/motor.h"
#include "drivers/motor_impl.h"
#include "drivers/pwm_output.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "pg/motor.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/ledc_ll.h"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop

#include "soc/ledc_struct.h"
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#define PWM_MAX_MOTORS 4

typedef struct {
    IO_t io;
    uint8_t ledcChannel;
    bool enabled;
    float pulseScale;
    float pulseOffset;
} pwmMotor_t;

static pwmMotor_t esp32PwmMotors[PWM_MAX_MOTORS];
static uint8_t esp32PwmMotorCount = 0;
static bool ledcInitialized = false;

static void pwmMotorPostInit(void) { }

static bool pwmMotorEnable(void)
{
    for (int i = 0; i < esp32PwmMotorCount; i++) {
        esp32PwmMotors[i].enabled = true;
    }
    return true;
}

static void pwmMotorDisable(void)
{
    for (int i = 0; i < esp32PwmMotorCount; i++) {
        esp32PwmMotors[i].enabled = false;
        ledc_ll_set_duty_int_part(&LEDC, LEDC_LOW_SPEED_MODE, esp32PwmMotors[i].ledcChannel, 0);
        ledc_ll_set_duty_start(&LEDC, LEDC_LOW_SPEED_MODE, esp32PwmMotors[i].ledcChannel);
        ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, esp32PwmMotors[i].ledcChannel);
    }
}

static bool pwmMotorIsEnabled(unsigned index)
{
    return index < esp32PwmMotorCount && esp32PwmMotors[index].enabled;
}

static void pwmMotorWrite(uint8_t index, float value)
{
    if (index >= esp32PwmMotorCount || !esp32PwmMotors[index].enabled) return;

    // Convert value to duty cycle
    uint32_t duty = lrintf(value * esp32PwmMotors[index].pulseScale + esp32PwmMotors[index].pulseOffset);

    ledc_ll_set_duty_int_part(&LEDC, LEDC_LOW_SPEED_MODE, esp32PwmMotors[index].ledcChannel, duty);
    ledc_ll_set_duty_start(&LEDC, LEDC_LOW_SPEED_MODE, esp32PwmMotors[index].ledcChannel);
    ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, esp32PwmMotors[index].ledcChannel);
}

static void pwmMotorWriteInt(uint8_t index, uint16_t value)
{
    pwmMotorWrite(index, (float)value);
}

static void pwmMotorUpdateComplete(void) { }
static bool pwmMotorDecodeTelemetry(void) { return true; }
static bool pwmMotorTelemetryWait(void) { return true; }
static void pwmMotorUpdateInit(void) { }

static void pwmMotorShutdown(void)
{
    pwmMotorDisable();
}

static bool pwmMotorIsIdle(unsigned index)
{
    UNUSED(index);
    return false;
}

static IO_t pwmMotorGetIO(unsigned index)
{
    if (index >= esp32PwmMotorCount) return IO_NONE;
    return esp32PwmMotors[index].io;
}

static void pwmMotorRequestTelemetry(unsigned index) { UNUSED(index); }

static const motorVTable_t pwmMotorVTable = {
    .postInit = pwmMotorPostInit,
    .convertExternalToMotor = NULL,
    .convertMotorToExternal = NULL,
    .enable = pwmMotorEnable,
    .disable = pwmMotorDisable,
    .isMotorEnabled = pwmMotorIsEnabled,
    .telemetryWait = pwmMotorTelemetryWait,
    .decodeTelemetry = pwmMotorDecodeTelemetry,
    .updateInit = pwmMotorUpdateInit,
    .write = pwmMotorWrite,
    .writeInt = pwmMotorWriteInt,
    .updateComplete = pwmMotorUpdateComplete,
    .shutdown = pwmMotorShutdown,
    .isMotorIdle = pwmMotorIsIdle,
    .getMotorIO = pwmMotorGetIO,
    .requestTelemetry = pwmMotorRequestTelemetry,
};

static void ledcInit(void)
{
    if (ledcInitialized) return;

    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
    ledc_ll_enable_bus_clock(true);
    ledc_ll_enable_reset_reg(false);

    ledcInitialized = true;
}

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorDevConfig, uint16_t idlePulse)
{
    ledcInit();

    // Configure timer 0 based on protocol
    uint32_t pwmRate = motorDevConfig->motorPwmRate;
    if (pwmRate == 0) pwmRate = 490;  // Default PWM rate

    // Calculate divider for desired frequency
    // LEDC timer: freq = APB_CLK / (divider * 2^resolution)
    // Use 16-bit resolution (65536 counts)
    uint8_t resolution = 16;
    uint32_t divider = 80000000 / (pwmRate * (1 << resolution));
    if (divider < 1) { divider = 1; resolution = 14; }  // Reduce resolution for high rates

    ledc_ll_set_clock_divider(&LEDC, LEDC_LOW_SPEED_MODE, 0, divider << 4);  // Q18.4 format
    ledc_ll_set_duty_resolution(&LEDC, LEDC_LOW_SPEED_MODE, 0, resolution);
    ledc_ll_timer_rst(&LEDC, LEDC_LOW_SPEED_MODE, 0);
    ledc_ll_ls_timer_update(&LEDC, LEDC_LOW_SPEED_MODE, 0);

    esp32PwmMotorCount = 0;

    for (int i = 0; i < PWM_MAX_MOTORS && i < MAX_SUPPORTED_MOTORS; i++) {
        const ioTag_t tag = motorDevConfig->ioTags[i];
        if (!tag) break;

        IO_t io = IOGetByTag(tag);
        if (!io) break;

        IOInit(io, OWNER_MOTOR, i);
        IOConfigGPIO(io, IOCFG_OUT_PP);

        uint32_t pin = IO_Pin(io);
        uint8_t ch = i;  // LEDC channel 0-3

        esp32PwmMotors[i].io = io;
        esp32PwmMotors[i].ledcChannel = ch;
        esp32PwmMotors[i].enabled = false;

        // Pulse scale: maps motor value to duty cycle counts
        // For standard PWM: 1000-2000us at the given resolution
        uint32_t maxDuty = (1 << resolution);
        float period_us = 1000000.0f / pwmRate;
        esp32PwmMotors[i].pulseScale = maxDuty / period_us;
        esp32PwmMotors[i].pulseOffset = idlePulse * esp32PwmMotors[i].pulseScale;

        // Bind channel to timer 0
        ledc_ll_bind_channel_timer(&LEDC, LEDC_LOW_SPEED_MODE, ch, 0);

        // Connect LEDC output to GPIO
        // LEDC output signals: LEDC_LS_SIG_OUT0_IDX + channel
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_output_enable(&GPIO, pin);
        esp_rom_gpio_connect_out_signal(pin, LEDC_LS_SIG_OUT0_IDX + ch, false, false);

        esp32PwmMotorCount++;
    }

    if (esp32PwmMotorCount == 0) return false;

    device->vTable = &pwmMotorVTable;
    device->count = esp32PwmMotorCount;
    device->initialized = true;

    return true;
}
