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
#include "soc/soc_caps.h"
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

// LEDC has 8 channels on ESP32-S3 (and on the originals via the
// low-speed group); cap motors at 8 to fit hex/octo.
#define PWM_MAX_MOTORS 8

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

// Standard PWM works in microseconds, so the external (1000-2000us) value maps
// straight through. These MUST be non-NULL: motorConvertFromExternal() (used by
// the CLI `motor` command and the mixer) calls them via the vtable with no NULL
// guard, so a NULL here faults the FC the moment a motor is driven.
static float pwmMotorConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmMotorConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static const motorVTable_t pwmMotorVTable = {
    .postInit = pwmMotorPostInit,
    .convertExternalToMotor = pwmMotorConvertFromExternal,
    .convertMotorToExternal = pwmMotorConvertToExternal,
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

    // Select the low-speed timer clock source (80MHz APB). Without this the LS
    // timers have no source selected, so the dividers computed against APB are
    // wrong and the duty never latches.
    ledc_ll_set_slow_clk_sel(&LEDC, LEDC_SLOW_CLK_APB);

    ledcInitialized = true;
}

// The LEDC low-speed timer is clocked from the 80MHz APB clock (see ledcInit).
#define LEDC_APB_HZ 80000000u

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorDevConfig, uint16_t idlePulse)
{
    // The written motor value is already the absolute pulse figure (1000-2000),
    // so the disarm/idle pulse is produced by the mixer writing it each loop, not
    // by a per-write offset. pulseOffset is derived from the protocol geometry.
    UNUSED(idlePulse);

    ledcInit();

    uint32_t pwmRate = motorDevConfig->motorPwmRate;
    if (pwmRate == 0) pwmRate = 490;  // Default PWM rate

    // Pulse geometry per protocol (seconds), mirroring the STM32 driver: a motor
    // value of 1000-2000 maps linearly onto [sMin, sMin + sLen].
    float sMin;
    float sLen;
    switch (motorDevConfig->motorProtocol) {
    case MOTOR_PROTOCOL_ONESHOT125: sMin = 125e-6f; sLen = 125e-6f; break;
    case MOTOR_PROTOCOL_ONESHOT42:  sMin = 42e-6f;  sLen = 42e-6f;  break;
    case MOTOR_PROTOCOL_MULTISHOT:  sMin = 5e-6f;   sLen = 20e-6f;  break;
    case MOTOR_PROTOCOL_BRUSHED:    sMin = 0.0f;    sLen = 1.0f / pwmRate; break;
    case MOTOR_PROTOCOL_PWM:
    default:                        sMin = 1e-3f;   sLen = 1e-3f;   break;
    }

    // Pick the highest duty resolution whose timer divider stays >= 1.0.
    // freq = APB / (divider * 2^resolution); the LS timer divider register is
    // fixed point with LEDC_LL_FRACTIONAL_BITS (8) fractional bits, so the value
    // written is divider * 256. Resolution must not exceed the timer counter
    // width (SOC_LEDC_TIMER_BIT_WIDTH, 14 on the S3) - the conf field is narrow
    // and a too-large value wraps to 0, giving a 1-count period and no PWM.
    uint8_t resolution = SOC_LEDC_TIMER_BIT_WIDTH;
    uint32_t maxDuty = 1u << resolution;
    uint32_t dividerQ8 = ((uint64_t)LEDC_APB_HZ << 8) / ((uint64_t)pwmRate * maxDuty);
    while (dividerQ8 < (1u << 8) && resolution > 8) {  // divider < 1.0: drop resolution
        resolution--;
        maxDuty = 1u << resolution;
        dividerQ8 = ((uint64_t)LEDC_APB_HZ << 8) / ((uint64_t)pwmRate * maxDuty);
    }

    ledc_ll_set_clock_divider(&LEDC, LEDC_LOW_SPEED_MODE, 0, dividerQ8);
    ledc_ll_set_duty_resolution(&LEDC, LEDC_LOW_SPEED_MODE, 0, resolution);
    ledc_ll_timer_rst(&LEDC, LEDC_LOW_SPEED_MODE, 0);
    ledc_ll_ls_timer_update(&LEDC, LEDC_LOW_SPEED_MODE, 0);
    ledc_ll_timer_resume(&LEDC, LEDC_LOW_SPEED_MODE, 0);  // un-pause: the timer must run

    // Duty counts per second of high time = APB / actual_divider. This is the
    // direct analog of the STM32 timer "hz": duty = pulse_seconds * dutyHz, and
    // since value is in microseconds, duty = value * pulseScale + pulseOffset.
    const float dutyHz = (float)LEDC_APB_HZ * 256.0f / (float)dividerQ8;
    const float pulseScale = sLen * dutyHz / 1000.0f;
    const float pulseOffset = (sMin * dutyHz) - (pulseScale * 1000.0f);

    esp32PwmMotorCount = 0;

    for (int i = 0; i < PWM_MAX_MOTORS && i < MAX_SUPPORTED_MOTORS; i++) {
        const ioTag_t tag = motorDevConfig->ioTags[i];
        if (!tag) break;

        IO_t io = IOGetByTag(tag);
        if (!io) break;

        IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(i));
        IOConfigGPIO(io, IOCFG_OUT_PP);

        uint32_t pin = IO_Pin(io);
        uint8_t ch = i;  // LEDC channel 0..(PWM_MAX_MOTORS-1)

        esp32PwmMotors[i].io = io;
        esp32PwmMotors[i].ledcChannel = ch;
        esp32PwmMotors[i].enabled = false;
        esp32PwmMotors[i].pulseScale = pulseScale;
        esp32PwmMotors[i].pulseOffset = pulseOffset;

        // Bind channel to timer 0 and enable its output. Without sig_out_en the
        // channel never drives its signal, and the fade params must be set for a
        // static duty (scale 0 = no fade) or the loaded duty does not take effect.
        ledc_ll_bind_channel_timer(&LEDC, LEDC_LOW_SPEED_MODE, ch, 0);
        ledc_ll_set_hpoint(&LEDC, LEDC_LOW_SPEED_MODE, ch, 0);
        ledc_ll_set_duty_direction(&LEDC, LEDC_LOW_SPEED_MODE, ch, LEDC_DUTY_DIR_INCREASE);
        ledc_ll_set_duty_num(&LEDC, LEDC_LOW_SPEED_MODE, ch, 1);
        ledc_ll_set_duty_cycle(&LEDC, LEDC_LOW_SPEED_MODE, ch, 1);
        ledc_ll_set_duty_scale(&LEDC, LEDC_LOW_SPEED_MODE, ch, 0);
        ledc_ll_set_idle_level(&LEDC, LEDC_LOW_SPEED_MODE, ch, 0);
        ledc_ll_set_sig_out_en(&LEDC, LEDC_LOW_SPEED_MODE, ch, true);
        ledc_ll_set_duty_int_part(&LEDC, LEDC_LOW_SPEED_MODE, ch, 0);
        ledc_ll_set_duty_start(&LEDC, LEDC_LOW_SPEED_MODE, ch);
        ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, ch);

        // Connect LEDC output to GPIO
        // LEDC output signals: LEDC_LS_SIG_OUT0_IDX + channel
        esp_rom_gpio_pad_select_gpio(pin);
        gpio_ll_output_enable(&GPIO, pin);
        esp_rom_gpio_connect_out_signal(pin, ESP32_LEDC_LS_SIG_OUT0_IDX + ch, false, false);

        esp32PwmMotorCount++;
    }

    if (esp32PwmMotorCount == 0) return false;

    device->vTable = &pwmMotorVTable;
    device->count = esp32PwmMotorCount;
    device->initialized = true;

    return true;
}
