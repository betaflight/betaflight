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
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_PWM_OUTPUT

#include "build/debug.h"
#include "build/debug_pin.h"

#include "common/maths.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_impl.h"
#include "drivers/motor_types.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "pg/motor.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "platform/pwm.h"

#define TOPMAX (0xfffe) // maximum practical TOP value to allow for 0% to 100% duty cycle
#define LEVELMAX (TOPMAX + 1) // maximum comparison value

static picoPwmOutput_t picoPwmMotors[MAX_SUPPORTED_MOTORS];
static bool useContinuousUpdate = false;

void pwmShutdownPulsesForAllMotors(void)
{
    for (int index = 0; index < pwmMotorCount; index++) {
        picoPwmMotors[index].level = 0;
        pwm_set_chan_level(picoPwmMotors[index].slice, picoPwmMotors[index].channel, 0);
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

static void pwmWriteStandard(uint8_t index, float value)
{
    uint32_t level = constrain(lrintf((value * pwmMotors[index].pulseScale) + pwmMotors[index].pulseOffset), 0, LEVELMAX);
    if (useContinuousUpdate) {
        // Writes on a running slice latch at the next wrap (no glitches).
        pwm_set_chan_level(picoPwmMotors[index].slice, picoPwmMotors[index].channel, level);
    } else {
        // One-shot modes: The active compare level is typically 0 (between pulses) at this point.
        // Store the level to write it directly (unbuffered) while pwm stopped in pwmCompleteMotorUpdate.
        picoPwmMotors[index].level = level;
    }
}

static void pwmCompleteMotorUpdate(void)
{
    if (useContinuousUpdate) {
        return;
    }

    // One-shot modes: emit exactly one pulse per motor per update. Writes on a
    // running slice only latch at counter wrap, but are immediate when the counter is stopped.
    // So, write the levels with the slice stopped, zero the counter, then queue level 0 to latch
    // at the wrap (after the pulse ends).
    // Slices (which share counters across channels) are handled once each.
    for (int index = 0; index < pwmMotorCount; index++) {
        if (!picoPwmMotors[index].sliceHead) {
            continue;
        }
        const uint16_t slice = picoPwmMotors[index].slice;

        pwm_set_enabled(slice, false);
        for (int i = index; i < pwmMotorCount; i++) {
            if (picoPwmMotors[i].slice == slice) {
                pwm_set_chan_level(slice, picoPwmMotors[i].channel, picoPwmMotors[i].level);
            }
        }
        pwm_set_counter(slice, 0);
        pwm_set_enabled(slice, true);

        for (int i = index; i < pwmMotorCount; i++) {
            if (picoPwmMotors[i].slice == slice) {
                pwm_set_chan_level(slice, picoPwmMotors[i].channel, 0);
            }
        }
    }
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static motorVTable_t motorPwmVTable = {
    .postInit = NULL,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
    .write = pwmWriteStandard,
    .decodeTelemetry = NULL,
    .updateComplete = pwmCompleteMotorUpdate,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
    .getMotorIO = pwmGetMotorIO,
};

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t idlePulse)
{
    UNUSED(idlePulse);

    if (!device || !motorConfig) {
        return false;
    }

    memset(pwmMotors, 0, sizeof(pwmMotors));
    memset(picoPwmMotors, 0, sizeof(picoPwmMotors));

    pwmMotorCount = device->count;
    device->vTable = &motorPwmVTable;

    useContinuousUpdate = motorConfig->useContinuousUpdate;

    float sMin = 0;
    float sLen = 0;
    switch (motorConfig->motorProtocol) {
    default:
    case MOTOR_PROTOCOL_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case MOTOR_PROTOCOL_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case MOTOR_PROTOCOL_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case MOTOR_PROTOCOL_BRUSHED:
        sMin = 0;
        useContinuousUpdate = true;
        break;
    case MOTOR_PROTOCOL_PWM :
        sMin = 1e-3f;
        sLen = 1e-3f;
        useContinuousUpdate = true;
        break;
    }

    for (unsigned motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < pwmMotorCount; motorIndex++) {

        const unsigned reorderedMotorIndex = motorConfig->motorOutputReordering[motorIndex];
        const ioTag_t tag = motorConfig->ioTags[reorderedMotorIndex];

        pwmMotors[motorIndex].io = IOGetByTag(tag);
        if (!tag || !pwmMotors[motorIndex].io) {
            /* not enough motors initialised for the mixer or a break in the motors */
            device->vTable = NULL;
            pwmMotorCount = 0;
            /* TODO: block arming and add reason system cannot arm */
            return false;
        }

        uint8_t pin = IO_PINBYTAG(tag);

        const uint16_t slice = pwm_gpio_to_slice_num(pin);
        const uint16_t channel = pwm_gpio_to_channel(pin);

        IOInit(pwmMotors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));

        picoPwmMotors[motorIndex].slice = slice;
        picoPwmMotors[motorIndex].channel = channel;

        bool sliceAlreadyUsed = false;
        for (unsigned i = 0; i < motorIndex; i++) {
            if (picoPwmMotors[i].slice == slice) {
                sliceAlreadyUsed = true;
                break;
            }
        }
        picoPwmMotors[motorIndex].sliceHead = !sliceAlreadyUsed;

        /* standard PWM outputs */
        // margin of safety is 4 periods when not continuous
        const unsigned pwmRateHz = useContinuousUpdate ? motorConfig->motorPwmRate : ceilf(1 / ((sMin + sLen) * 4));

        /*
            PWM Frequency = clock / (clkdiv * (wrap + 1))

            wrap is the 16-bit counter top (duty-cycle resolution); clkdiv is the
            pre-divider. To maximise resolution, use the smallest clkdiv that still
            lets a full period fit inside the 16-bit wrap register, then let wrap
            take up whatever's left.
        */
        const uint32_t clock = SystemCoreClock; // PICO timer clock is the CPU clock.

        // Clock ticks needed for one period
        const float ticksPerPeriod = (float)clock / (float)pwmRateHz;

        uint32_t clkdiv = (uint32_t)ceilf(ticksPerPeriod / 0xffff);
        clkdiv = constrain(clkdiv, 1, 255); // (Extra safety, but clkdiv isn't exceeding 256, even if sys clock was high as 800MHz, pwm rate as low as 50Hz)

        const uint32_t hz = clock / clkdiv; // counter tick rate after the divider

        int32_t wrap;
        if (useContinuousUpdate) {
            int32_t period = lrintf(ticksPerPeriod / (float)clkdiv);
            wrap = constrain(period - 1, 0, TOPMAX);
        } else {
            wrap = TOPMAX;
        }

        pwm_config config = pwm_get_default_config();

        pwm_config_set_clkdiv_int(&config, clkdiv);
        pwm_config_set_wrap(&config, wrap);
        gpio_set_function(pin, GPIO_FUNC_PWM);

        pwm_set_chan_level(slice, channel, 0);
        pwm_init(slice, &config, true);

        // Brushed spans the full period; wrap + 1 gives true 100% duty at full throttle.
        pwmMotors[motorIndex].pulseScale = ((motorConfig->motorProtocol == MOTOR_PROTOCOL_BRUSHED) ? (wrap + 1) : (sLen * hz)) / 1000.0f;
        pwmMotors[motorIndex].pulseOffset = (sMin * hz) - (pwmMotors[motorIndex].pulseScale * 1000);
        pwmMotors[motorIndex].enabled = true;
        picoPwmMotors[motorIndex].initialised = true;
    }

    return true;
}

#endif // USE_PWM_OUTPUT
