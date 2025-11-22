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

#if defined(USE_SERVOS)

#include <math.h>
#include "hardware/pwm.h"

#include "drivers/servo_impl.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/resource.h"

// Standard servo PWM frequency
#define SERVO_PWM_FREQUENCY_HZ 50.0f // 50 Hz (20ms period)

// We set a prescaler of 64 to keep the TOP count (WRAP) within the 16-bit register.
#define PWM_PRESCALER 64.0f 
// Calculated TOP count for 50 Hz: (125M / 50) / 64 = 39062.5. We use 39063 for integer WRAP.
#define PWM_TOP_COUNT ((uint16_t)roundf((SYS_CLK_HZ / SERVO_PWM_FREQUENCY_HZ) / PWM_PRESCALER)) // 39063

// Factor to convert Microseconds (us) to PWM counter counts.
// Counts = us * US_TO_COUNTS_FACTOR
// US_TO_COUNTS_FACTOR = (SYS_CLK_HZ / PWM_PRESCALER) / 1,000,000
#define US_TO_COUNTS_FACTOR (SYS_CLK_HZ / (PWM_PRESCALER * 1000000.0f)) // ~1.953 counts/us

typedef struct picoPwmServos_s {
    uint16_t slice;
    uint16_t channel;
} picoPwmServos_t;

static picoPwmServos_t picoPwmServos[MAX_SUPPORTED_SERVOS];

void servoDevInit(const servoDevConfig_t *servoDevConfig)
{
    if (!servoDevConfig) {
        return;
    }

    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const ioTag_t tag = servoDevConfig->ioTags[i];
        IO_t servoIO = IOGetByTag(tag);

        if (!servoIO) {
            continue;
        }

        const uint8_t pin = IO_GPIOPinIdx(servoIO);
        IOInit(servoIO, OWNER_SERVO, i);

        const uint8_t slice = pwm_gpio_to_slice_num(pin);
        const uint8_t channel = pwm_gpio_to_channel(pin);

        // Store the hardware mapping for fast access in servoWrite
        picoPwmServos[i].slice = slice;
        picoPwmServos[i].channel = channel;

        gpio_set_function(pin, GPIO_FUNC_PWM);
        
        // Configure the PWM slice frequency (50 Hz)
        pwm_set_clkdiv(slice, PWM_PRESCALER);
        pwm_set_wrap(slice, PWM_TOP_COUNT);

        // Set initial neutral position (using 'mid' value from config, typically 1500 us)
        const uint16_t neutral_pulse_us = servoDevConfig->servoCenterPulse; 
        const uint16_t initial_level_counts = (uint16_t)roundf((float)neutral_pulse_us * US_TO_COUNTS_FACTOR);
        
        pwm_set_chan_level(slice, channel, initial_level_counts);
        
        // Enable the PWM slice
        pwm_set_enabled(slice, true);
    }
}

void servoWrite(uint8_t index, float value)
{
    if (index >= MAX_SUPPORTED_SERVOS) {
        return;
    }

    // Ensure value is within a reasonable microsecond range (500us to 2500us)
    // to prevent hardware overflow or out-of-spec pulses.
    float clamped_value = fmaxf(PWM_SERVO_MIN, fminf(PWM_SERVO_MAX, value));

    // Convert the microsecond pulse width to PWM duty cycle counts
    uint16_t level = (uint16_t)roundf(clamped_value * US_TO_COUNTS_FACTOR);

    // Apply the new duty cycle level to the specific PWM channel
    pwm_set_chan_level(picoPwmServos[index].slice, picoPwmServos[index].channel, level);
}

#endif
