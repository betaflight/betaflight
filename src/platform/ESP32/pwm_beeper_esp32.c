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

#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"

// Stub: beeper PWM output
// TODO: implement using LEDC peripheral

void pwmWriteBeeper(bool onoff)
{
    UNUSED(onoff);
}

void pwmToggleBeeper(void)
{
    // NOOP
}

void beeperPwmInit(const ioTag_t tag, uint16_t frequency)
{
    UNUSED(tag);
    UNUSED(frequency);
}
