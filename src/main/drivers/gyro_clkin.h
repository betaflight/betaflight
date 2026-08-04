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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/io_types.h"

// Drive the supplied pin with a 50% duty-cycle square wave at freqHz, suitable
// for clocking an external sensor (e.g. ICM-426xx CLKIN). Implemented per
// platform — STM32 uses its timer/PWM abstraction, PICO uses the pico-sdk PWM
// hardware directly. Returns true on success, false if the pin can't be
// driven at the requested frequency or resources can't be allocated.
bool gyroClkInInit(ioTag_t tag, uint32_t freqHz, uint8_t resourceIndex);
