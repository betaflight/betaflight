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

/*
 * Weak stub functions for STM32C5 HAL2 bringup.
 *
 * These provide linker-resolvable symbols for peripheral drivers that
 * have not yet been ported to the HAL2 API. Each stub will be removed
 * as the corresponding HAL2 driver implementation is added.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/color.h"
#include "drivers/light_ws2811strip.h"

#define STUB __attribute__((weak))

/* ---- LED strip ---- */

STUB bool ws2811LedStripHardwareInit(void) { return false; }
STUB void ws2811LedStripStartTransfer(void) {}
