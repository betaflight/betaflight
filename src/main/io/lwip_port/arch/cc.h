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
 * lwip compiler/architecture port for betaflight (gcc, arm cortex-m7, little-endian, no_sys).
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>

#define LWIP_NO_UNISTD_H        1
#define LWIP_TIMEVAL_PRIVATE    0

/* lwip diagnostics/asserts become no-ops, no console in phone-config mode */
#define LWIP_PLATFORM_DIAG(x)
#define LWIP_PLATFORM_ASSERT(x) do { } while (0)
