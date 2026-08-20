/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// HPMicro IO helpers: 32-bit IOC alternate-function config and the logical
// port mapping that packs the X/Y/Z pin domains into ioTag_t.

#pragma once

#include "platform.h"
#include "drivers/io.h"
#include "drivers/io_def.h"
#include "hpm_ioc_regs.h"

// HPMicro variant of IOConfigGPIOAF: the alternate-function selector is a
// 32-bit IOC FUNC_CTL register value, not the STM32-style 4-bit AF number.
// Declared here (rather than drivers/io.h, whose USE_TIMER_AF-gated prototype
// uses uint8_t af) to keep the difference platform-local.
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint32_t af);

// To fit HPMicro ports into uint8_t ioTag_t, the special X/Y/Z domain is
// merged into one logical port (gpioID 6).  Within that port:
//   logical pins 0-7  -> PX0-7
//   logical pins 8-15 -> PY0-7
//   logical pins 16-23 -> PZ0-7
// The physical IOC PAD indices for X/Y/Z are taken directly from the HPM SDK
// hpm_ioc_regs.h macros.  A/B/C/D/E/F remain contiguous 32-pin ports starting
// at IOC PAD 0, matching IOC_PAD_PA00/PB00/.../PF00.

#define HPM_X_IOC_BASE IOC_PAD_PX00
#define HPM_Y_IOC_BASE IOC_PAD_PY00
#define HPM_Z_IOC_BASE IOC_PAD_PZ00

static inline uint32_t IOCIndex(IO_t io)
{
    return IO_Pin(io);
}
