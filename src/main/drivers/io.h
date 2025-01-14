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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "resource.h"

#include "drivers/io_types.h"

// preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred

// expand pinid to to ioTag_t
#define IO_TAG(pinid) DEFIO_TAG(pinid)

#define GPIO_STATE_HIGH 1
#define GPIO_STATE_LOW 0

#define GPIO_CONFIG_OUTPUT(pin, mode, state) \
    { IO_TAG(pin), mode, state, true }

#define GPIO_CONFIG_INPUT(pin, mode) \
    { IO_TAG(pin), mode, false, false }

// declare available IO pins. Available pins are specified per target
#include "io_def.h"

bool IORead(IO_t io);
void IOWrite(IO_t io, bool value);
void IOHi(IO_t io);
void IOLo(IO_t io);
void IOToggle(IO_t io);

void IOInit(IO_t io, resourceOwner_e owner, uint8_t index);
void IORelease(IO_t io);  // unimplemented
resourceOwner_e IOGetOwner(IO_t io);
bool IOIsFreeOrPreinit(IO_t io);
IO_t IOGetByTag(ioTag_t tag);

void IOConfigGPIO(IO_t io, ioConfig_t cfg);
#ifdef USE_TIMER_AF
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);
#endif

void IOInitGlobal(void);
void IOInitGPIODefault(void);

typedef void (*IOTraverseFuncPtr_t)(IO_t io);

void IOTraversePins(IOTraverseFuncPtr_t func);

GPIO_TypeDef* IO_GPIO(IO_t io);
uint16_t IO_Pin(IO_t io);
