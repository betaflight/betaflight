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

#include "common/utils.h"

// return ioTag_t for given pinid
// tag for NONE must be false
#define DEFIO_TAG(pinid) CONCAT(DEFIO_TAG__, pinid)
#define DEFIO_TAG__NONE 0
#define DEFIO_TAG_E(pinid) CONCAT(DEFIO_TAG_E__, pinid)
#define DEFIO_TAG_E__NONE 0

// return ioRec_t or NULL for given pinid
// tags should be preferred, possibly removing it in future
// io_impl.h must be included when this macro is used
#define DEFIO_REC(pinid) CONCAT(DEFIO_REC__, pinid)
#define DEFIO_REC__NONE NULL

#define DEFIO_IO(pinid) (IO_t)DEFIO_REC(pinid)
// TODO - macro to check for pinid NONE (fully in preprocessor)

// get ioRec by index
#define DEFIO_REC_INDEXED(idx) (ioRecs + (idx))

// split ioTag bits between pin and port
// port is encoded as +1 to avoid collision with 0x0 (false as bool)
#ifndef DEFIO_PORT_PINS
// pins per port
#define DEFIO_PORT_PINS 16
#endif

STATIC_ASSERT((DEFIO_PORT_PINS & (DEFIO_PORT_PINS - 1)) == 0, "DEFIO_PORT_PINS must be power of 2");

#define DEFIO_PORT_BITSHIFT LOG2(DEFIO_PORT_PINS)
#define DEFIO_PIN_BITMASK   ((1 << DEFIO_PORT_BITSHIFT ) - 1)

// ioTag_t accessor macros
#define DEFIO_TAG_MAKE(gpioid, pin) ((ioTag_t)((((gpioid) + 1) << DEFIO_PORT_BITSHIFT) | (pin)))
#define DEFIO_TAG_ISEMPTY(tag) (!(tag))
#define DEFIO_TAG_GPIOID(tag) (((tag) >> DEFIO_PORT_BITSHIFT) - 1)
#define DEFIO_TAG_PIN(tag) ((tag) & DEFIO_PIN_BITMASK)


// TARGET must define used pins
#include "target.h"
// include template-generated macros for IO pins
#include "io_def_generated.h"
