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

#include "platform.h"
#include "common/utils.h"
#include "drivers/timer_types.h"

// No generic timer channels; only the macro the common timer code references.
#if defined(USE_TIMER_MGMT)
#define TIMER_GET_IO_TAG(pin) DEFIO_TAG_E(pin)
#else
#define TIMER_GET_IO_TAG(pin) DEFIO_TAG(pin)
#endif
