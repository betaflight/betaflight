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
#include "drivers/io_types.h"
#include "drivers/resource.h"
#define IO_TAG(pin) (pin)
IO_t IOGetByTag(ioTag_t tag);
void IOConfigGPIO(IO_t io, ioConfig_t config);
void IOConfigGPIOAF(IO_t io, ioConfig_t config, uint8_t af);
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index);
