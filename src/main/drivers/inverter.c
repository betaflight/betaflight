/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef INVERTER

#include "io.h"
#include "io_impl.h"

#include "inverter.h"

static const IO_t pin = DEFIO_IO(INVERTER);

void initInverter(void)
{
    IOInit(pin, OWNER_INVERTER, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(pin, IOCFG_OUT_PP);

    inverterSet(false);
}

void inverterSet(bool on)
{
    IOWrite(pin, on);
}

#endif
