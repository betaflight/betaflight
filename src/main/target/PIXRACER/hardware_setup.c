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
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "drivers/time.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

void initialisePreBootHardware(void)
{
    // VDD_3V3_SENSORS_EN
    // On Chinese clone from BangGood VDD_3V3_SENSORS seems to be always enabled, but just in case the code is run on original board - init and pull high
    IOInit(DEFIO_IO(PE3), OWNER_SYSTEM, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(DEFIO_IO(PE3), IOCFG_OUT_PP);
    IOLo(DEFIO_IO(PE3));
    delay(100);
    IOHi(DEFIO_IO(PE3));

    // VDD_3V3_PERIPH_EN - Enables 3V3 for 8266 and Spektrum
    IOInit(DEFIO_IO(PC5), OWNER_SYSTEM, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(DEFIO_IO(PC5), IOCFG_OUT_PP);
    IOHi(DEFIO_IO(PC5));

    // SPEKTRUM_POWER - Enables 3V3 power output for RC connector
    IOInit(DEFIO_IO(PE4), OWNER_SYSTEM, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(DEFIO_IO(PE4), IOCFG_OUT_PP);
    IOHi(DEFIO_IO(PE4));
}