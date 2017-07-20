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

#include "drivers/io_impl.h"

#ifdef USE_HARDWARE_PREBOOT_SETUP
void initialisePreBootHardware(void)
{
    // Pulling down OSD switch pin
    IOInit(DEFIO_IO(OSD_CH_SWITCH), OWNER_SYSTEM, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(DEFIO_IO(OSD_CH_SWITCH), IOCFG_OUT_PP);
    IOLo(DEFIO_IO(OSD_CH_SWITCH));

    // Inverting the UART1 port by default
    IOInit(DEFIO_IO(PB13), OWNER_SYSTEM, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(DEFIO_IO(PB13), IOCFG_OUT_PP);
    IOHi(DEFIO_IO(PB13));
}
#endif
