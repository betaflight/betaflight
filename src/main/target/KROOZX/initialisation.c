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
#include "drivers/io.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"

void targetBusInit(void)
{
#ifdef USE_SPI
	#ifdef USE_SPI_DEVICE_1
		spiInit(SPIDEV_1);
	#endif
	#ifdef USE_SPI_DEVICE_2
		spiInit(SPIDEV_2);
	#endif
	#ifdef USE_SPI_DEVICE_3
		spiInit(SPIDEV_3);
	#endif
	#ifdef USE_SPI_DEVICE_4
		spiInit(SPIDEV_4);
	#endif
#endif

#ifdef USE_I2C
    #ifdef USE_I2C_DEVICE_1
        i2cInit(I2CDEV_1);
    #endif
    #ifdef USE_I2C_DEVICE_3
        i2cInit(I2CDEV_3);
    #endif
#endif
}

void targetPreInit(void)
{
	IO_t osdChSwitch = IOGetByTag(IO_TAG(OSD_CH_SWITCH));
    IOInit(osdChSwitch, OWNER_SYSTEM, 0);
    IOConfigGPIO(osdChSwitch, IOCFG_OUT_PP);
	IOLo(osdChSwitch);
}

