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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_BST

#include "bst.h"
#include "bus_bst.h"
#include "pg/bus_i2c.h"
#include "pg/bus_spi.h"

// XXX Requires some additional work here.
// XXX Can't do this now without proper semantics about I2C on this target.
void targetBusInit(void)
{
#ifdef USE_SPI
    spiPinConfigure(spiPinConfig(0));
#ifdef USE_SPI_DEVICE_1
    spiInit(SPIDEV_1);
#endif
#endif

    i2cHardwareConfigure(i2cConfig(0));
    i2cInit(I2CDEV_2);

    bstInit(BST_DEVICE);
}
#endif
