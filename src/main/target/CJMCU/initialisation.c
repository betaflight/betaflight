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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"

#include "io/serial.h"

#include "pg/bus_i2c.h"
#include "pg/bus_spi.h"

#include "sensors/initialisation.h"

void targetBusInit(void)
{
#if defined(USE_SPI) && defined(USE_SPI_DEVICE_1)
    spiPinConfigure(spiPinConfig(0));
    sensorsPreInit();
    spiPreinit();
    spiInit(SPIDEV_1);
#endif

    if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) {
        serialRemovePort(SERIAL_PORT_USART3);
        i2cHardwareConfigure(i2cConfig(0));
        i2cInit(I2C_DEVICE);
    }
}
