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

#include "config/feature.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "fc/config.h"

#include "io/serial.h"

#include "telemetry/telemetry.h"

#include "hardware_revision.h"

#define UART1_INVERTER          PC9

void targetPreInit(void)
{
    switch (hardwareRevision) {
    case BJF4_REV3:
    case BJF4_MINI_REV3A:
    case BJF4_REV4:
        break;
    default:
        return;
    }

    IO_t inverter = IOGetByTag(IO_TAG(UART1_INVERTER));
    IOInit(inverter, OWNER_INVERTER, 1);
    IOConfigGPIO(inverter, IOCFG_OUT_PP);

    bool high = false;
    serialPortConfig_t *portConfig = serialFindPortConfiguration(SERIAL_PORT_USART1);
    if (portConfig) {
        bool smartportEnabled = (portConfig->functionMask & FUNCTION_TELEMETRY_SMARTPORT);
        if (smartportEnabled && (!telemetryConfig()->telemetry_inverted) && (featureIsEnabled(FEATURE_TELEMETRY))) {
            high = true;
        }
    }
    /* reverse this for rev4, as it does not use the XOR gate */
    if (hardwareRevision == BJF4_REV4) {
        high = !high;
    }
    IOWrite(inverter, high);

    /* ensure the CS pin for the flash is pulled hi so any SD card initialisation does not impact the chip */
    if (hardwareRevision == BJF4_REV3) {
        IO_t flashIo = IOGetByTag(IO_TAG(FLASH_CS_PIN));
        IOConfigGPIO(flashIo, IOCFG_OUT_PP);
        IOHi(flashIo);

        IO_t sdcardIo = IOGetByTag(IO_TAG(SDCARD_SPI_CS_PIN));
        IOConfigGPIO(sdcardIo, IOCFG_OUT_PP);
        IOHi(sdcardIo);
    }
}
