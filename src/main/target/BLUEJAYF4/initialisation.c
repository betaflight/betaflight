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
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "hardware_revision.h"
#include "config/config_master.h"
#include "drivers/io.h"
#include "config/feature.h"

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
        if (smartportEnabled && (telemetryConfig()->telemetry_inversion) && (feature(FEATURE_TELEMETRY))) {
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
        IO_t io = IOGetByTag(IO_TAG(M25P16_CS_PIN));
        IOConfigGPIO(io, IOCFG_OUT_PP);
        IOHi(io);
    }
}

