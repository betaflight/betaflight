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
#include <string.h>

#include <platform.h>

#include "build/build_config.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/serial.h"
#include "drivers/system.h"

#include "osd/osd_serial.h"

#include "io/serial.h"

PG_REGISTER_WITH_RESET_FN(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);

void pgResetFn_serialConfig(serialConfig_t *serialConfig)
{
    memset(serialConfig, 0, sizeof(serialConfig_t));

    serialPortConfig_t portConfig_Reset = {
        .baudRates = {
                BAUD_115200,
                BAUD_115200
        }
    };

    for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
        memcpy(&serialConfig->portConfigs[i], &portConfig_Reset, sizeof(serialConfig->portConfigs[i]));
        serialConfig->portConfigs[i].identifier = serialPortIdentifiers[i];
    }

    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP_SERVER; // for USB/UART configuration.
    serialConfig->portConfigs[1].functionMask = FUNCTION_MSP_CLIENT; // for communication with FC.

    serialConfig->reboot_character = 'R';
}

void evaluateOtherData(serialPort_t *serialPort, uint8_t receivedChar)
{
    UNUSED(serialPort);

    if (receivedChar == serialConfig()->reboot_character) {
        systemResetToBootloader();
    }
}
