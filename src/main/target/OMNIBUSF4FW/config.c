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

#include <platform.h>

#ifdef USE_TARGET_CONFIG

#include "pg/pg.h"
#include "drivers/max7456.h"
#include "io/serial.h"

typedef struct targetSerialConfig_s {
    serialPortIdentifier_e identifier;
    serialPortFunction_e   function;    
} targetSerialConfig_t;

targetSerialConfig_t targetSerialConfig[] = {
    { SERIAL_PORT_USART1, FUNCTION_RX_SERIAL },
    { SERIAL_PORT_UART4,  FUNCTION_ESC_SENSOR },    
};

void targetConfiguration(void)
{
    for (unsigned i = 0 ; i < ARRAYLEN(targetSerialConfig) ; i++) {
        int index = findSerialPortIndexByIdentifier(targetSerialConfig[i].identifier);
        if (index >= 0) {
            serialConfigMutable()->portConfigs[index].functionMask = targetSerialConfig[i].function;
        }
    }
}
#endif
