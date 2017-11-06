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

#include "io/serial.h"

void targetConfiguration(void)
{
// OMNIBUS F7 V2 has an option to connect UART7_RX to ESC telemetry
#if defined(OMNIBUSF7V2) && defined(ESC_SENSOR_UART)
    serialPortConfig_t *serialEscSensorUartConfig = serialFindPortConfiguration(ESC_SENSOR_UART);
    if (serialEscSensorUartConfig) {
        serialEscSensorUartConfig->functionMask = FUNCTION_ESC_SENSOR;
    }
#endif
}
#endif
