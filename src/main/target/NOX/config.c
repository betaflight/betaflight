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

#include "build/debug.h"

void targetConfiguration(void)
{
    // SOFTSERIAL1 is ESC telemetry input
    const int index = findSerialPortIndexByIdentifier(SERIAL_PORT_SOFTSERIAL1);
    if (index >= 0) {
        serialConfigMutable()->portConfigs[index].functionMask = FUNCTION_ESC_SENSOR;
    }
}
#endif
