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

#include "drivers/serial.h"

#include "telemetry/telemetry.h"

#include "fc/config.h"

#ifdef USE_TARGET_CONFIG
void targetConfiguration(void)
{
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(GPS_UART)].functionMask = FUNCTION_GPS;
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(TELEMETRY_UART)].functionMask = TELEMETRY_PROVIDER_DEFAULT;
    telemetryConfigMutable()->halfDuplex = false;
}
#endif
