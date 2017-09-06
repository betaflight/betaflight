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

#ifdef TARGET_CONFIG

#include "io/serial.h"
#include "rx/rx.h"
#include "sensors/barometer.h"
#include "telemetry/telemetry.h"


void targetConfiguration(void)
{
    barometerConfigMutable()->baro_hardware = BARO_BMP280;
    rxConfigMutable()->sbus_inversion = 1;
    serialConfigMutable()->portConfigs[1].functionMask = FUNCTION_MSP; // So SPRacingF3OSD users don't have to change anything.
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(TELEMETRY_UART)].functionMask = FUNCTION_TELEMETRY_SMARTPORT;
    telemetryConfigMutable()->telemetry_inversion = true;
}
#endif
