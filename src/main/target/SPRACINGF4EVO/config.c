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

#ifdef USE_TARGET_CONFIG

#include "io/serial.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/barometer.h"

#include "telemetry/telemetry.h"

#include "config_helper.h"

#define TELEMETRY_UART                      SERIAL_PORT_UART5

static targetSerialPortFunction_t targetSerialPortFunction[] = {
    { SERIAL_PORT_USART1, FUNCTION_MSP                 },  // So SPRacingF3OSD users don't have to change anything.
    { TELEMETRY_UART,     FUNCTION_TELEMETRY_SMARTPORT },
};

void targetConfiguration(void)
{
    barometerConfigMutable()->baro_hardware = BARO_DEFAULT;
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
    telemetryConfigMutable()->halfDuplex = 0;
    telemetryConfigMutable()->telemetry_inverted = true;
}
#endif
