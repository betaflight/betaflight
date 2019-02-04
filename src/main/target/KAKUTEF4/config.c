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

#include "config_helper.h"
#include "io/serial.h"
#include "pg/max7456.h"
#include "telemetry/telemetry.h"

#define TELEMETRY_UART          SERIAL_PORT_USART1

static targetSerialPortFunction_t targetSerialPortFunction[] = {
    { TELEMETRY_UART, FUNCTION_TELEMETRY_SMARTPORT },
};

void targetConfiguration(void)
{
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
    telemetryConfigMutable()->halfDuplex = 0;
    telemetryConfigMutable()->telemetry_inverted = true;

    // Mark MAX7456 CS pin as OPU
    max7456ConfigMutable()->preInitOPU = true;
}
#endif
