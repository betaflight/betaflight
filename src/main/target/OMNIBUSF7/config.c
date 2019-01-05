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

#ifdef FPVM_BETAFLIGHTF7
#define ESC_SENSOR_UART         SERIAL_PORT_USART1
#elif defined(OMNIBUSF7V2)
#define ESC_SENSOR_UART         SERIAL_PORT_USART7
#endif

static targetSerialPortFunction_t targetSerialPortFunction[] = {
#ifdef ESC_SENSOR_UART
    { ESC_SENSOR_UART, FUNCTION_ESC_SENSOR },
#else
    { SERIAL_PORT_NONE, FUNCTION_NONE },
#endif
};

void targetConfiguration(void)
{
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
}
#endif
