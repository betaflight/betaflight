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

#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/compass/compass.h"
#include "drivers/serial.h"

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "io/serial.h"

#include "telemetry/telemetry.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "config/feature.h"

#include "fc/config.h"

#ifdef USE_TARGET_CONFIG

#include "config_helper.h"

#define GPS_UART                            SERIAL_PORT_USART3
#define TELEMETRY_UART                      SERIAL_PORT_UART5

static targetSerialPortFunction_t targetSerialPortFunction[] = {
#ifdef USE_GPS
    { GPS_UART,       FUNCTION_GPS },
#endif
#ifdef USE_TELEMETRY
    { TELEMETRY_UART, FUNCTION_TELEMETRY_SMARTPORT },
#endif
#if !defined(USE_GPS) && !defined(USE_TELEMETRY)
    { SERIAL_PORT_NONE, FUNCTION_NONE },
#endif
};

void targetConfiguration(void)
{
    barometerConfigMutable()->baro_hardware = BARO_DEFAULT;

#ifdef USE_TELEMETRY
    targetSerialPortFunctionConfig(targetSerialPortFunction, ARRAYLEN(targetSerialPortFunction));
    // change telemetry settings
    telemetryConfigMutable()->telemetry_inverted = 1;
    telemetryConfigMutable()->halfDuplex = 1;
#endif
}
#endif
