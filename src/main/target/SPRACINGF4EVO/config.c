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

#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/compass.h"
#include "drivers/serial.h"

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "io/serial.h"

#include "telemetry/telemetry.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "config/config_master.h"
#include "config/feature.h"

#include "fc/config.h"

#ifdef TARGET_CONFIG
void targetConfiguration(master_t *config)
{
    UNUSED(config);

    barometerConfig()->baro_hardware = BARO_DEFAULT;
    rxConfig()->sbus_inversion = 1;
    serialConfig()->portConfigs[1].functionMask = FUNCTION_MSP; // So SPRacingF3OSD users don't have to change anything.
    serialConfig()->portConfigs[findSerialPortIndexByIdentifier(SERIALRX_UART)].functionMask = FUNCTION_RX_SERIAL;
    serialConfig()->portConfigs[findSerialPortIndexByIdentifier(TELEMETRY_UART)].functionMask = FUNCTION_TELEMETRY_SMARTPORT;
    telemetryConfig()->telemetry_inversion = 0;
    telemetryConfig()->sportHalfDuplex = 0;

}
#endif
