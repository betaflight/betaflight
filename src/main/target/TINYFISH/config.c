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

#include <stdint.h>

#include <platform.h>

#ifdef USE_TARGET_CONFIG
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/timer.h"

#include "config/feature.h"

#include "fc/config.h"

#include "io/serial.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "telemetry/telemetry.h"

// set default settings to match our target
void targetConfiguration(void)
{
    // use the same uart for frsky telemetry and SBUS, both non inverted
    const int index = findSerialPortIndexByIdentifier(SBUS_TELEMETRY_UART);
    serialConfigMutable()->portConfigs[index].functionMask = FUNCTION_TELEMETRY_FRSKY_HUB | FUNCTION_RX_SERIAL;

    rxConfigMutable()->serialrx_provider = SERIALRX_SBUS;
    rxConfigMutable()->serialrx_inverted = true;
    telemetryConfigMutable()->telemetry_inverted = true;
}
#endif
