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

#ifdef TARGET_CONFIG
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/timer.h"

#include "config/feature.h"

#include "fc/config.h"

#include "io/serial.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "telemetry/telemetry.h"

#define TARGET_CPU_VOLTAGE 3.0

// set default settings to match our target
void targetConfiguration(void)
{
    batteryConfigMutable()->currentMeterOffset = 0;
    // we use an ina139, RL=0.005, Rs=30000
    // V/A = (0.005 * 0.001 * 30000) * I
    // rescale to 1/10th mV / A -> * 1000 * 10
    // we use 3.0V as cpu and adc voltage -> rescale by 3.0/3.3
    batteryConfigMutable()->currentMeterScale  = (0.005 * 0.001 * 30000) * 1000 * 10 * (TARGET_CPU_VOLTAGE / 3.3);

    // we use the same uart for frsky telemetry and SBUS, both non inverted
    const int index = findSerialPortIndexByIdentifier(SBUS_TELEMETRY_UART);
    serialConfigMutable()->portConfigs[index].functionMask = FUNCTION_TELEMETRY_FRSKY | FUNCTION_RX_SERIAL;

    rxConfigMutable()->serialrx_provider = SERIALRX_SBUS;
    rxConfigMutable()->sbus_inversion = 0;
    telemetryConfigMutable()->telemetry_inversion = 0;

    featureSet(FEATURE_CURRENT_METER | FEATURE_VBAT);
}
#endif
