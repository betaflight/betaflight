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

#include "pg/max7456.h"
#include "pg/pg.h"

void targetConfiguration(void)
{
#ifdef OMNIBUSF4BASE
    // OMNIBUS F4 AIO (1st gen) has a AB7456 chip that is detected as MAX7456
    max7456ConfigMutable()->clockConfig = MAX7456_CLOCK_CONFIG_FULL;
#endif

#ifdef EXUAVF4PRO
    serialConfigMutable()->portConfigs[1].functionMask = FUNCTION_TELEMETRY_SMARTPORT;
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_VTX_TRAMP;
    serialConfigMutable()->portConfigs[3].functionMask = FUNCTION_RCDEVICE;
    serialConfigMutable()->portConfigs[4].functionMask = FUNCTION_RX_SERIAL;
#endif
}
#endif
