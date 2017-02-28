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

#include "platform.h"

#include "config/feature.h"

#include "fc/config.h"

#include "io/serial.h"

#include "rx/rx.h"

#include "sensors/gyro.h"


void targetConfiguration(void)
{
    gyroConfigMutable()->looptime = 1000;

    rxConfigMutable()->rcmap[0] = 1;
    rxConfigMutable()->rcmap[1] = 2;
    rxConfigMutable()->rcmap[2] = 3;
    rxConfigMutable()->rcmap[3] = 0;
    rxConfigMutable()->rcmap[4] = 4;
    rxConfigMutable()->rcmap[5] = 5;
    rxConfigMutable()->rcmap[6] = 6;
    rxConfigMutable()->rcmap[7] = 7;

    featureSet(FEATURE_VBAT);
    featureSet(FEATURE_LED_STRIP);

    serialConfigMutable()->portConfigs[0].functionMask = FUNCTION_MSP;
    if (featureConfigured(FEATURE_RX_SERIAL)) {
        serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    }
}
