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

#include "telemetry/telemetry.h"

#include "pg/pinio.h"
#include "pg/piniobox.h"

#include "hardware_revision.h"

#include "sensors/gyro.h"

#include "pg/gyrodev.h"

void targetConfiguration(void)
{	
    if (hardwareRevision == FF_RACEPIT_REV_1) {
        gyroDeviceConfigMutable(0)->align = CW180_DEG;
    }
    else {
        gyroDeviceConfigMutable(0)->align = CW90_DEG_FLIP;
    }

    telemetryConfigMutable()->halfDuplex = false;

    pinioConfigMutable()->config[1] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
	
    pinioBoxConfigMutable()->permanentId[0] = 40;
    pinioBoxConfigMutable()->permanentId[1] = 41;
}
#endif
