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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "blackbox/blackbox.h"

#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/io.h"

#include "sensors/gyro.h"

#include "pg/adc.h"
#include "pg/beeper_dev.h"
#include "pg/gyrodev.h"

#include "hardware_revision.h"

// BEEPER_OPT will be handled by post-flash configuration
#define BEEPER_OPT              PB7

// alternative defaults settings for BlueJayF4 targets
void targetConfiguration(void)
{
    if (hardwareRevision == BJF4_REV1 || hardwareRevision == BJF4_REV2) {
        gyroDeviceConfigMutable(0)->align = CW180_DEG;
        beeperDevConfigMutable()->ioTag = IO_TAG(BEEPER_OPT);
    }

    if (hardwareRevision == BJF4_MINI_REV3A || hardwareRevision == BJF4_REV1) {
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
    }

    if (hardwareRevision == BJF4_MINI_REV3A) {
        adcConfigMutable()->vbat.ioTag = IO_TAG(PA4);
    }
}

void targetValidateConfiguration(void)
{
    /* make sure the SDCARD cannot be turned on */
    if (hardwareRevision == BJF4_MINI_REV3A || hardwareRevision == BJF4_REV1) {
        if (blackboxConfig()->device == BLACKBOX_DEVICE_SDCARD) {
            blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
        }
    }
}
#endif
