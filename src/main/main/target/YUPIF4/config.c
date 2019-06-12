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
#include "blackbox/blackbox.h"

#include "flight/pid.h"

#include "pg/adc.h"
#include "pg/beeper_dev.h"

#include "telemetry/telemetry.h"

#include "hardware_revision.h"

// BEEPER_OPT will be handled by post-flash configuration
#define BEEPER_OPT              PB14

// alternative defaults settings for YuPiF4 targets
void targetConfiguration(void)
{
    /* Changes depending on versions */
    if (hardwareRevision == YUPIF4_RACE3) {
            beeperDevConfigMutable()->ioTag = IO_TAG(BEEPER_OPT);
            telemetryConfigMutable()->halfDuplex = false;

    } else if (hardwareRevision == YUPIF4_RACE2) {
        beeperDevConfigMutable()->ioTag = IO_TAG(BEEPER_OPT);

    } else if (hardwareRevision == YUPIF4_MINI) {
        beeperDevConfigMutable()->frequency = 0;
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
        adcConfigMutable()->current.enabled = 0;

    } else if (hardwareRevision == YUPIF4_NAV) {
        beeperDevConfigMutable()->ioTag = IO_TAG(BEEPER_OPT);

    } else {
        adcConfigMutable()->current.enabled = 0;
    }

    /* Specific PID values for YupiF4 */
    for (uint8_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[PID_ROLL].P = 30;
        pidProfile->pid[PID_ROLL].I = 45;
        pidProfile->pid[PID_ROLL].D = 20;
        pidProfile->pid[PID_PITCH].P = 30;
        pidProfile->pid[PID_PITCH].I = 50;
        pidProfile->pid[PID_PITCH].D = 20;
        pidProfile->pid[PID_YAW].P = 40;
        pidProfile->pid[PID_YAW].I = 50;
    }
}
#endif
