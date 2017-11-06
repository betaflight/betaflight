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

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"

#include "drivers/serial.h"

#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/serial.h"

#include "rx/rx.h"

#include "sensors/boardalignment.h"
#include "sensors/compass.h"


// alternative defaults settings for Colibri/Gemini targets
void targetConfiguration(void)
{
    mixerConfigMutable()->mixerMode = MIXER_HEX6X;
    rxConfigMutable()->serialrx_provider = 2;

    motorConfigMutable()->minthrottle = 1070;
    motorConfigMutable()->maxthrottle = 2000;

    boardAlignmentMutable()->pitchDegrees = 10;
    //rcControlsConfigMutable()->deadband = 10;
    //rcControlsConfigMutable()->yaw_deadband = 10;
    compassConfigMutable()->mag_hardware = 1;

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        controlRateConfig->dynThrPID = 45;
        controlRateConfig->tpa_breakpoint = 1700;
    }

    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
}
#endif
