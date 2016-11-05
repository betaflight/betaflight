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

#include "drivers/serial.h"

#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/motors.h"
#include "io/serial.h"

#include "rx/rx.h"

#include "config/config_profile.h"
#include "config/config_master.h"


// alternative defaults settings for Colibri/Gemini targets
void targetConfiguration(master_t *config)
{
    config->mixerMode = MIXER_HEX6X;
    config->rxConfig.serialrx_provider = 2;

    config->motorConfig.minthrottle = 1070;
    config->motorConfig.maxthrottle = 2000;

    config->boardAlignment.pitchDegrees = 10;
    //config->rcControlsConfig.deadband = 10;
    //config->rcControlsConfig.yaw_deadband = 10;
    config->mag_hardware = 1;

    config->profile[0].controlRateProfile[0].dynThrPID = 45;
    config->profile[0].controlRateProfile[0].tpa_breakpoint = 1700;
    config->serialConfig.portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
}
