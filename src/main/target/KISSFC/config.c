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

#include "common/utils.h"

#include "drivers/io.h"

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "config/config_profile.h"
#include "config/config_master.h"

#include "sensors/boardalignment.h"


void targetConfiguration(master_t *config)
{
    UNUSED(config);

#ifdef KISSCC
    // alternative defaults settings for Beebrain target
    config->boardAlignment.rollDegrees = 180;
    config->boardAlignment.pitchDegrees = 0;
    config->boardAlignment.yawDegrees = 0;
#endif
}

