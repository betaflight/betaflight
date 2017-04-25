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
#include <stdbool.h>
#include <string.h>

#include <platform.h>

#ifdef TARGET_CONFIG

#include "blackbox/blackbox.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "io/ledstrip.h"
#include "io/serial.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"


void targetApplyDefaultLedStripConfig(ledConfig_t *ledConfigs)
{
	UNUSED(ledConfigs);
}

// alternative defaults settings for COLIBRI RACE targets
void targetConfiguration(void)
{
}

void targetValidateConfiguration()
{
}
#endif
