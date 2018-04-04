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

#include "platform.h"
#ifdef USE_BEGINNER_MODE
#include "common/maths.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "fc/fc_core.h"
#include "io/serial.h"
#include "beginner_mode.h"

PG_REGISTER_WITH_RESET_TEMPLATE(beginnerModeConfig_t, beginnerModeConfig, PG_BEGINNER_MODE_CONFIG, 0);
PG_RESET_TEMPLATE(beginnerModeConfig_t, beginnerModeConfig,
    .enabled_beginnerMode = 0,
    .maxRoll = 10,
    .maxPitch = 10
);

void beginnerModeHandleAttitude(float roll, float pitch, float yaw)
{
    UNUSED(yaw);
    roll = ABS(roll);
    pitch = ABS(pitch);
    if (((beginnerModeConfig()->enabled_beginnerMode & BEGINNER_MODE_ROLL) && roll >= beginnerModeConfig()->maxRoll) ||
            ((beginnerModeConfig()->enabled_beginnerMode & BEGINNER_MODE_PITCH) && pitch >= beginnerModeConfig()->maxPitch))
    {
        disarm();
    }
}

#endif //USE_BEGINNER_MODE