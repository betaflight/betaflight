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

#ifdef USE_SERVOS

#include "common/utils.h"

#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(tricopterMixerConfig_t, tricopterMixerConfig, PG_TRICOPTER_CONFIG, 0);

PG_RESET_TEMPLATE(tricopterMixerConfig_t, tricopterMixerConfig,
    .dummy = 0
);

#define TRICOPTER_ERROR_RATE_YAW_SATURATED 75 // rate at which tricopter yaw axis becomes saturated, determined experimentally by TriFlight

bool mixerTricopterIsServoSaturated(float errorRate)
{
    return errorRate > TRICOPTER_ERROR_RATE_YAW_SATURATED;
}

float mixerTricopterMotorCorrection(int motor)
{
    UNUSED(motor);
    return 0.0f;
}

void mixerTricopterInit(void)
{

}

#endif // USE_SERVOS
