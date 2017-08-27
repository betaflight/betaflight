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

#ifdef TARGET_CONFIG
#include "fc/config.h"

#include "flight/pid.h"


// alternative defaults settings for YuPiF4 targets
void targetConfiguration(void)
{
    /* Specific PID values for YupiF4 */
    pidProfilesMutable(0)->pid[PID_ROLL].P = 30;
    pidProfilesMutable(0)->pid[PID_ROLL].I = 45;
    pidProfilesMutable(0)->pid[PID_ROLL].D = 20;
    pidProfilesMutable(0)->pid[PID_PITCH].P = 30;
    pidProfilesMutable(0)->pid[PID_PITCH].I = 50;
    pidProfilesMutable(0)->pid[PID_PITCH].D = 20;
    pidProfilesMutable(0)->pid[PID_YAW].P = 40;
    pidProfilesMutable(0)->pid[PID_YAW].I = 50;
}
#endif
