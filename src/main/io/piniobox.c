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

#include "stdbool.h"
#include "stdint.h"

#include <platform.h>

#ifdef USE_PINIOBOX

#include "build/debug.h"

#include "common/time.h"
#include "common/utils.h"
#include "drivers/pinio.h"
#include "fc/rc_modes.h"

boxId_e pinioBox[MAX_PINIO] = { BOXPINIO1, BOXPINIO2, BOXPINIO3, BOXPINIO4 };

void pinioBoxUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    for (int i = 0; i < MAX_PINIO; i++) {
        pinioSet(i, IS_RC_MODE_ACTIVE(pinioBox[i]));
    }
}
#endif
