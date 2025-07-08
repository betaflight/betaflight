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

#pragma once

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/timer.h" // For HARDWARE_TIMER_DEFINITION_COUNT

#if defined(USE_TIMER_MGMT) && defined(USE_TIMER_UP_CONFIG)

#if defined(USED_TIMERS) && defined(TIMUP_TIMERS)
    STATIC_ASSERT((~USED_TIMERS  & TIMUP_TIMERS) == 0, "All TIMUP timers must be used");
#endif

typedef struct timerUpConfig_s {
    int8_t dmaopt;
} timerUpConfig_t;

PG_DECLARE_ARRAY(timerUpConfig_t, HARDWARE_TIMER_DEFINITION_COUNT, timerUpConfig);

#endif
