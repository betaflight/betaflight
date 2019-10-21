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

#ifdef USE_TIMER_MGMT

typedef struct timerIOConfig_s {
    ioTag_t ioTag;
    uint8_t index;
    int8_t dmaopt;
} timerIOConfig_t;

PG_DECLARE_ARRAY(timerIOConfig_t, MAX_TIMER_PINMAP_COUNT, timerIOConfig);

#endif
