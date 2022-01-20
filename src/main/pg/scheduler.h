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

#ifdef STM32F411xE
// Allow RX and OSD tasks to be scheduled at the second attempt on F411 processors
#define SCHEDULER_RELAX_RX  1
#define SCHEDULER_RELAX_OSD 1
#else
#define SCHEDULER_RELAX_RX  25
#define SCHEDULER_RELAX_OSD 25
#endif

typedef struct schedulerConfig_s {
    uint16_t rxRelaxDeterminism;
    uint16_t osdRelaxDeterminism;
} schedulerConfig_t;

PG_DECLARE(schedulerConfig_t, schedulerConfig);

