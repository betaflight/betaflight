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

// Tenths of a % of tasks late
#define CPU_LOAD_LATE_LIMIT 10

typedef struct schedulerConfig_s {
    uint16_t rxRelaxDeterminism;
    uint16_t osdRelaxDeterminism;
    uint16_t cpuLatePercentageLimit;
    uint8_t debugTask;
} schedulerConfig_t;

PG_DECLARE(schedulerConfig_t, schedulerConfig);

