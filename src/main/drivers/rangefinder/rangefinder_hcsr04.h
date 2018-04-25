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
#include "common/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "sensors/battery.h"

#define RANGEFINDER_HCSR04_TASK_PERIOD_MS 70

typedef struct sonarConfig_s {
    ioTag_t triggerTag;
    ioTag_t echoTag;
} sonarConfig_t;

PG_DECLARE(sonarConfig_t, sonarConfig);

bool hcsr04Detect(rangefinderDev_t *dev, const sonarConfig_t * sonarConfig);
