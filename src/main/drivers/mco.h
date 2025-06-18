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

#include "pg/mco.h"

typedef enum {
    MCODEV_1 = 0,
    MCODEV_2,
} MCODevice_e;

#ifdef STM32G4
#define MCO_SOURCE_COUNT   8
#define MCO_DIVIDER_COUNT  5
#endif

void mcoInit(void);
void mcoConfigure(MCODevice_e device, const mcoConfig_t *config);
