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

#include <stdbool.h>
#include <stdint.h>

typedef struct propwashControlConfig_s
{
    uint8_t propwash_control_sensitivity;
    uint8_t propwash_control_d_boost;
} propwashControlConfig_t;

PG_DECLARE(propwashControlConfig_t, propwashControlConfig);

void checkPropwash(void);
bool canApplyBoost(void);
float computeBoost();
void initAntiPropwashThrottleFilter(void);
void updateAntiPropwashThrottleFilter(float throttle);
bool isInPropwashZone(void);
