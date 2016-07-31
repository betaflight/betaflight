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

#pragma once

typedef struct sonarHcsr04Hardware_s {
    ioTag_t triggerTag;
    ioTag_t echoTag;
} sonarHcsr04Hardware_t;

struct rangefinder_s;
void hcsr04_set_sonar_hardware(void);
void hcsr04_init(struct rangefinder_s *rangefinder);
void hcsr04_start_reading(void);
int32_t hcsr04_get_distance(void);
