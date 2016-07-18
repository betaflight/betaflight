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

#define MAX_PROFILE_COUNT 3 // do not use more than 3 due to some legacy code (e.g. profile selection, msp) and more recent code (config storage)

typedef struct profileSelection_s {
    uint8_t current_profile_index;
} profileSelection_t;

PG_DECLARE(profileSelection_t, profileSelection);

uint8_t getCurrentProfile(void);
void setProfile(uint8_t profileIndex);
