/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/pg.h"

#define MAX_NAME_LENGTH 16u
#define OSD_CUSTOM_MSG_COUNT 4

typedef struct pilotConfig_s {
    char craftName[MAX_NAME_LENGTH + 1];
    char pilotName[MAX_NAME_LENGTH + 1];
    char message[OSD_CUSTOM_MSG_COUNT][MAX_NAME_LENGTH + 1];
} pilotConfig_t;

PG_DECLARE(pilotConfig_t, pilotConfig);
