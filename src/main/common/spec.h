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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#define MAX_NAME_SIZE 16
#define LOGO_WIDTH 4 // 3 + \n
#define LOGO_HEIGHT 3
#define LOGO_GROUPS 4

// Enum representing different types of specs
typedef enum {
    SPEC_FREEDOM,
    SPEC_7IN,
    SPEC_MAYHEM,
    SPEC_TT,
    SPEC_LLIGUETA,
    SPEC_COUNT // must be last
} SpecType;

// Struct representing the settings for each spec type
typedef struct specSettings_s {
    char name[MAX_NAME_SIZE]; // Null-terminated name string
    bool rpm_limit;
    uint16_t rpm_limit_p;
    uint16_t rpm_limit_i;
    uint16_t rpm_limit_d;
    uint16_t rpm_limit_value;
    uint8_t motorPoleCount;
    uint16_t kv;
    char logo[LOGO_GROUPS][LOGO_HEIGHT][LOGO_WIDTH]; // 4x3 array of strings
} specSettings_t;

// Declaration of specArray for external access
extern specSettings_t specArray[];

bool checkSpec(SpecType specType);
void setSpec(SpecType specType);
SpecType getCurrentSpec(void);