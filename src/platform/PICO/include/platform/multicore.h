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

#include "pico/multicore.h"

typedef enum multicoreCommand_e {
    MULTICORE_CMD_NONE = 0,
    MULTICORE_CMD_FUNC,
    MULTICORE_CMD_FUNC_BLOCKING, // Command to execute a function on the second core and wait for completion
    MULTICORE_CMD_STOP, // Command to stop the second core
} multicoreCommand_e;

// Define function types for clarity
typedef void (*core1_func_t)(void);

void multicoreStart(void);
void multicoreStop(void);
void multicoreExecute(core1_func_t command);
void multicoreExecuteBlocking(core1_func_t command);
