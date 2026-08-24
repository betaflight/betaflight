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

// ESP32 multicore: core 0 runs the FC and owns interrupts/peripherals; core 1
// is a compute-offload helper. Same API shape as the PICO port.

typedef enum multicoreCommand_e {
    MULTICORE_CMD_NONE = 0,
    MULTICORE_CMD_FUNC,            // run a function on core 1, don't wait
    MULTICORE_CMD_FUNC_BLOCKING,   // run a function on core 1 and wait for it
    MULTICORE_CMD_STOP,            // stop the core 1 command loop
} multicoreCommand_e;

typedef void core1_func_t(void);

void multicoreStart(void);
void multicoreStop(void);
void multicoreExecute(core1_func_t *func);
void multicoreExecuteBlocking(core1_func_t *func);
