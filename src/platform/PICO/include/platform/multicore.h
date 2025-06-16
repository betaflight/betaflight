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

#define MULTICORE_CMD_MASK   0x00FFFFFF // Mask to extract command values
#define MULTICORE_FLAGS_MASK 0xFF000000 // Mask to extract flags bits

typedef enum {
    MULTICORE_CMD_NONE = 0,
    MULTICORE_CMD_UART0_IRQ_ENABLE,
    MULTICORE_CMD_UART1_IRQ_ENABLE,
    MULTICORE_CMD_CDC_INIT, // Initialize the USB CDC interface
    // Add more commands as needed
    MULTICORE_CMD_STOP,
} multicoreCommand_e;

typedef enum {
    MULTICORE_BLOCKING = (1 << 31), // Use this bit to indicate blocking commands
} multicoreFlags_e;

void multicoreExecute(multicoreCommand_e command);
uint32_t multicoreExecuteBlocking(multicoreCommand_e command);
