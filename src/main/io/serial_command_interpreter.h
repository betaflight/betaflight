/*
 * This file is part of Cleanflight, Betaflight and INAV
 *
 * Cleanflight, Betaflight and INAV are free software. You can
 * redistribute this software and/or modify this software under
 * the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Cleanflight, Betaflight and INAV are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "drivers/time.h"

typedef enum {
    SERIAL_COMMAND_INTERPRETER_STATE_IDLE,
    SERIAL_COMMAND_INTERPRETER_STATE_PLUS_ONE,
    SERIAL_COMMAND_INTERPRETER_STATE_PLUS_TWO,
    SERIAL_COMMAND_INTERPRETER_STATE_PLUS_THREE,
    SERIAL_COMMAND_INTERPRETER_STATE_PLUS_THREE_WHITESPACE,
    SERIAL_COMMAND_INTERPRETER_STATE_A,
    SERIAL_COMMAND_INTERPRETER_STATE_T,
} serialCommandInterpreterState_e;

typedef enum {
    SERIAL_COMMAND_INTERPRETER_RESULT_NONE,
    SERIAL_COMMAND_INTERPRETER_RESULT_USED,
    SERIAL_COMMAND_INTERPRETER_RESULT_HANG,
} serialCommandInterpreterResult_e;

typedef struct serialCommandInterpreter_s {
    timeMs_t lastByteMs;
    serialCommandInterpreterState_e state;
} serialCommandInterpreter_t;

void serialCommandInterpreterInit(serialCommandInterpreter_t *interpreter);
serialCommandInterpreterResult_e serialCommandInterpreterUpdate(serialCommandInterpreter_t *interpreter, uint8_t c);
