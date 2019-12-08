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

#include <ctype.h>

#include "platform.h"

#include "io/serial_command_interpreter.h"

#define PAUSE_INTERVAL_MS_MIN 1000

void serialCommandInterpreterInit(serialCommandInterpreter_t *interpreter)
{
    interpreter->lastByteMs = 0;
    interpreter->state = SERIAL_COMMAND_INTERPRETER_STATE_IDLE;
}

serialCommandInterpreterResult_e serialCommandInterpreterUpdate(serialCommandInterpreter_t *interpreter, uint8_t c)
{
    serialCommandInterpreterResult_e result = SERIAL_COMMAND_INTERPRETER_RESULT_NONE;
    timeMs_t now = millis();
    switch (interpreter->state) {
    case SERIAL_COMMAND_INTERPRETER_STATE_IDLE:
        if (c == '+' && now - interpreter->lastByteMs >= PAUSE_INTERVAL_MS_MIN) {
            interpreter->state = SERIAL_COMMAND_INTERPRETER_STATE_PLUS_ONE;
        }
        break;
    case SERIAL_COMMAND_INTERPRETER_STATE_PLUS_ONE:
    case SERIAL_COMMAND_INTERPRETER_STATE_PLUS_TWO:
        if (c == '+' && now - interpreter->lastByteMs < PAUSE_INTERVAL_MS_MIN) {
            interpreter->state++;
            break;
        }
        interpreter->state = SERIAL_COMMAND_INTERPRETER_STATE_IDLE;
        break;
    case SERIAL_COMMAND_INTERPRETER_STATE_PLUS_THREE:
    case SERIAL_COMMAND_INTERPRETER_STATE_PLUS_THREE_WHITESPACE:
        // We might be in command mode here. Ignore one whitespace to let the user
        // input the command via the configurator CLI.
        if (isspace(c) && interpreter->state == SERIAL_COMMAND_INTERPRETER_STATE_PLUS_THREE) {
            interpreter->state = SERIAL_COMMAND_INTERPRETER_STATE_PLUS_THREE_WHITESPACE;
            break;
        }
        if (now - interpreter->lastByteMs >= PAUSE_INTERVAL_MS_MIN) {
            if (c == 'A') {
                interpreter->state = SERIAL_COMMAND_INTERPRETER_STATE_A;
                result = SERIAL_COMMAND_INTERPRETER_RESULT_USED;
                break;
            }
        }
        interpreter->state = SERIAL_COMMAND_INTERPRETER_STATE_IDLE;
        break;
    case SERIAL_COMMAND_INTERPRETER_STATE_A:
        if (c == 'T') {
            interpreter->state = SERIAL_COMMAND_INTERPRETER_STATE_T;
            result = SERIAL_COMMAND_INTERPRETER_RESULT_USED;
            break;
        }
        interpreter->state = SERIAL_COMMAND_INTERPRETER_STATE_IDLE;
        break;
    case SERIAL_COMMAND_INTERPRETER_STATE_T:
        switch (c) {
        case 'H':
            result = SERIAL_COMMAND_INTERPRETER_RESULT_HANG;
            break;
        }
        break;
    }
    interpreter->lastByteMs = now;
    return result;
}
