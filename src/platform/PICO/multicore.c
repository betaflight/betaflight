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

#include "platform.h"
#include "platform/multicore.h"
#include "usb/usb_cdc.h"
#include "platform/uart.h"

#ifdef USE_MULTICORE

void core1_main(void)
{
    // This function is called on the second core
    // Implement the multicore functionality here
    while (true) {

        uint32_t command = multicore_fifo_pop_blocking();

        uint32_t result = 0;

        // Handle the command received from the first core
        switch (command) {
        case MULTICORE_CMD_FUNC:
            core1_func_void_t funcVoid = (core1_func_void_t)multicore_fifo_pop_blocking();
            funcVoid();
            continue;
        case MULTICORE_CMD_UINT:
            core1_func_uint_t funcUint = (core1_func_uint_t)multicore_fifo_pop_blocking();
            result = funcUint();
            break;
        case MULTICORE_CMD_STOP:
            // Handle stop command
            multicore_reset_core1();
            return; // Exit the core1_main function
        default:
            // Handle unknown command
            continue; // Skip to the next iteration
        }

        // For blocking commands, we can return a value
        multicore_fifo_push_blocking(result);

        // Yield to allow other core to run
        tight_loop_contents();
    }
}

void multicoreStart(void)
{
    // Start the multicore program
    multicore_launch_core1(core1_main);
}
#endif // USE_MULTICORE


uint32_t multicoreExecuteBlocking(core1_func_uint_t command)
{
#ifdef USE_MULTICORE
    // Send a command to the second core but wait for a response
    multicore_fifo_push_blocking((uint32_t)command);
    return multicore_fifo_pop_blocking();
#else
    // If multicore is not used, execute the command directly
    command();
    return 0; // Return 0 as a default response
#endif // USE_MULTICORE
}

void multicoreExecute(core1_func_void_t command)
{
#ifdef USE_MULTICORE
    // Send a command to the second core
    multicore_fifo_push_blocking((uint32_t)command);
#else
    // If multicore is not used, execute the command directly
    command();
#endif // USE_MULTICORE
}

