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
#include "pico/multicore.h"
#include "pico/util/queue.h"

#ifdef USE_MULTICORE

// Define a structure for the message we'll pass through the queue
typedef struct {
    multicoreCommand_e command;
    core1_func_t func;
} core_message_t;

// Define the queue
queue_t core0_queue;
queue_t core1_queue;

static void core1_main(void)
{
    // This function is called on the second core
    // Implement the multicore functionality here
    while (true) {

        core_message_t msg;
        // Block until a message is available in the queue
        queue_remove_blocking(&core1_queue, &msg);

        // Handle the command received from the first core
        switch (msg.command) {
        case MULTICORE_CMD_FUNC:
            if (msg.func != NULL) {
                // Call the function passed from core0
                msg.func();
            }
            break;
        case MULTICORE_CMD_FUNC_BLOCKING:
            if (msg.func != NULL) {
                // Call the function passed from core0 and wait for completion
                msg.func();
                // Send the result back to core0
                bool result = true;
                queue_add_blocking(&core0_queue, &result);
            }
            break;
        case MULTICORE_CMD_STOP:
            // Handle stop command
            multicore_reset_core1();
            return; // Exit the core1_main function
        default:
            // Handle unknown command
            break; // Skip to the next iteration
        }

        // Yield to allow other core to run
        tight_loop_contents();
    }
}

void multicoreStart(void)
{
    queue_init(&core1_queue, sizeof(core_message_t), 4); // Initialize the queue with a size of 4
    queue_init(&core0_queue, sizeof(bool), 1); // Initialize the queue with a size of 4

    // Start core 1
    multicore_launch_core1(core1_main);
}

void multicoreStop(void)
{
   core_message_t msg;
    msg.command = MULTICORE_CMD_STOP; // Set the command type
    msg.func = NULL;

    queue_add_blocking(&core1_queue, &msg);
 }
#endif // USE_MULTICORE


void multicoreExecuteBlocking(core1_func_t command)
{
#ifdef USE_MULTICORE
    core_message_t msg;
    msg.command = MULTICORE_CMD_FUNC_BLOCKING; // Set the command type
    msg.func = command; // Assign the function to run

    bool result;

    queue_add_blocking(&core1_queue, &msg);
    queue_remove_blocking(&core0_queue, &result); // Wait for the command to complete
#else
    // If multicore is not used, execute the command directly
    if (command) {
        command();
    }
#endif // USE_MULTICORE
}

void multicoreExecute(core1_func_t command)
{
#ifdef USE_MULTICORE
    core_message_t msg;
    msg.command = MULTICORE_CMD_FUNC; // Set the command type
    msg.func = command; // Assign the function to run

    queue_add_blocking(&core1_queue, &msg);
#else
    // If multicore is not used, execute the command directly
    if (command) {
        command();
    }
#endif // USE_MULTICORE
}

