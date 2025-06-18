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
    core1_func_t *func;
} core_message_t;

// Define the queue
queue_t core0_queue;
queue_t core1_queue;

static void core1_main(void)
{
    // This loop is run on the second core
    while (true) {

        core_message_t msg;
        queue_remove_blocking(&core1_queue, &msg);

        switch (msg.command) {
        case MULTICORE_CMD_FUNC:
            if (msg.func) {
                msg.func();
            }
            break;
        case MULTICORE_CMD_FUNC_BLOCKING:
            if (msg.func) {
                msg.func();

                // Send the result back to core0 (it will be blocking until this is done)
                bool result = true;
                queue_add_blocking(&core0_queue, &result);
            }
            break;
        case MULTICORE_CMD_STOP:
            multicore_reset_core1();
            return; // Exit the core1_main function
        default:
            // unknown command or none
            break;
        }

        // Yield to allow other core to run
        tight_loop_contents();
    }
}

void multicoreStart(void)
{
     // Initialize the queue with a size of 4 (to be determined based on expected load)
    queue_init(&core1_queue, sizeof(core_message_t), 4);
    // Initialize the queue with a size of 1 (only needed for blocking results)
    queue_init(&core0_queue, sizeof(bool), 1);

    // Start core 1
    multicore_launch_core1(core1_main);
}

void multicoreStop(void)
{
   core_message_t msg;
    msg.command = MULTICORE_CMD_STOP;
    msg.func = NULL;

    queue_add_blocking(&core1_queue, &msg);
 }
#endif // USE_MULTICORE


void multicoreExecuteBlocking(core1_func_t *func)
{
#ifdef USE_MULTICORE
    core_message_t msg;
    msg.command = MULTICORE_CMD_FUNC_BLOCKING;
    msg.func = func;

    bool result;

    queue_add_blocking(&core1_queue, &msg);
    // Wait for the command to complete
    queue_remove_blocking(&core0_queue, &result);
#else
    // If multicore is not used, execute the command directly
    if (func) {
        func();
    }
#endif // USE_MULTICORE
}

void multicoreExecute(core1_func_t *func)
{
#ifdef USE_MULTICORE
    core_message_t msg;
    msg.command = MULTICORE_CMD_FUNC;
    msg.func = func;

    queue_add_blocking(&core1_queue, &msg);
#else
    // If multicore is not used, execute the command directly
    if (func) {
        func();
    }
#endif // USE_MULTICORE
}

