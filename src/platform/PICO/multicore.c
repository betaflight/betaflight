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
        uint32_t value = multicore_fifo_pop_blocking();
        multicoreCommand_e command = (multicoreCommand_e)(value & MULTICORE_CMD_MASK);

        uint32_t result = 0;

        // Handle the command received from the first core
        switch (command) {
        case MULTICORE_CMD_UART0_IRQ_ENABLE:
            irq_set_exclusive_handler(UART0_IRQ, on_uart0);
            irq_set_enabled(UART0_IRQ, true);
            break;
        case MULTICORE_CMD_UART1_IRQ_ENABLE:
            irq_set_exclusive_handler(UART1_IRQ, on_uart1);
            irq_set_enabled(UART1_IRQ, true);
            break;
        case MULTICORE_CMD_CDC_INIT:
            // Initialize the USB CDC interface
            cdc_usb_init();
            break;
        case MULTICORE_CMD_STOP:
            // Handle stop command
            multicore_reset_core1();
            return; // Exit the core1_main function
        default:
            // Handle unknown command
            break;
        }

        // Respond back to the first core
        // For blocking commands, we can return a value
        if (value & MULTICORE_BLOCKING) {
            // For blocking commands, we can return a value
            multicore_fifo_push_blocking(result);
        }

        // Yield to allow other core to run
        tight_loop_contents();
    }
}

void multicoreStart(void)
{
    // Start the multicore program
    multicore_launch_core1(core1_main);
}

uint32_t multicoreExecuteBlocking(multicoreCommand_e command)
{
    // Send a command to the second core but wait for a response
    multicore_fifo_push_blocking(MULTICORE_BLOCKING | command);
    return multicore_fifo_pop_blocking();
}

void multicoreExecute(multicoreCommand_e command)
{
    // Send a command to the second core
    multicore_fifo_push_blocking(command);
}

#endif // USE_MULTICORE
