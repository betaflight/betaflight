/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/system.h"

#include "fc/init.h"

#ifdef USE_MULTICORE
#include "platform/multicore.h"
#include "usb/usb_cdc.h"
#endif

#include "scheduler/scheduler.h"

void run(void);

int main(int argc, char * argv[])
{
#ifdef USE_MAIN_ARGS
    targetParseArgs(argc, argv);
#else
    UNUSED(argc);
    UNUSED(argv);
#endif

    // Do basic system initialisation including multicore support if applicable
    systemInit();

#ifdef USE_USB_MSC
    if (checkMsc()) {
        // MSC mode so boot using single core
        initPhase1();
        initPhase2();
        initMsc();
        // Never returns
    }
#endif

#ifdef USE_MULTICORE
    // Perform early initialisation prior to USB
    multicoreExecuteBlocking(initPhase1);

    // initialise the USB CDC interface using core 0 all USB code, including
    // interrupts, must run on core 0
    cdc_usb_init();

    // Now perform the core initialisation
    multicoreExecuteBlocking(initPhase2);

    // Now perform the final initialisation
    multicoreExecuteBlocking(initPhase3);
#else
    initPhase1();
    initPhase2();
    initPhase3();
#endif

    // Launch the scheduler
    run();

    return 0;
}

void FAST_CODE run(void)
{
    while (true) {
        scheduler();
#if defined(RUN_LOOP_DELAY_US) && RUN_LOOP_DELAY_US > 0
        delayMicroseconds_real(RUN_LOOP_DELAY_US);
#endif
    }
}
