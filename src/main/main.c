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

#ifdef USE_VCP
#include "drivers/serial_usb_vcp.h"
#endif

#include "drivers/system.h"

#include "fc/init.h"

#ifdef USE_MULTICORE
#include "platform/multicore.h"
#include "usb/usb_cdc.h"
#endif

#ifdef USE_USB_MSC
#include "drivers/usb_msc.h"
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

#if SERIAL_PORT_COUNT > 0
    printfSerialInit();
#endif

    // Do basic system initialisation including multicore support if applicable
    systemInit();

#ifdef USE_MULTICORE
    // Perform early initialisation prior to USB
    multicoreExecuteBlocking(initPhase1);

    // Now perform the core initialisation
    multicoreExecuteBlocking(initPhase2);
#else
    initPhase1();
    initPhase2();
#endif

#ifdef USE_USB_MSC
    mscButtonInit();
    if (checkMsc()) {
        // MSC mode TODO: boot using single core
        initMsc();
        // Never returns (but just in case...)
        return 0;
    }
#endif

#ifdef USE_VCP
    // initialise the USB CDC interface using core 0 all USB code, including
    // interrupts, must run on core 0
    usbVcpInit();
#endif

#ifdef USE_MULTICORE
    // Now perform the final initialisation
    multicoreExecuteBlocking(initPhase3);
#else
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
