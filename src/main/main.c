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

#include "fc/init.h"

#include "scheduler/scheduler.h"

#include "FreeRTOS.h"
#include "task.h"

int ucHeap[configTOTAL_HEAP_SIZE];

void run( void *pvParameters );

int main(void)
{
	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping( 0 );

	xTaskCreate( run, "run", 256, NULL, tskIDLE_PRIORITY + TASK_PRIORITY_MEDIUM, NULL );

	/* Start the RTOS scheduler, this function should not return as it causes the
     * execution context to change from main() to one of the created tasks.
     */
   vTaskStartScheduler();

   /* Should never get here! */
   return 0;
}

void FAST_CODE FAST_CODE_NOINLINE run( void *pvParameters )
{
	UNUSED(pvParameters);

	init();

    while (true) {
        scheduler();
        processLoopback();

#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif
    }
}
