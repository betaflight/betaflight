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

#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "pico/unique_id.h"

int main(int argc, char * argv[]);

void Reset_Handler(void);
void Default_Handler(void);

// cycles per microsecond
static uint32_t usTicks = 0;

void (* const vector_table[])() __attribute__((section(".vectors"))) = {
    (void (*)())0x20000000, // Initial Stack Pointer
    Reset_Handler,           // Interrupt Handler for reset
    Default_Handler,         // Default handler for other interrupts
};

uint32_t SystemCoreClock; /* System Clock Frequency (Core Clock)*/

void SystemCoreClockUpdate (void)
{
    SystemCoreClock = clock_get_hz(clk_sys);
}

void __attribute__((constructor)) SystemInit (void)
{
    SystemCoreClockUpdate();
}

void Reset_Handler(void)
{
    // Initialize data segments
    extern uint32_t _sdata, _edata, _sidata;
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    while (dst < &_edata) {
        *dst++ = *src++;
    }

    // Clear bss segment
    extern uint32_t _sbss, _ebss;
    dst = &_sbss;

    while (dst < &_ebss) {
        *dst++ = 0;
    }

    usTicks = clock_get_hz(clk_sys) / 1000000;

    // Call main function
    main(0, 0);
}

void Default_Handler(void)
{
    while (1); // Infinite loop on default handler
}

void systemReset(void)
{
    //TODO: implement
}

uint32_t systemUniqueId[3] = { 0 };

void systemInit(void)
{
    //TODO: implement

    // load the unique id into a local array
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    memcpy(&systemUniqueId, &id.id, MIN(sizeof(systemUniqueId), PICO_UNIQUE_BOARD_ID_SIZE_BYTES));
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    //TODO: correction required?
    return time_us_32() / 1000;
}

// Return system uptime in micros
uint32_t micros(void)
{
    return time_us_32();
}

uint32_t microsISR(void)
{
    return micros();
}

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

void delay(uint32_t ms)
{
    while (ms--) {
        delayMicroseconds(1000);
    }
}

uint32_t getCycleCounter(void)
{
    return time_us_32();
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros / usTicks;
}
