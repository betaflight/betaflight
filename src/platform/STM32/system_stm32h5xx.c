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

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/memprot.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

bool isMPUSoftReset(void)
{
    if (cachedResetFlags & RCC_RSR_SFTRSTF)
        return true;
    else
        return false;
}

void systemInit(void)
{
    memProtReset();
    memProtConfigure(mpuRegions, mpuRegionCount);

    // STM32H5 (Cortex-M33): the instruction cache is a separate peripheral,
    // disabled at reset, and is the ONLY flash accelerator on H5 (FLASH_ACR
    // has no prefetch bit). Without it, hot code runs from flash at 5 wait
    // states with no acceleration, inflating CPU load. Enable it here.
    //
    // Coherency notes:
    //   - DMA buffers live in SRAM accessed via the system-bus alias
    //     (0x2000_0000+), which bypasses ICACHE, so DMA writes need no
    //     invalidation.
    //   - The one region that changes at runtime under the code alias is the
    //     config flash storage; configLock() invalidates ICACHE after each
    //     config write so reads stay coherent.
    //   - The OTP / read-only factory info block (UID, ADC calibration) is
    //     marked non-cacheable by memProtConfigure() above — its non-burst
    //     reads would otherwise fault an ICACHE cache-line refill.
    //
    // Do NOT call HAL_ICACHE_Invalidate() before enabling: on power-on/reset
    // the cache is auto-invalidated by hardware, and the HAL invalidate wait
    // loop is gated on HAL_GetTick(), which is not yet advancing this early in
    // systemInit() — if it entered the wait it could spin forever and hang the
    // board before USB comes up. HAL_ICACHE_Enable() alone is sufficient.
    HAL_ICACHE_Enable();

    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->RSR value to use it in isMPUSoftReset() and others
    cachedResetFlags = RCC->RSR;

    // Init cycle counter
    cycleCounterInit();
}

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetWithoutDisablingCaches(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);

        break;
    }

    __disable_irq();
    NVIC_SystemReset();
}

// STM32H5 system bootloader vector-table address (AN2606).
// This is the bootloader entry point, NOT the FLASH_SYSTEM_BASE_NS region base
// (0x0BF80000): reading the initial SP / reset vector from the region base lands
// on garbage, so the software `bl`/MSP DFU request never enters the bootloader
// while the hardware BOOT pin still works (the ROM handles that path itself).
#if defined(STM32H562xx) || defined(STM32H563xx) || defined(STM32H573xx)
#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)0x0BF97000)
#elif defined(STM32H503xx)
#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)0x0BF87000)
#else
#error "STM32H5: system bootloader address unknown for this part (see AN2606)"
#endif

typedef void *(*bootJumpPtr)(void);

typedef void resetHandler_t(void);

typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;

void systemJumpToBootloader(void)
{
    //DeInit all used peripherals
    HAL_RCC_DeInit();

    //Disable all system timers and set to default values
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    //Disable all interrupts
    __disable_irq();

    // STM32H5 (Cortex-M33) does not have SYSCFG memory remap.
    // Use VTOR to point directly to the system flash vector table.
    SCB->VTOR = (uint32_t)SYSMEMBOOT_VECTOR_TABLE;

    //default bootloader call stack routine
    uint32_t bootStack = SYSMEMBOOT_VECTOR_TABLE[0];

    bootJumpPtr SysMemBootJump = (bootJumpPtr)SYSMEMBOOT_VECTOR_TABLE[1];

    __set_MSP(bootStack); //Set the main stack pointer to its default values

    SysMemBootJump();

    while (1);
}

void systemProcessResetReason(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
    case RESET_BOOTLOADER_REQUEST_ROM:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);
        systemJumpToBootloader();

        break;

    case RESET_FORCED:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        break;

    case RESET_BOOTLOADER_POST:
        // Boot loader activity magically prevents SysTick from interrupting.
        // Issue a soft reset to prevent the condition.
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FORCED);
        systemResetWithoutDisablingCaches();

        break;

    case RESET_MSC_REQUEST:
    case RESET_NONE:
    default:
        break;
    }
}
