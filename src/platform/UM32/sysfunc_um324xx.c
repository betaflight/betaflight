/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/memprot.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

void checkForBootLoaderRequest(void);

bool isMPUSoftReset(void)
{
    if (cachedResetFlags & RCM_RFR_SOFTRSTF)
        return true;
    else
        return false;
}

void systemInit(void)
{
    persistentObjectInit();

    checkForBootLoaderRequest();

    // cache RCM->RFR value to use it in isMPUSoftReset() and others
    cachedResetFlags = RCM->RFR;

    // Configure NVIC preempt/priority groups. HAL_Init() (called earlier
    // from SystemInit) sets NVIC_PRIORITYGROUP_4, which discards all
    // sub-priority bits. Restore the grouping the rest of the codebase
    // encodes priorities for (2 bits preempt + 2 bits sub), mirroring what
    // the STM32 platform does in its systemInit().
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}

void systemReset(void)
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

#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)0x1fff0000)
#define SYSMEMBOOT_LOADER       ((uint32_t *)0x1fff0000)

typedef void *(*bootJumpPtr)(void);

typedef void resetHandler_t(void);

typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;

static void systemJumpToBootloader(void)
{

}

// Used in the startup files for 4xF
void checkForBootLoaderRequest(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    if (bootloaderRequest != RESET_BOOTLOADER_REQUEST_ROM) {
        return;
    }
    // persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE); //polo: move to BOOT

    extern isrVector_t system_isr_vector_table_base;

    __set_MSP(system_isr_vector_table_base.stackEnd);
    system_isr_vector_table_base.resetHandler();
    while (1);
}

void systemProcessResetReason(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
    case RESET_MSC_REQUEST:
        // RESET_REASON will be reset by MSC
    case RESET_NONE:
        break;

    case RESET_BOOTLOADER_REQUEST_ROM:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);

        systemJumpToBootloader();

        break;
    }
}
