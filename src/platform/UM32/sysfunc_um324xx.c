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

    systemProcessResetReason();

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

typedef void resetHandler_t(void);

typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;

void systemProcessResetReason(void)
{
    if (persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON) != RESET_BOOTLOADER_REQUEST_ROM) {
        return;
    }

    // The UM324xF DFU bootloader's SystemInit() reads and clears RTC_BKREG3.
    // With BOOT0 low and a cleared request, it immediately resets into the application.

    extern isrVector_t system_isr_vector_table_base;

    __set_MSP(system_isr_vector_table_base.stackEnd);
    system_isr_vector_table_base.resetHandler();
    while (1);
}
