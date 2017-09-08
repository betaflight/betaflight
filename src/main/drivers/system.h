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

#pragma once

#include <stdint.h>
#include <stdbool.h>

void systemInit(void);
void systemClockSetup(uint8_t cpuUnderclock);

typedef enum {
    FAILURE_DEVELOPER = 0,
    FAILURE_MISSING_ACC,
    FAILURE_ACC_INIT,
    FAILURE_ACC_INCOMPATIBLE,
    FAILURE_INVALID_EEPROM_CONTENTS,
    FAILURE_FLASH_WRITE_FAILED,
    FAILURE_GYRO_INIT_FAILED
} failureMode_e;

// failure
void failureMode(failureMode_e mode);

// bootloader/IAP
void systemReset(void);
void systemResetToBootloader(void);
bool isMPUSoftReset(void);
void cycleCounterInit(void);
void checkForBootLoaderRequest(void);

void enableGPIOPowerUsageAndNoiseReductions(void);
// current crystal frequency - 8 or 12MHz

extern uint32_t hse_value;

typedef void extiCallbackHandlerFunc(void);

typedef struct extiCallbackHandlerConfig_s {
    IRQn_Type irqn;
    extiCallbackHandlerFunc* fn;
} extiCallbackHandlerConfig_t;

#ifndef EXTI_CALLBACK_HANDLER_COUNT
#define EXTI_CALLBACK_HANDLER_COUNT 1
#endif

extern extiCallbackHandlerConfig_t extiHandlerConfigs[EXTI_CALLBACK_HANDLER_COUNT];

void registerExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);
void unregisterExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);

extern uint32_t cachedRccCsrValue;

