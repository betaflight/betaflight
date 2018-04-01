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

typedef enum {
    FAILURE_DEVELOPER = 0,
    FAILURE_MISSING_ACC,
    FAILURE_ACC_INIT,
    FAILURE_ACC_INCOMPATIBLE,
    FAILURE_INVALID_EEPROM_CONTENTS,
    FAILURE_FLASH_WRITE_FAILED,
    FAILURE_GYRO_INIT_FAILED
} failureMode_e;

#define WARNING_FLASH_DURATION_MS 50
#define WARNING_FLASH_COUNT 5
#define WARNING_PAUSE_DURATION_MS 500
#define WARNING_CODE_DURATION_LONG_MS 250
#define WARNING_CODE_DURATION_SHORT_MS 50

// failure
void indicateFailure(failureMode_e mode, int repeatCount);
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
extern uint32_t cachedRccCsrValue;

typedef void extiCallbackHandlerFunc(void);

void registerExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);void unregisterExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);

