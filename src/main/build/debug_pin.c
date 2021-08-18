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

#include "platform.h"

#ifdef USE_DEBUG_PIN

#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "debug_pin.h"

typedef struct dbgPinState_s {
    GPIO_TypeDef *gpio;
    uint32_t setBSRR;
    uint32_t resetBSRR;
} dbgPinState_t;

#ifndef DEBUG_PIN_COUNT
#define DEBUG_PIN_COUNT 1
#endif

// Provide a non-weak reference in target.c or elsewhere
__weak dbgPin_t dbgPins[DEBUG_PIN_COUNT] = {
    { .tag = IO_TAG(NONE) },
};

dbgPinState_t dbgPinStates[DEBUG_PIN_COUNT] = { 0 };

void dbgPinInit(void)
{
    for (unsigned i = 0; i < ARRAYLEN(dbgPins); i++) {
        dbgPin_t *dbgPin = &dbgPins[i];
        dbgPinState_t *dbgPinState = &dbgPinStates[i];
        IO_t io = IOGetByTag(dbgPin->tag);
        if (!io) {
            continue;
        }
        IOInit(io, OWNER_SYSTEM, 0);
        IOConfigGPIO(io, IOCFG_OUT_PP);
        dbgPinState->gpio = IO_GPIO(io);
        int pinSrc = IO_GPIO_PinSource(io);
        dbgPinState->setBSRR = (1 << pinSrc);
        dbgPinState->resetBSRR = (1 << (pinSrc + 16));
    }
}

void dbgPinHi(int index)
{
    if ((unsigned)index >= ARRAYLEN(dbgPins)) {
        return;
    }

    dbgPinState_t *dbgPinState = &dbgPinStates[index];
    if (dbgPinState->gpio) {
#if defined(STM32F7) || defined(STM32H7)
        dbgPinState->gpio->BSRR = dbgPinState->setBSRR;
#else
        dbgPinState->gpio->BSRRL = dbgPinState->setBSRR;
#endif
    }
}

void dbgPinLo(int index)
{
    if ((unsigned)index >= ARRAYLEN(dbgPins)) {
        return;
    }

    dbgPinState_t *dbgPinState = &dbgPinStates[index];

    if (dbgPinState->gpio) {
#if defined(STM32F7) || defined(STM32H7)
        dbgPinState->gpio->BSRR = dbgPinState->resetBSRR;
#else
        dbgPinState->gpio->BSRRL = dbgPinState->resetBSRR;
#endif
    }
}

#endif
