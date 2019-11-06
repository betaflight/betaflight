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

typedef struct dbgPin_s {
    ioTag_t tag;
    GPIO_TypeDef *gpio;
    uint32_t setBSRR;
    uint32_t resetBSRR;
} dbgPin_t;

dbgPin_t dbgPins[] = {
    { .tag = IO_TAG(NONE) },
};

void dbgPinInit(void)
{
    for (unsigned i = 0; i < ARRAYLEN(dbgPins); i++) {
        dbgPin_t *dbgPin = &dbgPins[i];
        IO_t io = IOGetByTag(dbgPin->tag);
        if (!io) {
            continue;
        }
        IOConfigGPIO(io, IOCFG_OUT_PP);
        dbgPin->gpio = IO_GPIO(io);
        int pinSrc = IO_GPIO_PinSource(io);
        dbgPin->setBSRR = (1 << pinSrc);
        dbgPin->resetBSRR = (1 << (pinSrc + 16));
    }
}

void dbgPinHi(int index)
{
    if ((unsigned)index >= ARRAYLEN(dbgPins)) {
        return;
    }

    dbgPin_t *dbgPin = &dbgPins[index];
    if (dbgPin->gpio) {
#if defined(STM32F7)
        dbgPin->gpio->BSRR = dbgPin->setBSRR;
#else
        dbgPin->gpio->BSRRL = dbgPin->setBSRR;
#endif
    }
}

void dbgPinLo(int index)
{
    if ((unsigned)index > ARRAYLEN(dbgPins)) {
        return;
    }

    dbgPin_t *dbgPin = &dbgPins[index];

    if (dbgPins->gpio) {
#if defined(STM32F7)
        dbgPin->gpio->BSRR = dbgPin->resetBSRR;
#else
        dbgPin->gpio->BSRRL = dbgPin->resetBSRR;
#endif
    }
}

#endif
