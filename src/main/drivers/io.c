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

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"

#include "common/utils.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(SITL)
const struct ioPortDef_s ioPortDefs[] = { 0 };
#endif

ioRec_t* IO_Rec(IO_t io)
{
    return io;
}

GPIO_TypeDef* IO_GPIO(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->gpio;
}

uint16_t IO_Pin(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->pin;
}

#if defined(STM32F4) || defined(APM32F4)
int IO_EXTI_PortSourceGPIO(IO_t io)
{
    return IO_GPIOPortIdx(io);
}
#endif

int IO_GPIO_PortSource(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

// zero based pin index
int IO_GPIOPinIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    return 31 - __builtin_clz(IO_Pin(io));
}

#if defined(STM32F4) || defined(APM32F4)
int IO_EXTI_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}
#endif

int IO_GPIO_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

// claim IO pin, set owner and resources
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
{
    if (!io) {
        return;
    }
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = owner;
    ioRec->index = index;
}

void IORelease(IO_t io)
{
    if (!io) {
        return;
    }
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = OWNER_FREE;
}

resourceOwner_e IOGetOwner(IO_t io)
{
    if (!io) {
        return OWNER_FREE;
    }
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->owner;
}

bool IOIsFreeOrPreinit(IO_t io)
{
    resourceOwner_e owner = IOGetOwner(io);

    if (owner == OWNER_FREE || owner == OWNER_PREINIT) {
        return true;
    }

    return false;
}

#if DEFIO_IO_USED_COUNT
ioRec_t ioRecs[DEFIO_IO_USED_COUNT];
#else
// Avoid -Wpedantic warning
ioRec_t ioRecs[1];
#endif

void IOTraversePins(IOTraverseFuncPtr_t fnPtr)
{
    for (int i = 0; i < DEFIO_IO_USED_COUNT; i++) {
        fnPtr(&ioRecs[i]);
    }
}
