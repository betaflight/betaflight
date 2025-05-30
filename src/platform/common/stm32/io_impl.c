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

#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"

#if DEFIO_PORT_USED_COUNT > 0
static const uint16_t ioDefUsedMask[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_USED_LIST };
static const uint8_t ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_OFFSET_LIST };
#else
// Avoid -Wpedantic warning
static const uint16_t ioDefUsedMask[1] = {0};
static const uint8_t ioDefUsedOffset[1] = {0};
#endif

// initialize all ioRec_t structures from ROM
// currently only bitmask is used, this may change in future
void IOInitGlobal(void)
{
    ioRec_t *ioRec = ioRecs;

    for (unsigned port = 0; port < ARRAYLEN(ioDefUsedMask); port++) {
        for (unsigned pin = 0; pin < sizeof(ioDefUsedMask[0]) * 8; pin++) {
            if (ioDefUsedMask[port] & (1 << pin)) {
                ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + (port << 10));   // ports are 0x400 apart
                ioRec->pin = 1 << pin;
                ioRec++;
            }
        }
    }
}

IO_t IOGetByTag(ioTag_t tag)
{
    const int portIdx = DEFIO_TAG_GPIOID(tag);
    const int pinIdx = DEFIO_TAG_PIN(tag);

    if (portIdx < 0 || portIdx >= DEFIO_PORT_USED_COUNT) {
        return NULL;
    }
    // check if pin exists
    if (!(ioDefUsedMask[portIdx] & (1 << pinIdx))) {
        return NULL;
    }
    // count bits before this pin on single port
    int offset = popcount(((1 << pinIdx) - 1) & ioDefUsedMask[portIdx]);
    // and add port offset
    offset += ioDefUsedOffset[portIdx];
    return ioRecs + offset;
}

int IO_GPIOPortIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    return (((size_t)IO_GPIO(io) - GPIOA_BASE) >> 10);
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
