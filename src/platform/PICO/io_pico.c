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

#include "common/utils.h"
#include "hardware/gpio.h"

// initialize all ioRec_t structures from ROM
// currently only bitmask is used, this may change in future
void IOInitGlobal(void)
{
    ioRec_t *ioRec = ioRecs;

    for (unsigned pin = 0; pin < DEFIO_PIN_USED_COUNT; pin++) {
        ioRec->pin = pin;
        ioRec++;
    }
}

uint32_t IO_EXTI_Line(IO_t io)
{
    UNUSED(io);
    return 0;
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
    return gpio_get(IO_Pin(io));
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
    gpio_put(IO_Pin(io), hi);
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
    gpio_put(IO_Pin(io), 1);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
    gpio_put(IO_Pin(io), 0);
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }
    gpio_put(IO_Pin(io), !gpio_get(IO_Pin(io)));
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    /*
TODO: update to support the following
IOCFG_AF_PP
IOCFG_IN_FLOATING
IOCFG_IPD
IOCFG_IPU
IOCFG_OUT_OD
IOCFG_OUT_PP
IO_RESET_CFG

SPI_IO_CS_CFG (as defined)
SPI_IO_CS_HIGH_CFG (as defined)
    */
    if (!io) {
        return;
    }

    uint16_t ioPin = IO_Pin(io);
    gpio_init(ioPin);
    gpio_set_dir(ioPin, (cfg & 0x01)); // 0 = in, 1 = out
}

IO_t IOGetByTag(ioTag_t tag)
{
    const int portIdx = DEFIO_TAG_GPIOID(tag);
    const int pinIdx = DEFIO_TAG_PIN(tag);

    if (portIdx < 0 || portIdx >= DEFIO_PORT_USED_COUNT) {
        return NULL;
    }

    if (pinIdx >= DEFIO_PIN_USED_COUNT) {
        return NULL;
    }

    return &ioRecs[pinIdx];
}
