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

/*
 * GPIO on the RGPIO blocks, ports A..E are GPIO1..GPIO5. Pad settings live in iomux_nxp.c.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "iomux_nxp.h"

#if DEFIO_PORT_USED_COUNT > 0
static const uint32_t ioDefUsedMask[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_USED_LIST };
static const uint8_t ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_OFFSET_LIST };
#else
// Avoid a zero length array
static const uint32_t ioDefUsedMask[1] = { 0 };
static const uint8_t ioDefUsedOffset[1] = { 0 };
#endif

static RGPIO_Type *const gpioPorts[] = {
    GPIO1, GPIO2, GPIO3, GPIO4, GPIO5,
};

// IOInitGlobal looks up each used port here.
STATIC_ASSERT(ARRAYLEN(gpioPorts) == NXP_GPIO_PORT_COUNT, gpio_ports_must_list_every_port);
STATIC_ASSERT(DEFIO_PORT_USED_COUNT <= NXP_GPIO_PORT_COUNT, target_uses_more_ports_than_the_family_has);

// Local accessors keep the ITCM functions here from calling into flash.
static inline uint16_t ioPin(IO_t io)
{
    return ((const ioRec_t *)io)->pin;
}

static inline RGPIO_Type *ioGpio(IO_t io)
{
    return (RGPIO_Type *)((const ioRec_t *)io)->gpio;
}

void IOInitGlobal(void)
{
    ioRec_t *ioRec = ioRecs;

    for (unsigned port = 0; port < ARRAYLEN(ioDefUsedMask); port++) {
        for (unsigned pin = 0; pin < DEFIO_PORT_PINS; pin++) {
            if (ioDefUsedMask[port] & (1u << pin)) {
                ioRec->gpio = (gpioResource_t *)gpioPorts[port];
                // An index, not a mask: pin 31 would not fit the 16 bit field.
                ioRec->pin = pin;
                ioRec++;
            }
        }
    }

    nxpIomuxClaimReserved();
}

IO_t IOGetByTag(ioTag_t tag)
{
    const int portIdx = DEFIO_TAG_GPIOID(tag);
    const int pinIdx = DEFIO_TAG_PIN(tag);

    if (portIdx < 0 || portIdx >= DEFIO_PORT_USED_COUNT) {
        return NULL;
    }

    if (!(ioDefUsedMask[portIdx] & (1u << pinIdx))) {
        return NULL;
    }

    // Used pins before this one on its port, plus the used pins of earlier ports.
    const int offset = ioDefUsedOffset[portIdx]
                     + popcount(((1u << pinIdx) - 1) & ioDefUsedMask[portIdx]);

    return ioRecs + offset;
}

int IO_GPIOPortIdx(IO_t io)
{
    if (!io) {
        return -1;
    }

    const RGPIO_Type *gpio = ioGpio(io);
    for (unsigned port = 0; port < ARRAYLEN(gpioPorts); port++) {
        if (gpioPorts[port] == gpio) {
            return port;
        }
    }

    return -1;
}

int IO_GPIOPinIdx(IO_t io)
{
    if (!io) {
        return -1;
    }

    return ioPin(io);
}

FAST_CODE bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }

    return (ioGpio(io)->PDIR & (1u << ioPin(io))) != 0;
}

FAST_CODE void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }

    if (hi) {
        ioGpio(io)->PSOR = 1u << ioPin(io);
    } else {
        ioGpio(io)->PCOR = 1u << ioPin(io);
    }
}

FAST_CODE void IOHi(IO_t io)
{
    if (!io) {
        return;
    }

    ioGpio(io)->PSOR = 1u << ioPin(io);
}

FAST_CODE void IOLo(IO_t io)
{
    if (!io) {
        return;
    }

    ioGpio(io)->PCOR = 1u << ioPin(io);
}

FAST_CODE void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }

    ioGpio(io)->PTOR = 1u << ioPin(io);
}

// IOCFG_MODE_AF writes the pad settings only: this layer cannot know which peripheral the mux had,
// so a driver that took its pad to GPIO restores the mux itself with nxpIomuxConfigPin().
void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }

    nxpIomuxConfigGpio(io, cfg);

    // PDDR only matters while the pad is muxed to GPIO.
    if (IOCFG_MODE(cfg) == IOCFG_MODE_AF) {
        return;
    }

    const uint32_t mask = 1u << ioPin(io);

    // The M7 is a secure master and RGPIO ignores its accesses to pins marked nonsecure.
    // PCNS and PDDR are read-modify-write, so a board puts M7 pins only in RGPIO blocks Linux does not drive.
    ioGpio(io)->PCNS &= ~mask;

    if (IOCFG_MODE(cfg) == IOCFG_MODE_OUT) {
        ioGpio(io)->PDDR |= mask;
    } else {
        ioGpio(io)->PDDR &= ~mask;
    }
}
