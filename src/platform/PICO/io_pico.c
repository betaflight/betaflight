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

#if DEFIO_PORT_USED_COUNT > 1
#error PICO code currently based on a single io port
#endif

// Initialize all ioRec_t structures.
// PICO (single port) doesn't use the gpio field.
void IOInitGlobal(void)
{
    ioRec_t *ioRec = ioRecs;

    for (unsigned pin = 0; pin < DEFIO_PIN_USED_COUNT; pin++) {
        ioRec->pin = pin;
        ioRec++;
    }

#ifdef PICO_TRACE
#ifdef PICO_TRACE_TX_GPIO
    ioRecs[PICO_TRACE_TX_GPIO].owner = OWNER_SYSTEM;
#endif
#ifdef PICO_TRACE_RX_GPIO
    ioRecs[PICO_TRACE_RX_GPIO].owner = OWNER_SYSTEM;
#endif
#endif

    // Some boards (e.g. Hellbender) require a pin to be held low in order to generate a 5V / 9V
    // power supply from the main battery.
    // (TODO: should we manage a list of pins that we want to send low or high?)
#ifdef PICO_BEC_5V_ENABLE_PIN
    const int pin5 = IO_PINBYTAG(IO_TAG(PICO_BEC_5V_ENABLE_PIN));
    gpio_init(pin5);
    gpio_set_dir(pin5, 1);
    gpio_put(pin5, 0);
    bprintf("5V enable pin: %d set low", pin5);
    ioRecs[pin5].owner = OWNER_SYSTEM;
#endif

#ifdef PICO_BEC_9V_ENABLE_PIN
    const int pin9 = IO_PINBYTAG(IO_TAG(PICO_BEC_9V_ENABLE_PIN));
    gpio_init(pin9);
    gpio_set_dir(pin9, 1);
    gpio_put(pin9, 0);
    bprintf("9V enable pin: %d set low", pin9);
    ioRecs[pin9].owner = OWNER_SYSTEM;
#endif
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
    bprintf("pico IOConfigGPIO gpio %d for 0x%02x (0=in, 1=out)",ioPin, cfg);

    gpio_function_t currentFunction = gpio_get_function(ioPin);
    if (currentFunction == GPIO_FUNC_NULL) {
        // Select GPIO_FUNC_SIO, set direction to input, clear output value (set to low)
        gpio_init(ioPin);
    } else if (currentFunction != GPIO_FUNC_SIO) {
        bprintf("Warning: not redefining gpio function type from %d to SIO\n", currentFunction);
    }

    gpio_set_dir(ioPin, (cfg & 0x01)); // 0 = in, 1 = out
    gpio_set_pulls(ioPin, (cfg >> 5) & GPIO_PULLUP, (cfg >> 5) & GPIO_PULLDOWN);
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

int IO_GPIOPortIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    return 0; // Single port
}

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
    return IO_Pin(io);
}

int IO_GPIO_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}
