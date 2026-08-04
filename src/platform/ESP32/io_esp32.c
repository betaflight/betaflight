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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "common/utils.h"

#if DEFIO_PORT_USED_COUNT > 1
#error ESP32 code currently based on a single io port
#endif

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
    return gpio_ll_get_level(&GPIO, IO_Pin(io));
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
    gpio_ll_set_level(&GPIO, IO_Pin(io), hi);
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
    gpio_ll_set_level(&GPIO, IO_Pin(io), 1);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
    gpio_ll_set_level(&GPIO, IO_Pin(io), 0);
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }
    uint16_t pin = IO_Pin(io);
    gpio_ll_set_level(&GPIO, pin, !gpio_ll_get_level(&GPIO, pin));
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }

    uint16_t pin = IO_Pin(io);
    uint8_t mode = cfg & 0x03;
    uint8_t pupd = (cfg >> 5) & 0x03;

    // Select GPIO function for this pad
    esp_rom_gpio_pad_select_gpio(pin);

    if (mode == BF_GPIO_MODE_OUTPUT) {
        gpio_ll_output_enable(&GPIO, pin);
        gpio_ll_input_disable(&GPIO, pin);
    } else {
        gpio_ll_output_disable(&GPIO, pin);
        gpio_ll_input_enable(&GPIO, pin);
    }

    if (pupd & BF_GPIO_PULLUP) {
        gpio_ll_pullup_en(&GPIO, pin);
    } else {
        gpio_ll_pullup_dis(&GPIO, pin);
    }
    if (pupd & BF_GPIO_PULLDOWN) {
        gpio_ll_pulldown_en(&GPIO, pin);
    } else {
        gpio_ll_pulldown_dis(&GPIO, pin);
    }
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
