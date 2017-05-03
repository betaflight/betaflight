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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"
#include "io_impl.h"

#include "inverter.h"

#ifdef USE_INVERTER
static void inverterSet(IO_t pin, bool on)
{
    IOWrite(pin, on);
}

static void initInverter(ioTag_t ioTag)
{
    IO_t pin = IOGetByTag(ioTag);
    IOInit(pin, OWNER_INVERTER, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(pin, IOCFG_OUT_PP);

    inverterSet(pin, false);
}
#endif

void initInverters(void)
{
#ifdef INVERTER_PIN_UART1
    initInverter(IO_TAG(INVERTER_PIN_UART1));
#endif

#ifdef INVERTER_PIN_UART2
    initInverter(IO_TAG(INVERTER_PIN_UART2));
#endif

#ifdef INVERTER_PIN_UART3
    initInverter(IO_TAG(INVERTER_PIN_UART3));
#endif

#ifdef INVERTER_PIN_USART4
    initInverter(IO_TAG(INVERTER_PIN_USART4));
#endif

#ifdef INVERTER_PIN_USART5
    initInverter(IO_TAG(INVERTER_PIN_USART5));
#endif

#ifdef INVERTER_PIN_UART6
    initInverter(IO_TAG(INVERTER_PIN_UART6));
#endif
}

void enableInverter(USART_TypeDef *USARTx, bool on)
{
#ifdef USE_INVERTER
    IO_t pin = IO_NONE;

#ifdef INVERTER_PIN_UART1
    if (USARTx == USART1) {
        pin = IOGetByTag(IO_TAG(INVERTER_PIN_UART1));
    }
#endif

#ifdef INVERTER_PIN_UART2
    if (USARTx == USART2) {
        pin = IOGetByTag(IO_TAG(INVERTER_PIN_UART2));
    }
#endif

#ifdef INVERTER_PIN_UART3
    if (USARTx == USART3) {
        pin = IOGetByTag(IO_TAG(INVERTER_PIN_UART3));
    }
#endif

#ifdef INVERTER_PIN_USART4
    if (USARTx == USART4) {
        pin = IOGetByTag(IO_TAG(INVERTER_PIN_USART4));
    }
#endif

#ifdef INVERTER_PIN_USART5
    if (USARTx == USART5) {
        pin = IOGetByTag(IO_TAG(INVERTER_PIN_USART5));
    }
#endif

#ifdef INVERTER_PIN_UART6
    if (USARTx == USART6) {
        pin = IOGetByTag(IO_TAG(INVERTER_PIN_UART6));
    }
#endif

    inverterSet(pin, on);
#else
    UNUSED(USARTx);
    UNUSED(on);
#endif
}
