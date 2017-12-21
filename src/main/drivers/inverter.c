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

#include "io/serial.h" // For SERIAL_PORT_IDENTIFIER_TO_INDEX
#include "drivers/io.h"
#include "drivers/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "inverter.h"

#ifdef USE_INVERTER

static const serialPinConfig_t *pSerialPinConfig;

static void inverterSet(int identifier, bool on)
{
    IO_t pin = IOGetByTag(pSerialPinConfig->ioTagInverter[SERIAL_PORT_IDENTIFIER_TO_INDEX(identifier)]);

    if (pin) {
        IOWrite(pin, on);
    }
}

static void initInverter(int identifier)
{
    int uartIndex = SERIAL_PORT_IDENTIFIER_TO_INDEX(identifier);
    IO_t pin = IOGetByTag(pSerialPinConfig->ioTagInverter[uartIndex]);

    if (pin) {
        IOInit(pin, OWNER_INVERTER, RESOURCE_INDEX(uartIndex));
        IOConfigGPIO(pin, IOCFG_OUT_PP);

        inverterSet(identifier, false);
    }
}

void initInverters(const serialPinConfig_t *serialPinConfigToUse)
{
    pSerialPinConfig = serialPinConfigToUse;

#ifdef USE_UART1
    initInverter(SERIAL_PORT_USART1);
#endif

#ifdef USE_UART2
    initInverter(SERIAL_PORT_USART2);
#endif

#ifdef USE_UART3
    initInverter(SERIAL_PORT_USART3);
#endif

#ifdef USE_UART4
    initInverter(SERIAL_PORT_UART4);
#endif

#ifdef USE_UART5
    initInverter(SERIAL_PORT_UART5);
#endif

#ifdef USE_UART6
    initInverter(SERIAL_PORT_USART6);
#endif
}

void enableInverter(USART_TypeDef *USARTx, bool on)
{
    int identifier = SERIAL_PORT_NONE;

#ifdef USE_UART1
    if (USARTx == USART1) {
        identifier = SERIAL_PORT_USART1;
    }
#endif

#ifdef USE_UART2
    if (USARTx == USART2) {
        identifier = SERIAL_PORT_USART2;
    }
#endif

#ifdef USE_UART3
    if (USARTx == USART3) {
        identifier = SERIAL_PORT_USART3;
    }
#endif

#ifdef USE_UART4
    if (USARTx == UART4) {
        identifier = SERIAL_PORT_UART4;
    }
#endif

#ifdef USE_UART5
    if (USARTx == UART5) {
        identifier = SERIAL_PORT_UART5;
    }
#endif

#ifdef USE_UART6
    if (USARTx == USART6) {
        identifier = SERIAL_PORT_USART6;
    }
#endif

    if (identifier != SERIAL_PORT_NONE) {
        inverterSet(identifier, on);
    }
}
#endif // USE_INVERTER
