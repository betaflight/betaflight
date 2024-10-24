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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "io/serial.h"

// convert identifier into port type
serialType_e serialType(serialPortIdentifier_e identifier)
{
#ifdef USE_VCP
    if (identifier == SERIAL_PORT_USB_VCP) {
        return SERIALTYPE_USB_VCP;
    }
#endif
#ifdef USE_UART
    if (identifier >= SERIAL_PORT_USART1 && identifier < SERIAL_PORT_USART1 + SERIAL_UART_MAX) {
        unsigned idx = identifier - SERIAL_PORT_USART1;
        if (BIT(idx) & SERIAL_UART_MASK) {
            return SERIALTYPE_UART;
        } else {
            // no other type in this range
            return SERIALTYPE_INVALID;
        }
    }
#endif
#ifdef USE_LPUART
    if (identifier >= SERIAL_PORT_LPUART1 && identifier < SERIAL_PORT_LPUART1 + SERIAL_LPUART_MAX) {
        unsigned idx = identifier - SERIAL_PORT_LPUART1;
        if (BIT(idx) & SERIAL_LPUART_MASK) {
            return SERIALTYPE_LPUART;
        } else {
            // no other type in this range
            return SERIALTYPE_INVALID;
        }
    }
#endif
#ifdef USE_SOFTSERIAL
    if (identifier >= SERIAL_PORT_SOFTSERIAL1 && identifier < SERIAL_PORT_SOFTSERIAL1 + SERIAL_SOFTSERIAL_MAX) {
        // sotserials from 1, no holes
        return SERIALTYPE_SOFTSERIAL;
    }
#endif
    return SERIALTYPE_INVALID;
}


// resourceOwner_e for this serial (UART/LPUART/SOFTSERIAL/..)
// Used together with serialOwnerIndex to identify claimed resources
// Tx member is returned, Rx is always Tx + 1
resourceOwner_e serialOwnerTxRx(serialPortIdentifier_e identifier)
{
    switch (serialType(identifier)) {
    case SERIALTYPE_UART:
        return OWNER_SERIAL_TX;
    case SERIALTYPE_LPUART:
        return OWNER_LPUART_TX;
    case SERIALTYPE_SOFTSERIAL:
        return OWNER_SOFTSERIAL_TX;
    default:
        return 0;
    }
}


// return index used to claim given resource. Returned value is 1 based, for IOInit and similar
// 0 is returned for given port is not defined and for singleton ports (USB)
int serialOwnerIndex(serialPortIdentifier_e identifier)
{
    serialPortIdentifier_e firstId[SERIALTYPE_COUNT] = {
        [SERIALTYPE_USB_VCP] = SERIAL_PORT_USB_VCP + RESOURCE_INDEX(0), // make it return 0 for VCP
        [SERIALTYPE_UART] = SERIAL_PORT_USART1,
        [SERIALTYPE_LPUART] = SERIAL_PORT_LPUART1,
        [SERIALTYPE_SOFTSERIAL] = SERIAL_PORT_SOFTSERIAL1,
    };
    STATIC_ASSERT(ARRAYLEN(firstId) == SERIALTYPE_COUNT, "firstId table mismatch");

    const serialType_e type = serialType(identifier);
    if (type == SERIALTYPE_INVALID) {
        return 0;  // TODO - maybe -1 ?
    }
    return RESOURCE_INDEX(identifier - firstId[type]);
}

// map identifier into index used to store port resources:
//   pins(RX,TX), external inversion, port DMA configuration
// order is UART, LPUART, SOFTSERIAL, with each group using index
//  coresponding to port name (UART1 -> 0, UART5 -> 4, but LPUART -> 5 if
//  there is no UART6 and higher on given target.
// -1 is returned if given port is not defined or is not using resources
// some code uses this ordering for optimizations, be carefull if reordering is necessary
int serialResourceIndex(serialPortIdentifier_e identifier)
{
    switch (serialType(identifier)) {
    case SERIALTYPE_UART:
        return RESOURCE_UART_OFFSET + identifier - SERIAL_PORT_UART1;
    case SERIALTYPE_LPUART:
        return RESOURCE_LPUART_OFFSET + identifier - SERIAL_PORT_LPUART1;
    case SERIALTYPE_SOFTSERIAL:
        return RESOURCE_SOFTSERIAL_OFFSET + identifier - SERIAL_PORT_SOFTSERIAL1;
    default:
        return -1;
    }
}
