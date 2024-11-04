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
#include <limits.h>

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
        const unsigned idx = identifier - SERIAL_PORT_USART1;
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
        const unsigned idx = identifier - SERIAL_PORT_LPUART1;
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
        // sotserials always start from 1, without holes
        return SERIALTYPE_SOFTSERIAL;
    }
#endif
    return SERIALTYPE_INVALID;
}


static const struct SerialTypeInfo {
    resourceOwner_e owner;
    serialPortIdentifier_e firstId;
    int8_t resourceOffset;
} serialTypeMap[] = {
    [SERIALTYPE_USB_VCP] = { OWNER_FREE /* no owner*/, SERIAL_PORT_USB_VCP, -1 },
    [SERIALTYPE_UART] = { OWNER_SERIAL_TX, SERIAL_PORT_USART1, RESOURCE_UART_OFFSET },
    [SERIALTYPE_LPUART] = { OWNER_LPUART_TX, SERIAL_PORT_LPUART1, RESOURCE_LPUART_OFFSET },
    [SERIALTYPE_SOFTSERIAL] = { OWNER_SOFTSERIAL_TX, SERIAL_PORT_SOFTSERIAL1, RESOURCE_SOFTSERIAL_OFFSET },
};

STATIC_ASSERT(ARRAYLEN(serialTypeMap) == SERIALTYPE_COUNT, "type table mismatch");

static const struct SerialTypeInfo* serialTypeInfo(serialPortIdentifier_e identifier)
{
    const serialType_e type = serialType(identifier);
    if (type == SERIALTYPE_INVALID) {
        return NULL;
    }
    return serialTypeMap + type;
}

// resourceOwner_e for this serial (UART/LPUART/SOFTSERIAL/..)
// Used together with serialOwnerIndex to identify claimed resources
// Tx member is returned, Rx is always Tx + 1
// OWNER_FREE is returned for serials without owner (VCP)
resourceOwner_e serialOwnerTxRx(serialPortIdentifier_e identifier)
{
    const struct SerialTypeInfo* inf = serialTypeInfo(identifier);
    return inf ? inf->owner : OWNER_FREE;
}


// return index used to claim given resource. Returned value is 1 based, for IOInit and similar
// 0 is returned when given port is not defined or if it is singleton port (USB)
int serialOwnerIndex(serialPortIdentifier_e identifier)
{
    const struct SerialTypeInfo* inf = serialTypeInfo(identifier);
    if (!inf || inf->owner == OWNER_FREE) {
        return 0;
    }
    return RESOURCE_INDEX(identifier - inf->firstId);
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
    const struct SerialTypeInfo* inf = serialTypeInfo(identifier);
    if (!inf || inf->resourceOffset < 0) {
        return -1;
    }
    return identifier - inf->firstId + inf->resourceOffset;
}
