#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "serial.h"
#include "serial_impl.h"

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

// convert options into pin pull mode (up/down/none)
serialPullMode_t serialOptions_pull(portOptions_e options)
{
    if (options & SERIAL_BIDIR_NOPULL) {
        return serialPullNone;                            // explicit nopull
    } else if (options & (SERIAL_INVERTED | SERIAL_BIDIR_PP_PD)) {
        return serialPullDown;
    } else {
        return serialPullUp;
    }
}

// is pushPull mode necessary
bool serialOptions_pushPull(portOptions_e options)
{
    return options & (SERIAL_INVERTED | SERIAL_BIDIR_PP | SERIAL_BIDIR_PP_PD);
}


