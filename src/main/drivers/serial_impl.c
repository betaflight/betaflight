#include <stdbool.h>
#include <stdint.h>

#include "platform.h"


#include "io/serial.h"

#include "serial.h"
#include "serial_impl.h"

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


