#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "io/serial.h"

#include "serial.h"
#include "serial_impl.h"

// convert options into pin pull mode (up/down/none)
serialPullMode_t serialOptions_pull(portOptions_e options)
{
    // handle SmartAudio first - different SA versions need different values
    // add more cases here if necessary
    if (options & SERIAL_PULL_SMARTAUDIO) {
#ifdef USE_SMARTAUDIO_NOPULLDOWN
        return serialPullNone;
#else
        return serialPullDown;
#endif
    }
    if (options & SERIAL_PULL_NONE) {
        return serialPullNone;                            // explicit nopull
    } else if (options & SERIAL_INVERTED) {
        return serialPullDown;
    } else {
        return serialPullUp;
    }
}

// is pushPull mode necessary
bool serialOptions_pushPull(portOptions_e options)
{
    return options & (SERIAL_INVERTED | SERIAL_BIDIR_PP);
}

