#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader)
{
    if (toBootloader) {
        // 1FFFF000 -> 20000200 -> SP
        // 1FFFF004 -> 1FFFF021 -> PC

        *((uint32_t *)0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
    }

    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}
