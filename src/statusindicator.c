
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/system_common.h"
#include "drivers/gpio_common.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"

#include "statusindicator.h"

void blinkLedAndSoundBeeper(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;            // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}
