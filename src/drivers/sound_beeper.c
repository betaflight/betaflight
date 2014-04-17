
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "system_common.h"
#include "gpio_common.h"

#include "sound_beeper.h"


#ifdef BUZZER

void (* systemBeepPtr)(bool onoff) = NULL;

static void beepRev4(bool onoff)
{
    if (onoff) {
        digitalLo(BEEP_GPIO, BEEP_PIN);
    } else {
        digitalHi(BEEP_GPIO, BEEP_PIN);
    }
}

static void beepRev5(bool onoff)
{
    if (onoff) {
        digitalHi(BEEP_GPIO, BEEP_PIN);
    } else {
        digitalLo(BEEP_GPIO, BEEP_PIN);
    }
}
#endif

void systemBeep(bool onoff)
{
#ifdef BUZZER
    systemBeepPtr(onoff);
#endif
}

void beeperInit(void)
{
#ifdef BUZZER
    // Configure gpio
    // rev5 needs inverted beeper. oops.
    if (hse_value == 12000000)
        systemBeepPtr = beepRev5;
    else
        systemBeepPtr = beepRev4;
    BEEP_OFF;
#endif
}
