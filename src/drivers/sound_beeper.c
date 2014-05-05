
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "system_common.h"
#include "gpio_common.h"

#include "sound_beeper.h"


#ifdef BUZZER

void (* systemBeepPtr)(bool onoff) = NULL;

static void beepNormal(bool onoff)
{
    if (onoff) {
        digitalLo(BEEP_GPIO, BEEP_PIN);
    } else {
        digitalHi(BEEP_GPIO, BEEP_PIN);
    }
}

static void beepInverted(bool onoff)
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

static inline bool isBuzzerOutputInverted(void)
{
#ifdef BUZZER_INVERTED
    return true;
#else
    // Naze rev5 needs inverted beeper.
    return hse_value == 12000000;
#endif
}

void beeperInit(void)
{
#ifdef BUZZER
    // Configure gpio
    if (isBuzzerOutputInverted())
        systemBeepPtr = beepInverted;
    else
        systemBeepPtr = beepNormal;
    BEEP_OFF;
#endif
}
