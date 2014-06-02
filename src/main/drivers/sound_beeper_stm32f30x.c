#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "gpio.h"

#include "sound_beeper.h"

void initBeeperHardware(void)
{
#ifdef BUZZER
    struct {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup = {
        .gpio = BEEP_GPIO,
        .cfg = { BEEP_PIN, Mode_Out_OD, Speed_2MHz }
    };

    RCC_AHBPeriphClockCmd(BEEP_PERIPHERAL, ENABLE);

    gpioInit(gpio_setup.gpio, &gpio_setup.cfg);

#endif
}
