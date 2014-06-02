#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "gpio.h"

#include "light_led.h"

void ledInit(void)
{
    uint32_t i;

    struct {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup[] = {
#ifdef LED0
        {
            .gpio = LED0_GPIO,
            .cfg = { LED0_PIN, Mode_Out_PP, Speed_2MHz }
        },
#endif
#ifdef LED1
        {
            .gpio = LED1_GPIO,
            .cfg = { LED1_PIN, Mode_Out_PP, Speed_2MHz }
        }
#endif
    };

    uint8_t gpio_count = sizeof(gpio_setup) / sizeof(gpio_setup[0]);

#ifdef LED0
    RCC_AHBPeriphClockCmd(LED0_PERIPHERAL, ENABLE);
#endif
#ifdef LED1
    RCC_AHBPeriphClockCmd(LED1_PERIPHERAL, ENABLE);
#endif

    LED0_OFF;
    LED1_OFF;

    for (i = 0; i < gpio_count; i++) {
        gpioInit(gpio_setup[i].gpio, &gpio_setup[i].cfg);
    }
}

