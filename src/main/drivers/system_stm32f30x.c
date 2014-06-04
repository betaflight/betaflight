
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "gpio.h"

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader)
{
    if (toBootloader) {
        // 1FFFF000 -> 20000200 -> SP
        // 1FFFF004 -> 1FFFF021 -> PC

        *((uint32_t *)0x20009FFC) = 0xDEADBEEF; // 40KB SRAM STM32F30X
    }

    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}


void enableGPIOPowerUsageAndNoiseReductions(void)
{
    RCC_AHBPeriphClockCmd(
        RCC_AHBPeriph_GPIOA |
        RCC_AHBPeriph_GPIOB |
        RCC_AHBPeriph_GPIOC |
        RCC_AHBPeriph_GPIOD |
        RCC_AHBPeriph_GPIOE |
        RCC_AHBPeriph_GPIOF,
        ENABLE
    );

    gpio_config_t gpio;

    gpio.mode = Mode_AIN;

    gpio.pin = Pin_All & ~(Pin_13|Pin_14|Pin_15);  // Leave JTAG pins alone
    gpioInit(GPIOA, &gpio);

    gpio.pin = Pin_All;
    gpioInit(GPIOB, &gpio);
    gpioInit(GPIOC, &gpio);
    gpioInit(GPIOD, &gpio);
    gpioInit(GPIOE, &gpio);
    gpioInit(GPIOF, &gpio);
}
