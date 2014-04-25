
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "gpio_common.h"

void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config)
{
    uint32_t pinpos;
    for (pinpos = 0; pinpos < 16; pinpos++) {
        // are we doing this pin?
        if (config->pin & (0x1 << pinpos)) {
            // reference CRL or CRH, depending whether pin number is 0..7 or 8..15
            __IO uint32_t *cr = &gpio->CRL + (pinpos / 8);
            // mask out extra bits from pinmode, leaving just CNF+MODE
            uint32_t currentmode = config->mode & 0x0F;
            // offset to CNF and MODE portions of CRx register
            uint32_t shift = (pinpos % 8) * 4;
            // Read out current CRx value
            uint32_t tmp = *cr;
            // if we're in output mode, add speed too.
            if (config->mode & 0x10)
                currentmode |= config->speed;
            // Mask out 4 bits
            tmp &= ~(0xF << shift);
            // apply current pinmode
            tmp |= currentmode << shift;
            *cr = tmp;
            // Special handling for IPD/IPU
            if (config->mode == Mode_IPD) {
                gpio->ODR &= ~(1U << pinpos);
            } else if (config->mode == Mode_IPU) {
                gpio->ODR |= (1U << pinpos);
            }
        }
    }
}

void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc)
{
    uint32_t tmp = 0x00;

    tmp = ((uint32_t)0x0F) << (0x04 * (pinsrc & (uint8_t)0x03));
    AFIO->EXTICR[pinsrc >> 0x02] &= ~tmp;
    AFIO->EXTICR[pinsrc >> 0x02] |= (((uint32_t)portsrc) << (0x04 * (pinsrc & (uint8_t)0x03)));
}
