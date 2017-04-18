/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/gpio.h"

#define MODE_OFFSET 0
#define PUPD_OFFSET 5

#define MODE_MASK ((1|2|16))
#define PUPD_MASK ((1|2))

void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    uint32_t pinIndex;
    for (pinIndex = 0; pinIndex < 16; pinIndex++) {
        // are we doing this pin?
        uint32_t pinMask = (0x1 << pinIndex);
        if (config->pin & pinMask) {

            GPIO_InitStructure.Pin =  pinMask;
            GPIO_InitStructure.Mode = (config->mode >> MODE_OFFSET) & MODE_MASK;

            uint32_t speed = GPIO_SPEED_FREQ_MEDIUM;
            switch (config->speed) {
                case Speed_10MHz:
                    speed = GPIO_SPEED_FREQ_MEDIUM;
                    break;
                case Speed_2MHz:
                    speed = GPIO_SPEED_FREQ_LOW;
                    break;
                case Speed_50MHz:
                    speed = GPIO_SPEED_FREQ_HIGH;
                    break;
            }

            GPIO_InitStructure.Speed = speed;
            GPIO_InitStructure.Pull = (config->mode >> PUPD_OFFSET) & PUPD_MASK;
            HAL_GPIO_Init(gpio, &GPIO_InitStructure);
        }
    }
}

void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc)
{
    (void)portsrc;
    (void)pinsrc;
    //SYSCFG_EXTILineConfig(portsrc, pinsrc);
}
