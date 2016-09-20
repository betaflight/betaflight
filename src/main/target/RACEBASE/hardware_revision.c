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
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "drivers/system.h"
#include "drivers/bus_spi.h"
#include "drivers/sensor.h"
#include "drivers/io.h"
#include "drivers/gpio.h"
#include "drivers/exti.h"
#include "drivers/accgyro.h"
#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_mpu6500.h"

#include "hardware_revision.h"

uint8_t hardwareRevision = 1;

void detectHardwareRevision(void)
{
    gpio_config_t gpio;

    // GYRO CS as output
    gpio.pin = GPIO_Pin_5;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(GPIOB, &gpio);
    GPIO_SetBits(GPIOB,   GPIO_Pin_5);

    gpio.pin = GPIO_Pin_7;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(GPIOA, &gpio);
    GPIO_SetBits(GPIOA,   GPIO_Pin_7);

    gpio.pin = GPIO_Pin_12;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(GPIOB, &gpio);
    GPIO_SetBits(GPIOB,   GPIO_Pin_12);
}


void updateHardwareRevision(void)
{

}

