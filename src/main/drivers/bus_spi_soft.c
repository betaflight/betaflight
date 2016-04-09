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

#include <platform.h>

#include "build_config.h"

#ifdef USE_SOFTSPI

#include "gpio.h"
#include "bus_spi_soft.h"


void softSpiInit(const softSPIDevice_t *dev)
{

#ifdef STM32F303xC
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    // SCK as output
    GPIO_InitStructure.GPIO_Pin = dev->sck_pin;
    GPIO_InitStructure(dev->sck_gpio, &GPIO_InitStructure);
    // MOSI as output
    GPIO_InitStructure.GPIO_Pin = dev->mosi_pin;
    GPIO_Init(dev->mosi_gpio, &GPIO_InitStructure);
    // MISO as input
    GPIO_InitStructure.GPIO_Pin = dev->miso_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(dev->miso_gpio, &GPIO_InitStructure);
#ifdef SOFTSPI_NSS_PIN
    // NSS as gpio not slave select
    GPIO_InitStructure.GPIO_Pin = dev->nss_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure(dev->csn_gpio, &GPIO_InitStructure);
#endif
#endif
#ifdef STM32F10X
    gpio_config_t gpio;
    gpio.speed = Speed_50MHz;
    gpio.mode = Mode_AF_PP;
    // SCK as output
    gpio.pin = dev->sck_pin;
    gpioInit(dev->sck_gpio, &gpio);
    // MOSI as output
    gpio.pin = dev->mosi_pin;
    gpioInit(dev->mosi_gpio, &gpio);
    // MISO as input
    gpio.pin = dev->miso_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(dev->miso_gpio, &gpio);
#ifdef SOFTSPI_NSS_PIN
    // NSS as output
    gpio.pin = dev->nss_pin;
    gpio.mode = Mode_Out_PP;
    gpioInit(dev->csn_gpio, &gpio);
#endif
#endif
}

uint8_t softSpiTransferByte(const softSPIDevice_t *dev, uint8_t byte)
{
    for(int ii = 0; ii < 8; ++ii) {
        if (byte & 0x80) {
            GPIO_SetBits(dev->mosi_gpio, dev->mosi_pin);
        } else {
            GPIO_ResetBits(dev->mosi_gpio, dev->mosi_pin);
        }
        GPIO_SetBits(dev->sck_gpio, dev->sck_pin);
        byte <<= 1;
        if (GPIO_ReadInputDataBit(dev->miso_gpio, dev->miso_pin) == 1) {
            byte |= 1;
        }
        GPIO_ResetBits(dev->sck_gpio, dev->sck_pin);
    }
    return byte;
}
#endif
