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
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    // SCK as output
    GPIO_InitStructure.GPIO_Pin = dev->sck_pin;
    GPIO_Init(dev->sck_gpio, &GPIO_InitStructure);
    // MOSI as output
    GPIO_InitStructure.GPIO_Pin = dev->mosi_pin;
    GPIO_Init(dev->mosi_gpio, &GPIO_InitStructure);
    // MISO as input
    GPIO_InitStructure.GPIO_Pin = dev->miso_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(dev->miso_gpio, &GPIO_InitStructure);
#ifdef SOFTSPI_NSS_PIN
    // NSS as output
    GPIO_InitStructure.GPIO_Pin = dev->nss_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(dev->nss_gpio, &GPIO_InitStructure);
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
