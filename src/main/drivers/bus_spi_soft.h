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

#pragma once

typedef struct softSPIDevice_s {
    GPIO_TypeDef* sck_gpio;
    uint16_t sck_pin;
    GPIO_TypeDef* mosi_gpio;
    uint16_t mosi_pin;
    GPIO_TypeDef* miso_gpio;
    uint16_t miso_pin;
#ifdef SOFTSPI_NSS_PIN
    GPIO_TypeDef* nss_gpio;
    uint16_t nss_pin;
#endif
} softSPIDevice_t;


void softSpiInit(const softSPIDevice_t *dev);
uint8_t softSpiTransferByte(const softSPIDevice_t *dev, uint8_t data);
