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
#ifdef SOFTSPI_NSS_PIN
    GPIO_TypeDef* nss_gpio;
    uint16_t nss_pin;
#endif
    GPIO_TypeDef* sck_gpio;
    uint16_t sck_pin;
    GPIO_TypeDef* mosi_gpio;
    uint16_t mosi_pin;
    GPIO_TypeDef* miso_gpio;
    uint16_t miso_pin;
} softSPIDevice_t;

// Convenience macros to set pins high or low,
// where 'dev' is an instance of SoftSPIDevice
/*#define SOFTSPI_CSN_LO(dev)  {GPIO_ResetBits(dev.csn_gpio, dev.nss_pin);}
#define SOFTSPI_CSN_HI(dev)  {GPIO_SetBits(dev.csn_gpio, dev.nss_pin);}

#define SOFTSPI_SCK_LO(dev)  {GPIO_ResetBits(dev.sck_gpio, dev.sck_pin);}
#define SOFTSPI_SCK_HI(dev)  {GPIO_SetBits(dev.sck_gpio, dev.sck_pin);}

#define SOFTSPI_MISO_LO(dev) {GPIO_ResetBits(dev.miso_gpio, dev.miso_pin);}
#define SOFTSPI_MISO_HI(dev) {GPIO_SetBits(dev.miso_gpio, dev.miso_pin);}

#define SOFTSPI_MOSI_LO(dev) {GPIO_ResetBits(dev.mosi_gpio, dev.mosi_pin);}
#define SOFTSPI_MOSI_HI(dev) {GPIO_SetBits(dev.mosi_gpio, dev.mosi_pin);}

#define SOFTSPI_MISO_R(dev) {GPIO_ReadInputDataBit(dev.miso_gpio, dev.miso_pin);}
*/

void softSpiInit(const softSPIDevice_t *dev);
uint8_t softSpiTransferByte(const softSPIDevice_t *dev, uint8_t data);
//bool softSpiInit(SPI_TypeDef *instance);
//uint8_t softSpiTransferByte(SPI_TypeDef *instance, uint8_t data);
