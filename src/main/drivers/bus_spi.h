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

#ifndef SPI1_GPIO
#define SPI1_GPIO               GPIOA
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOA
#define SPI1_NSS_GPIO           GPIOA
#define SPI1_NSS_PIN            GPIO_Pin_4
#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource4
#define SPI1_SCK_PIN            GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource5
#define SPI1_MISO_PIN           GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource6
#define SPI1_MOSI_PIN           GPIO_Pin_7
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource7
#endif

#ifndef SPI2_GPIO
#define SPI2_GPIO               GPIOB
#define SPI2_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI2_NSS_PIN            GPIO_Pin_12
#define SPI2_NSS_PIN_SOURCE     GPIO_PinSource12
#define SPI2_SCK_PIN            GPIO_Pin_13
#define SPI2_SCK_PIN_SOURCE     GPIO_PinSource13
#define SPI2_MISO_PIN           GPIO_Pin_14
#define SPI2_MISO_PIN_SOURCE    GPIO_PinSource14
#define SPI2_MOSI_PIN           GPIO_Pin_15
#define SPI2_MOSI_PIN_SOURCE    GPIO_PinSource15
#endif

#if defined(USE_SPI_DEVICE_3) && defined(STM32F303xC)

#ifndef SPI3_GPIO
#define SPI3_GPIO               GPIOB
#define SPI3_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI3_SCK_PIN            GPIO_Pin_3
#define SPI3_SCK_PIN_SOURCE     GPIO_PinSource3
#define SPI3_MISO_PIN           GPIO_Pin_4
#define SPI3_MISO_PIN_SOURCE    GPIO_PinSource4
#define SPI3_MOSI_PIN           GPIO_Pin_5
#define SPI3_MOSI_PIN_SOURCE    GPIO_PinSource5
#endif

#ifndef SPI3_NSS_GPIO
#define SPI3_NSS_GPIO           GPIOA
#define SPI3_NSS_PERIPHERAL     RCC_AHBPeriph_GPIOA
#define SPI3_NSS_PIN            GPIO_Pin_15
#define SPI3_NSS_PIN_SOURCE     GPIO_PinSource15
#endif


#endif

typedef enum {
    SPI_CLOCK_INITIALIZATON = 256,
    SPI_CLOCK_SLOW          = 128, //00.56250 MHz
    SPI_CLOCK_STANDARD      = 4,   //09.00000 MHz
    SPI_CLOCK_FAST          = 2,   //18.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2,   //18.00000 MHz
} SPIClockDivider_e;

bool spiInit(SPI_TypeDef *instance);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t in);
bool spiIsBusBusy(SPI_TypeDef *instance);

void spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len);
