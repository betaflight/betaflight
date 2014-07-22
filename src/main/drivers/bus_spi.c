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

#include "gpio.h"

#include "bus_spi.h"

static volatile uint16_t spi1ErrorCount = 0;
static volatile uint16_t spi2ErrorCount = 0;
#ifdef STM32F303xC
static volatile uint16_t spi3ErrorCount = 0;
#endif

void initSpi1(void)
{
    // Specific to the STM32F103
    // SPI1 Driver
    // PA7    17    SPI1_MOSI
    // PA6    16    SPI1_MISO
    // PA5    15    SPI1_SCK
    // PA4    14    SPI1_NSS

    gpio_config_t gpio;
    SPI_InitTypeDef spi;

    // Enable SPI1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);

    // MOSI + SCK as output
    gpio.mode = Mode_AF_PP;
    gpio.pin = Pin_7 | Pin_5;
    gpio.speed = Speed_50MHz;
    gpioInit(GPIOA, &gpio);
    // MISO as input
    gpio.pin = Pin_6;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOA, &gpio);
    // NSS as gpio slave select
    gpio.pin = Pin_4;
    gpio.mode = Mode_Out_PP;
    gpioInit(GPIOA, &gpio);

    // Init SPI2 hardware
    SPI_I2S_DeInit(SPI1);

    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}

void initSpi2(void)
{

    // Specific to the STM32F103
    // SPI2 Driver
    // PB15     28      SPI2_MOSI
    // PB14     27      SPI2_MISO
    // PB13     26      SPI2_SCK
    // PB12     25      SPI2_NSS

    gpio_config_t gpio;
    SPI_InitTypeDef spi;

    // Enable SPI2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);

    // MOSI + SCK as output
    gpio.mode = Mode_AF_PP;
    gpio.pin = Pin_13 | Pin_15;
    gpio.speed = Speed_50MHz;
    gpioInit(GPIOB, &gpio);
    // MISO as input
    gpio.pin = Pin_14;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOB, &gpio);
    // NSS as gpio slave select
    gpio.pin = Pin_12;
    gpio.mode = Mode_Out_PP;
    gpioInit(GPIOB, &gpio);

    // Init SPI2 hardware
    SPI_I2S_DeInit(SPI2);

    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

    SPI_Init(SPI2, &spi);
    SPI_Cmd(SPI2, ENABLE);
}

bool spiInit(SPI_TypeDef *instance)
{
    if (instance == SPI1) {
        initSpi1();
    } else if (instance == SPI2) {
        initSpi2();
    } else {
        return false;
    }

    return true;
}

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    if (instance == SPI1) {
        spi1ErrorCount++;
    } else if (instance == SPI2) {
        spi2ErrorCount++;
    }
#ifdef STM32F303xC
    else {
        spi3ErrorCount++;
        return spi3ErrorCount;
    }
#endif
    return -1;
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data)
{
    uint16_t spiTimeout = 1000;

    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

#ifdef STM32F303xC
    SPI_SendData8(instance, data);
#endif
#ifdef STM32F10X_MD
    SPI_I2S_SendData(instance, data);
#endif
    spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

#ifdef STM32F303xC
    return ((uint8_t)SPI_ReceiveData8(instance));
#endif
#ifdef STM32F10X_MD
    return ((uint8_t)SPI_I2S_ReceiveData(instance));
#endif
    }

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, uint8_t *in, int len)
{
    uint16_t spiTimeout = 1000;

    uint8_t b;
    instance->DR;
    while (len--) {
        b = in ? *(in++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
#ifdef STM32F303xC
        SPI_I2S_SendData16(instance, b);
#endif
#ifdef STM32F10X_MD
        SPI_I2S_SendData(instance, b);
#endif
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
#ifdef STM32F303xC
        b = SPI_I2S_ReceiveData16(instance);
#endif
#ifdef STM32F10X_MD
        b = SPI_I2S_ReceiveData(instance);
#endif
        if (out)
            *(out++) = b;
    }

    return true;
}


void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;

    SPI_Cmd(instance, DISABLE);

    tempRegister = instance->CR1;

    switch (divisor) {
        case 2:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_2;
            break;

        case 4:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_4;
            break;

        case 8:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_8;
            break;

        case 16:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_16;
            break;

        case 32:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_32;
            break;

        case 64:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_64;
            break;

        case 128:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_128;
            break;

        case 256:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_256;
            break;
    }

    instance->CR1 = tempRegister;

    SPI_Cmd(instance, ENABLE);
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    if (instance == SPI1) {
        return spi1ErrorCount;
    } else if (instance == SPI2) {
        return spi2ErrorCount;
    }
    return 0;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    if (instance == SPI1) {
        spi1ErrorCount = 0;
    } else if (instance == SPI2) {
        spi2ErrorCount = 0;
    }
}

