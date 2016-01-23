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

#include "gpio.h"

#include "bus_spi.h"

static volatile uint16_t spi1ErrorCount = 0;
static volatile uint16_t spi2ErrorCount = 0;
#ifdef STM32F303xC
static volatile uint16_t spi3ErrorCount = 0;
#endif

#ifdef USE_SPI_DEVICE_1

#ifndef SPI1_GPIO
#define SPI1_GPIO               GPIOA
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOA
#define SPI1_NSS_PIN            GPIO_Pin_4
#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource4
#define SPI1_SCK_PIN            GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource5
#define SPI1_MISO_PIN           GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource6
#define SPI1_MOSI_PIN           GPIO_Pin_7
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource7
#endif

void initSpi1(void)
{
    // Specific to the STM32F103
    // SPI1 Driver
    // PA4    14    SPI1_NSS
    // PA5    15    SPI1_SCK
    // PA6    16    SPI1_MISO
    // PA7    17    SPI1_MOSI

    SPI_InitTypeDef spi;

    // Enable SPI1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);


#ifdef STM32F303xC
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(SPI1_GPIO_PERIPHERAL, ENABLE);

    GPIO_PinAFConfig(SPI1_GPIO, SPI1_SCK_PIN_SOURCE, GPIO_AF_5);
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MISO_PIN_SOURCE, GPIO_AF_5);
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MOSI_PIN_SOURCE, GPIO_AF_5);
#ifdef SPI1_NSS_PIN_SOURCE
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_NSS_PIN_SOURCE, GPIO_AF_5);
#endif
    // Init pins
    GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

#ifdef SPI1_NSS_PIN
    GPIO_InitStructure.GPIO_Pin = SPI1_NSS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
#endif

#endif

#ifdef STM32F10X
    gpio_config_t gpio;
    // MOSI + SCK as output
    gpio.mode = Mode_AF_PP;
    gpio.pin = SPI1_MOSI_PIN | SPI1_SCK_PIN;
    gpio.speed = Speed_50MHz;
    gpioInit(GPIOA, &gpio);
    // MISO as input
    gpio.pin = SPI1_MISO_PIN;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOA, &gpio);
#ifdef SPI1_NSS_PIN
    // NSS as gpio slave select
    gpio.pin = SPI1_NSS_PIN;
    gpio.mode = Mode_Out_PP;
    gpioInit(GPIOA, &gpio);
#endif
#endif

    // Init SPI hardware
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

#ifdef STM32F303xC
    // Configure for 8-bit reads.
    SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
#endif

    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}
#endif

#ifdef USE_SPI_DEVICE_2

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

void initSpi2(void)
{
    // Specific to the STM32F103 / STM32F303 (AF5)
    // SPI2 Driver
    // PB12     25      SPI2_NSS
    // PB13     26      SPI2_SCK
    // PB14     27      SPI2_MISO
    // PB15     28      SPI2_MOSI

    SPI_InitTypeDef spi;

    // Enable SPI2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);


#ifdef STM32F303xC
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(SPI2_GPIO_PERIPHERAL, ENABLE);

    GPIO_PinAFConfig(SPI2_GPIO, SPI2_SCK_PIN_SOURCE, GPIO_AF_5);
    GPIO_PinAFConfig(SPI2_GPIO, SPI2_MISO_PIN_SOURCE, GPIO_AF_5);
    GPIO_PinAFConfig(SPI2_GPIO, SPI2_MOSI_PIN_SOURCE, GPIO_AF_5);
#ifdef SPI2_NSS_PIN_SOURCE
    GPIO_PinAFConfig(SPI2_GPIO, SPI2_NSS_PIN_SOURCE, GPIO_AF_5);
#endif

    GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN | SPI2_MISO_PIN | SPI2_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);

#ifdef SPI2_NSS_PIN
    GPIO_InitStructure.GPIO_Pin = SPI2_NSS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);
#endif

#endif

#ifdef STM32F10X
    gpio_config_t gpio;

    // MOSI + SCK as output
    gpio.mode = Mode_AF_PP;
    gpio.pin = SPI2_SCK_PIN | SPI2_MOSI_PIN;
    gpio.speed = Speed_50MHz;
    gpioInit(SPI2_GPIO, &gpio);
    // MISO as input
    gpio.pin = SPI2_MISO_PIN;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(SPI2_GPIO, &gpio);

#ifdef SPI2_NSS_PIN
    // NSS as gpio slave select
    gpio.pin = SPI2_NSS_PIN;
    gpio.mode = Mode_Out_PP;
    gpioInit(SPI2_GPIO, &gpio);
#endif
#endif

    // Init SPI2 hardware
    SPI_I2S_DeInit(SPI2);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;

#ifdef STM32F303xC
    // Configure for 8-bit reads.
    SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);
#endif
    SPI_Init(SPI2, &spi);
    SPI_Cmd(SPI2, ENABLE);

    // Drive NSS high to disable connected SPI device.
    GPIO_SetBits(SPI2_GPIO, SPI2_NSS_PIN);


}
#endif

bool spiInit(SPI_TypeDef *instance)
{
#if (!(defined(USE_SPI_DEVICE_1) && defined(USE_SPI_DEVICE_2)))
    UNUSED(instance);
#endif

#ifdef USE_SPI_DEVICE_1
    if (instance == SPI1) {
        initSpi1();
        return true;
    }
#endif
#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2) {
        initSpi2();
        return true;
    }
#endif
    return false;
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
#ifdef STM32F10X
    SPI_I2S_SendData(instance, data);
#endif
    spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

#ifdef STM32F303xC
    return ((uint8_t)SPI_ReceiveData8(instance));
#endif
#ifdef STM32F10X
    return ((uint8_t)SPI_I2S_ReceiveData(instance));
#endif
}

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
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
        SPI_SendData8(instance, b);
        //SPI_I2S_SendData16(instance, b);
#endif
#ifdef STM32F10X
        SPI_I2S_SendData(instance, b);
#endif
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
#ifdef STM32F303xC
        b = SPI_ReceiveData8(instance);
        //b = SPI_I2S_ReceiveData16(instance);
#endif
#ifdef STM32F10X
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

