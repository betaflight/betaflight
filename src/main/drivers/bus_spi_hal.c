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

#include "drivers/bus_spi.h"
#include "dma.h"
#include "drivers/io.h"
#include "io_impl.h"
#include "drivers/nvic.h"
#include "rcc.h"

#ifndef SPI1_SCK_PIN
#define SPI1_NSS_PIN    PA4
#define SPI1_SCK_PIN    PA5
#define SPI1_MISO_PIN   PA6
#define SPI1_MOSI_PIN   PA7
#endif

#ifndef SPI2_SCK_PIN
#define SPI2_NSS_PIN    PB12
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_NSS_PIN    PA15
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif

#ifndef SPI4_SCK_PIN
#define SPI4_NSS_PIN    PA15
#define SPI4_SCK_PIN    PB3
#define SPI4_MISO_PIN   PB4
#define SPI4_MOSI_PIN   PB5
#endif

#ifndef SPI1_NSS_PIN
#define SPI1_NSS_PIN NONE
#endif
#ifndef SPI2_NSS_PIN
#define SPI2_NSS_PIN NONE
#endif
#ifndef SPI3_NSS_PIN
#define SPI3_NSS_PIN NONE
#endif
#ifndef SPI4_NSS_PIN
#define SPI4_NSS_PIN NONE
#endif


static spiDevice_t spiHardwareMap[] = {
    { .dev = SPI1, .nss = IO_TAG(SPI1_NSS_PIN), .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .rcc = RCC_APB2(SPI1), .af = GPIO_AF5_SPI1, .leadingEdge = false, .dmaIrqHandler = DMA2_ST3_HANDLER },
    { .dev = SPI2, .nss = IO_TAG(SPI2_NSS_PIN), .sck = IO_TAG(SPI2_SCK_PIN), .miso = IO_TAG(SPI2_MISO_PIN), .mosi = IO_TAG(SPI2_MOSI_PIN), .rcc = RCC_APB1(SPI2), .af = GPIO_AF5_SPI2, .leadingEdge = false, .dmaIrqHandler = DMA1_ST4_HANDLER },
    { .dev = SPI3, .nss = IO_TAG(SPI3_NSS_PIN), .sck = IO_TAG(SPI3_SCK_PIN), .miso = IO_TAG(SPI3_MISO_PIN), .mosi = IO_TAG(SPI3_MOSI_PIN), .rcc = RCC_APB1(SPI3), .af = GPIO_AF6_SPI3, .leadingEdge = false, .dmaIrqHandler = DMA1_ST7_HANDLER },
    { .dev = SPI4, .nss = IO_TAG(SPI4_NSS_PIN), .sck = IO_TAG(SPI4_SCK_PIN), .miso = IO_TAG(SPI4_MISO_PIN), .mosi = IO_TAG(SPI4_MOSI_PIN), .rcc = RCC_APB2(SPI4), .af = GPIO_AF5_SPI4, .leadingEdge = false, .dmaIrqHandler = DMA2_ST1_HANDLER }
};

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
    if (instance == SPI1)
        return SPIDEV_1;

    if (instance == SPI2)
        return SPIDEV_2;

    if (instance == SPI3)
        return SPIDEV_3;

    if (instance == SPI4)
        return SPIDEV_4;

    return SPIINVALID;
}

SPI_HandleTypeDef* spiHandleByInstance(SPI_TypeDef *instance)
{
    return &spiHardwareMap[spiDeviceByInstance(instance)].hspi;
}

DMA_HandleTypeDef* dmaHandleByInstance(SPI_TypeDef *instance)
{
    return &spiHardwareMap[spiDeviceByInstance(instance)].hdma;
}

void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiHardwareMap[SPIDEV_1].hspi);
}

void SPI2_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiHardwareMap[SPIDEV_2].hspi);
}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiHardwareMap[SPIDEV_3].hspi);
}

void SPI4_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiHardwareMap[SPIDEV_4].hspi);
}


void spiInitDevice(SPIDevice device)
{
    spiDevice_t *spi = &(spiHardwareMap[device]);

#ifdef SDCARD_SPI_INSTANCE
    if (spi->dev == SDCARD_SPI_INSTANCE) {
        spi->leadingEdge = true;
    }
#endif

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI, RESOURCE_SPI_SCK,  device + 1);
    IOInit(IOGetByTag(spi->miso), OWNER_SPI, RESOURCE_SPI_MISO, device + 1);
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI, RESOURCE_SPI_MOSI, device + 1);

#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7)
    if (spi->leadingEdge == true) {
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->af);
    } else {
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->af);
    }
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);

    if (spi->nss) {
        IOInit(IOGetByTag(spi->nss),  OWNER_SPI, RESOURCE_SPI_CS,  device + 1);
        IOConfigGPIO(IOGetByTag(spi->nss), SPI_IO_CS_CFG);
    }
#endif
#if defined(STM32F10X)
    IOConfigGPIO(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG);
    IOConfigGPIO(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG);
    IOConfigGPIO(IOGetByTag(spi->mosi), SPI_IO_AF_MOSI_CFG);

    if (spi->nss) {
        IOInit(IOGetByTag(spi->nss), OWNER_SPI_CS, RESOURCE_INDEX(device));
        IOConfigGPIO(IOGetByTag(spi->nss), SPI_IO_CS_CFG);
    }
#endif
    spiHardwareMap[device].hspi.Instance = spi->dev;
    // Init SPI hardware
    HAL_SPI_DeInit(&spiHardwareMap[device].hspi);

    spiHardwareMap[device].hspi.Init.Mode = SPI_MODE_MASTER;
    spiHardwareMap[device].hspi.Init.Direction = SPI_DIRECTION_2LINES;
    spiHardwareMap[device].hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    spiHardwareMap[device].hspi.Init.NSS = SPI_NSS_SOFT;
    spiHardwareMap[device].hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spiHardwareMap[device].hspi.Init.CRCPolynomial = 7;
    spiHardwareMap[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    spiHardwareMap[device].hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spiHardwareMap[device].hspi.Init.TIMode = SPI_TIMODE_DISABLED;

    if (spi->leadingEdge) {
        spiHardwareMap[device].hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
        spiHardwareMap[device].hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    } else {
        spiHardwareMap[device].hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
        spiHardwareMap[device].hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    }

    if (HAL_SPI_Init(&spiHardwareMap[device].hspi) == HAL_OK) {
        if (spi->nss) {
            IOHi(IOGetByTag(spi->nss));
        }
    }
}

bool spiInit(SPIDevice device)
{
    switch (device)
    {
    case SPIINVALID:
        return false;
    case SPIDEV_1:
#if defined(USE_SPI_DEVICE_1)
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_2:
#if defined(USE_SPI_DEVICE_2)
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3)
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_4:
#if defined(USE_SPI_DEVICE_4)
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    }
    return false;
}

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID)
        return -1;
    spiHardwareMap[device].errorCount++;
    return spiHardwareMap[device].errorCount;
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t in)
{
    spiTransfer(instance, &in, &in, 1);
    return in;
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (spiHardwareMap[device].hspi.State == HAL_SPI_STATE_BUSY)
        return true;
    else
        return false;
}

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
{
    SPIDevice device = spiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

#define SPI_DEFAULT_TIMEOUT 10

    if (!out) // Tx only
    {
        status = HAL_SPI_Transmit(&spiHardwareMap[device].hspi, (uint8_t *)in, len, SPI_DEFAULT_TIMEOUT);
    }
    else if (!in) // Rx only
    {
        status = HAL_SPI_Receive(&spiHardwareMap[device].hspi, out, len, SPI_DEFAULT_TIMEOUT);
    }
    else // Tx and Rx
    {
        status = HAL_SPI_TransmitReceive(&spiHardwareMap[device].hspi, (uint8_t *)in, out, len, SPI_DEFAULT_TIMEOUT);
    }

    if ( status != HAL_OK)
        spiTimeoutUserCallback(instance);

    return true;
}


void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (HAL_SPI_DeInit(&spiHardwareMap[device].hspi) == HAL_OK)
    {
    }

    switch (divisor) {
    case 2:
        spiHardwareMap[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        break;

    case 4:
        spiHardwareMap[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
        break;

    case 8:
        spiHardwareMap[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        break;

    case 16:
        spiHardwareMap[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        break;

    case 32:
        spiHardwareMap[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
        break;

    case 64:
        spiHardwareMap[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        break;

    case 128:
        spiHardwareMap[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
        break;

    case 256:
        spiHardwareMap[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
        break;
    }

    if (HAL_SPI_Init(&spiHardwareMap[device].hspi) == HAL_OK)
    {
    }
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID)
        return 0;
    return spiHardwareMap[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID)
        spiHardwareMap[device].errorCount = 0;
}

void dmaSPIIRQHandler(dmaChannelDescriptor_t* descriptor)
{
    SPIDevice device = descriptor->userParam;
    if (device != SPIINVALID)
        HAL_DMA_IRQHandler(&spiHardwareMap[device].hdma);
}


DMA_HandleTypeDef* spiSetDMATransmit(DMA_Stream_TypeDef *Stream, uint32_t Channel, SPI_TypeDef *Instance, uint8_t *pData, uint16_t Size)
{
    SPIDevice device = spiDeviceByInstance(Instance);

    spiHardwareMap[device].hdma.Instance = Stream;
    spiHardwareMap[device].hdma.Init.Channel = Channel;
    spiHardwareMap[device].hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    spiHardwareMap[device].hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    spiHardwareMap[device].hdma.Init.MemInc = DMA_MINC_ENABLE;
    spiHardwareMap[device].hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    spiHardwareMap[device].hdma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    spiHardwareMap[device].hdma.Init.Mode = DMA_NORMAL;
    spiHardwareMap[device].hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    spiHardwareMap[device].hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    spiHardwareMap[device].hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;
    spiHardwareMap[device].hdma.Init.MemBurst = DMA_MBURST_SINGLE;
    spiHardwareMap[device].hdma.Init.Priority = DMA_PRIORITY_LOW;

    HAL_DMA_DeInit(&spiHardwareMap[device].hdma);
    HAL_DMA_Init(&spiHardwareMap[device].hdma);

    __HAL_DMA_ENABLE(&spiHardwareMap[device].hdma);
    __HAL_SPI_ENABLE(&spiHardwareMap[device].hspi);

    /* Associate the initialized DMA handle to the spi handle */
    __HAL_LINKDMA(&spiHardwareMap[device].hspi, hdmatx, spiHardwareMap[device].hdma);

    // DMA TX Interrupt
    dmaSetHandler(spiHardwareMap[device].dmaIrqHandler, dmaSPIIRQHandler, NVIC_BUILD_PRIORITY(3, 0), (uint32_t)device);

    //HAL_CLEANCACHE(pData,Size);
    // And Transmit
    HAL_SPI_Transmit_DMA(&spiHardwareMap[device].hspi, pData, Size);

    return &spiHardwareMap[device].hdma;
}
