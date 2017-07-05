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
#include <string.h>

#include <platform.h>

#ifdef USE_SPI

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

spiDevice_t spiDevice[SPIDEV_COUNT];

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

#define SPI_DEFAULT_TIMEOUT 10

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
#ifdef USE_SPI_DEVICE_1
    if (instance == SPI1)
        return SPIDEV_1;
#endif

#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2)
        return SPIDEV_2;
#endif

#ifdef USE_SPI_DEVICE_3
    if (instance == SPI3)
        return SPIDEV_3;
#endif

#ifdef USE_SPI_DEVICE_4
    if (instance == SPI4)
        return SPIDEV_4;
#endif

    return SPIINVALID;
}

SPI_HandleTypeDef* spiHandleByInstance(SPI_TypeDef *instance)
{
    return &spiDevice[spiDeviceByInstance(instance)].hspi;
}

DMA_HandleTypeDef* dmaHandleByInstance(SPI_TypeDef *instance)
{
    return &spiDevice[spiDeviceByInstance(instance)].hdma;
}

void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_1].hspi);
}

void SPI2_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_2].hspi);
}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_3].hspi);
}

void SPI4_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spiDevice[SPIDEV_4].hspi);
}


void spiInitDevice(SPIDevice device)
{
    spiDevice_t *spi = &(spiDevice[device]);

#ifdef SDCARD_SPI_INSTANCE
    if (spi->dev == SDCARD_SPI_INSTANCE) {
        spi->leadingEdge = true;
    }
#endif
#ifdef RX_SPI_INSTANCE
    if (spi->dev == RX_SPI_INSTANCE) {
        spi->leadingEdge = true;
    }
#endif
#ifdef MPU6500_SPI_INSTANCE
    if (spi->dev == MPU6500_SPI_INSTANCE) {
        spi->leadingEdge = true;
    }
#endif

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));

#if defined(STM32F7)
    if (spi->leadingEdge == true)
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->sckAF);
    else
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->sckAF);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->misoAF);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->mosiAF);
#endif
#if defined(STM32F3) || defined(STM32F4)
    if (spi->leadingEdge == true)
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_LOW, spi->af);
    else
        IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG_HIGH, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);
#endif
#if defined(STM32F10X)
    IOConfigGPIO(IOGetByTag(spi->sck), SPI_IO_AF_SCK_CFG);
    IOConfigGPIO(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG);
    IOConfigGPIO(IOGetByTag(spi->mosi), SPI_IO_AF_MOSI_CFG);
#endif
    spiDevice[device].hspi.Instance = spi->dev;
    // Init SPI hardware
    HAL_SPI_DeInit(&spiDevice[device].hspi);

    spiDevice[device].hspi.Init.Mode = SPI_MODE_MASTER;
    spiDevice[device].hspi.Init.Direction = SPI_DIRECTION_2LINES;
    spiDevice[device].hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    spiDevice[device].hspi.Init.NSS = SPI_NSS_SOFT;
    spiDevice[device].hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spiDevice[device].hspi.Init.CRCPolynomial = 7;
    spiDevice[device].hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    spiDevice[device].hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spiDevice[device].hspi.Init.TIMode = SPI_TIMODE_DISABLED;

    if (spi->leadingEdge) {
        spiDevice[device].hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
        spiDevice[device].hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    }
    else {
        spiDevice[device].hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
        spiDevice[device].hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    }

    if (HAL_SPI_Init(&spiDevice[device].hspi) == HAL_OK)
    {
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
    spiDevice[device].errorCount++;
    return spiDevice[device].errorCount;
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (spiDevice[device].hspi.State == HAL_SPI_STATE_BUSY)
        return true;
    else
        return false;
}

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
{
    SPIDevice device = spiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    if (!out) // Tx only
    {
        status = HAL_SPI_Transmit(&spiDevice[device].hspi, (uint8_t *)in, len, SPI_DEFAULT_TIMEOUT);
    }
    else if (!in) // Rx only
    {
        status = HAL_SPI_Receive(&spiDevice[device].hspi, out, len, SPI_DEFAULT_TIMEOUT);
    }
    else // Tx and Rx
    {
        status = HAL_SPI_TransmitReceive(&spiDevice[device].hspi, in, out, len, SPI_DEFAULT_TIMEOUT);
    }

    if ( status != HAL_OK)
        spiTimeoutUserCallback(instance);

    return true;
}

static bool spiBusReadBuffer(const busDevice_t *bus, uint8_t *out, int len)
{
    const HAL_StatusTypeDef status = HAL_SPI_Receive(bus->spi.handle, out, len, SPI_DEFAULT_TIMEOUT);
    if (status != HAL_OK) {
        spiTimeoutUserCallback(bus->spi.instance);
    }
    return true;
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t in)
{
    spiTransfer(instance, &in, &in, 1);
    return in;
}

// return uint8_t value or -1 when failure
static uint8_t spiBusTransferByte(const busDevice_t *bus, uint8_t in)
{
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(bus->spi.handle, &in, &in, 1, SPI_DEFAULT_TIMEOUT);
    if (status != HAL_OK) {
        spiTimeoutUserCallback(bus->spi.instance);
    }
    return in;
}

bool spiBusTransfer(const busDevice_t *bus, uint8_t *rxData, const uint8_t *txData, int len)
{
    IOLo(bus->spi.csnPin);
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(bus->spi.handle, txData, rxData, len, SPI_DEFAULT_TIMEOUT);
    IOHi(bus->spi.csnPin);
    if (status != HAL_OK) {
        spiTimeoutUserCallback(bus->spi.instance);
    }
    return true;
}

void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (HAL_SPI_DeInit(&spiDevice[device].hspi) == HAL_OK)
    {
    }

    spiDevice[device].hspi.Init.BaudRatePrescaler = (uint8_t []) {
        0, 0,
        SPI_BAUDRATEPRESCALER_2, SPI_BAUDRATEPRESCALER_4,
        SPI_BAUDRATEPRESCALER_8, SPI_BAUDRATEPRESCALER_16,
        SPI_BAUDRATEPRESCALER_32, SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256
    }[ffs(divisor | 0x100)];

    if (HAL_SPI_Init(&spiDevice[device].hspi) == HAL_OK)
    {
    }
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID)
        return 0;
    return spiDevice[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID)
        spiDevice[device].errorCount = 0;
}

bool spiWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    IOLo(bus->spi.csnPin);
    spiBusTransferByte(bus, reg);
    spiBusTransferByte(bus, data);
    IOHi(bus->spi.csnPin);

    return true;
}

bool spiReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t length, uint8_t *data)
{
    IOLo(bus->spi.csnPin);
    spiBusTransferByte(bus, reg | 0x80); // read transaction
    spiBusReadBuffer(bus, data, length);
    IOHi(bus->spi.csnPin);

    return true;
}

uint8_t spiReadRegister(const busDevice_t *bus, uint8_t reg)
{
    uint8_t data;
    IOLo(bus->spi.csnPin);
    spiBusTransferByte(bus, reg | 0x80); // read transaction
    spiBusReadBuffer(bus, &data, 1);
    IOHi(bus->spi.csnPin);

    return data;
}

void spiBusSetInstance(busDevice_t *bus, SPI_TypeDef *instance)
{
    bus->spi.instance = instance;
    bus->spi.handle = spiHandleByInstance(instance);
}

void dmaSPIIRQHandler(dmaChannelDescriptor_t* descriptor)
{
    SPIDevice device = descriptor->userParam;
    if (device != SPIINVALID)
        HAL_DMA_IRQHandler(&spiDevice[device].hdma);
}


DMA_HandleTypeDef* spiSetDMATransmit(DMA_Stream_TypeDef *Stream, uint32_t Channel, SPI_TypeDef *Instance, uint8_t *pData, uint16_t Size)
{
    SPIDevice device = spiDeviceByInstance(Instance);

    spiDevice[device].hdma.Instance = Stream;
    spiDevice[device].hdma.Init.Channel = Channel;
    spiDevice[device].hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    spiDevice[device].hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    spiDevice[device].hdma.Init.MemInc = DMA_MINC_ENABLE;
    spiDevice[device].hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    spiDevice[device].hdma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    spiDevice[device].hdma.Init.Mode = DMA_NORMAL;
    spiDevice[device].hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    spiDevice[device].hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    spiDevice[device].hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;
    spiDevice[device].hdma.Init.MemBurst = DMA_MBURST_SINGLE;
    spiDevice[device].hdma.Init.Priority = DMA_PRIORITY_LOW;

    HAL_DMA_DeInit(&spiDevice[device].hdma);
    HAL_DMA_Init(&spiDevice[device].hdma);

    __HAL_DMA_ENABLE(&spiDevice[device].hdma);
    __HAL_SPI_ENABLE(&spiDevice[device].hspi);

    /* Associate the initialized DMA handle to the spi handle */
    __HAL_LINKDMA(&spiDevice[device].hspi, hdmatx, spiDevice[device].hdma);

    // DMA TX Interrupt
    dmaSetHandler(spiDevice[device].dmaIrqHandler, dmaSPIIRQHandler, NVIC_BUILD_PRIORITY(3, 0), (uint32_t)device);

    //HAL_CLEANCACHE(pData,Size);
    // And Transmit
    HAL_SPI_Transmit_DMA(&spiDevice[device].hspi, pData, Size);

    return &spiDevice[device].hdma;
}
#endif
