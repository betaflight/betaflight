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
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"
#ifdef USE_DMA_SPI_DEVICE
#ifndef GYRO_READ_TIMEOUT
    #define GYRO_READ_TIMEOUT 20
#endif //GYRO_READ_TIMEOUT
#include "drivers/dma_spi.h"
#include "drivers/time.h"
#endif //USE_DMA_SPI_DEVICE

spiDevice_t spiDevice[SPIDEV_COUNT];

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

SPI_TypeDef *spiInstanceByDevice(SPIDevice device)
{
    if (device >= SPIDEV_COUNT) {
        return NULL;
    }

    return spiDevice[device].dev;
}

bool spiInit(SPIDevice device)
{
    switch (device) {
    case SPIINVALID:
        return false;
    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3) && !defined(STM32F1)
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
    if (device == SPIINVALID) {
        return -1;
    }
    spiDevice[device].errorCount++;
    return spiDevice[device].errorCount;
}


bool spiBusTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length)
{

    #ifdef USE_DMA_SPI_DEVICE
        (void)(bus);
        uint32_t timeoutCheck = millis();
        memcpy(dmaTxBuffer, (uint8_t *)txData, length);
        dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, length, 1);
        while(dmaSpiReadStatus != DMA_SPI_READ_DONE)
        {
            if(millis() - timeoutCheck > GYRO_READ_TIMEOUT)
            {
                //GYRO_READ_TIMEOUT ms max, read failed, cleanup spi and return 0
                IOHi(bus->busdev_u.spi.csnPin);
                dmaSpicleanupspi();
                return false;
            }
        }
        memcpy((uint8_t *)rxData, dmaRxBuffer, length);
    #else
        IOLo(bus->busdev_u.spi.csnPin);
        spiTransfer(bus->busdev_u.spi.instance, txData, rxData, length);
        IOHi(bus->busdev_u.spi.csnPin);
    #endif

    return true;
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return 0;
    }
    return spiDevice[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID) {
        spiDevice[device].errorCount = 0;
    }
}

bool spiBusWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    #ifdef USE_DMA_SPI_DEVICE
        (void)(bus);
        uint32_t timeoutCheck = millis();
        dmaTxBuffer[0] = reg;
        dmaTxBuffer[1] = data;
        dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, 2, 1);
        while(dmaSpiReadStatus != DMA_SPI_READ_DONE)
        {
            if(millis() - timeoutCheck > GYRO_READ_TIMEOUT)
            {
                //GYRO_READ_TIMEOUT ms max, read failed, cleanup spi and return 0
                IOHi(bus->busdev_u.spi.csnPin);
                dmaSpicleanupspi();
                return false;
            }
        }
    #else
        IOLo(bus->busdev_u.spi.csnPin);
        spiTransferByte(bus->busdev_u.spi.instance, reg);
        spiTransferByte(bus->busdev_u.spi.instance, data);
        IOHi(bus->busdev_u.spi.csnPin);
    #endif

    return true;
}

bool spiBusReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    #ifdef USE_DMA_SPI_DEVICE
        (void)(bus);
        uint32_t timeoutCheck = millis();
        dmaTxBuffer[0] = reg | 0x80;
        dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, length+1, 1);
        while(dmaSpiReadStatus != DMA_SPI_READ_DONE)
        {
            if(millis() - timeoutCheck > GYRO_READ_TIMEOUT)
            {
                //GYRO_READ_TIMEOUT ms max, read failed, cleanup spi and return 0
                IOHi(bus->busdev_u.spi.csnPin);
                dmaSpicleanupspi();
                return false;
            }
        }
        memcpy(data, dmaRxBuffer+1, length);
    #else
        IOLo(bus->busdev_u.spi.csnPin);
        spiTransferByte(bus->busdev_u.spi.instance, reg | 0x80); // read transaction
        spiTransfer(bus->busdev_u.spi.instance, NULL, data, length);
        IOHi(bus->busdev_u.spi.csnPin);
    #endif

    return true;
}

uint8_t spiBusReadRegister(const busDevice_t *bus, uint8_t reg)
{
    #ifdef USE_DMA_SPI_DEVICE
        (void)(bus);
        uint32_t timeoutCheck = millis();
        dmaTxBuffer[0] = reg | 0x80;
        dmaSpiTransmitReceive(dmaTxBuffer, dmaRxBuffer, 2, 1);
        while(dmaSpiReadStatus != DMA_SPI_READ_DONE)
        {
            if(millis() - timeoutCheck > GYRO_READ_TIMEOUT)
            {
                //GYRO_READ_TIMEOUT ms max, read failed, cleanup spi and return 0
                IOHi(bus->busdev_u.spi.csnPin);
                dmaSpicleanupspi();
                return 0;
            }
        }
        return dmaRxBuffer[1];
    #else
        uint8_t data;
        IOLo(bus->busdev_u.spi.csnPin);
        spiTransferByte(bus->busdev_u.spi.instance, reg | 0x80); // read transaction
        spiTransfer(bus->busdev_u.spi.instance, NULL, &data, 1);
        IOHi(bus->busdev_u.spi.csnPin);
        return data;
    #endif
}

void spiBusSetInstance(busDevice_t *bus, SPI_TypeDef *instance)
{
    bus->bustype = BUSTYPE_SPI;
    bus->busdev_u.spi.instance = instance;
}
#endif
