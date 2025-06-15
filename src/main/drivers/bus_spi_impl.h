/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#if PLATFORM_TRAIT_RCC
#include "platform/rcc_types.h"
#endif

#define SPI_TIMEOUT_US  10000

#define BUS_SPI_FREE   0x0

typedef struct spiPinDef_s {
    ioTag_t pin;
#if SPI_TRAIT_AF_PIN //TODO: move to GPIO
    uint8_t af;
#endif
} spiPinDef_t;

typedef struct spiHardware_s {
    SPIDevice device;
    SPI_TypeDef *reg;
    spiPinDef_t sckPins[MAX_SPI_PIN_SEL];
    spiPinDef_t misoPins[MAX_SPI_PIN_SEL];
    spiPinDef_t mosiPins[MAX_SPI_PIN_SEL];
#if SPI_TRAIT_AF_PORT
    uint8_t af;
#endif

#if PLATFORM_TRAIT_RCC
    rccPeriphTag_t rcc;
#endif

#ifdef USE_DMA
    uint8_t dmaIrqHandler;
#endif
} spiHardware_t;

extern const spiHardware_t spiHardware[SPIDEV_COUNT];

typedef struct SPIDevice_s {
    SPI_TypeDef *dev;
    ioTag_t sck;
    ioTag_t miso;
    ioTag_t mosi;
#if SPI_TRAIT_AF_PIN
    uint8_t sckAF;
    uint8_t misoAF;
    uint8_t mosiAF;
#endif
#if SPI_TRAIT_AF_PORT
    uint8_t af;
#endif
#if SPI_TRAIT_HANDLE
    SPI_HandleTypeDef hspi;
#endif
#if PLATFORM_TRAIT_RCC
    rccPeriphTag_t rcc;
#endif
    volatile uint16_t errorCount;
    bool leadingEdge;
#ifdef USE_DMA
    uint8_t dmaIrqHandler;
#endif
} spiDevice_t;

extern spiDevice_t spiDevice[SPIDEV_COUNT];

void spiInitDevice(SPIDevice device);
void spiInternalInitStream(const extDevice_t *dev, bool preInit);
void spiInternalStartDMA(const extDevice_t *dev);
void spiInternalStopDMA (const extDevice_t *dev);
void spiInternalResetStream(dmaChannelDescriptor_t *descriptor);
void spiInternalResetDescriptors(busDevice_t *bus);
bool spiInternalReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len);
void spiSequenceStart(const extDevice_t *dev);
