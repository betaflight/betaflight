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

#include "drivers/bus.h"
#include "drivers/io_types.h"
#include "drivers/bus.h"
#include "drivers/rcc_types.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#if defined(STM32F4)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define SPI_IO_AF_SDI_CFG      IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#elif defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN)
#define SPI_IO_AF_SDI_CFG      IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#elif defined(AT32F4)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_NONE)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL ,  GPIO_PULL_UP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL ,  GPIO_PULL_DOWN)
#define SPI_IO_AF_SDI_CFG      IO_CONFIG(GPIO_MODE_MUX, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL ,  GPIO_PULL_UP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL , GPIO_PULL_NONE)
#endif

// De facto standard mode
// See https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
//
// Mode CPOL CPHA
//  0    0    0
//  1    0    1
//  2    1    0
//  3    1    1
typedef enum {
    SPI_MODE0_POL_LOW_EDGE_1ST = 0,
    SPI_MODE1_POL_LOW_EDGE_2ND,
    SPI_MODE2_POL_HIGH_EDGE_1ST,
    SPI_MODE3_POL_HIGH_EDGE_2ND
} SPIMode_e;

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_4,
    SPIDEV_5,
    SPIDEV_6
} SPIDevice;

#if defined(STM32F4)
#define SPIDEV_COUNT 3
#elif defined(STM32F7)
#define SPIDEV_COUNT 4
#elif defined(STM32H7)
#define SPIDEV_COUNT 6
#else
#define SPIDEV_COUNT 4
#endif

// Macros to convert between CLI bus number and SPIDevice.
#define SPI_CFG_TO_DEV(x)   ((x) - 1)
#define SPI_DEV_TO_CFG(x)   ((x) + 1)

// Work around different check routines in the libraries for different MCU types
#if defined(STM32H7)
#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXWNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->RXDR)
#else
#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->DR)
#endif

void spiPreinit(void);
void spiPreinitRegister(ioTag_t iotag, uint8_t iocfg, uint8_t init);
void spiPreinitByIO(IO_t io);
void spiPreinitByTag(ioTag_t tag);

bool spiInit(SPIDevice device);

// Called after all devices are initialised to enable SPI DMA where streams are available.
void spiInitBusDMA();


SPIDevice spiDeviceByInstance(SPI_TypeDef *instance);
SPI_TypeDef *spiInstanceByDevice(SPIDevice device);

// BusDevice API

// Mark a device's associated bus as being SPI
bool spiSetBusInstance(extDevice_t *dev, uint32_t device);
// Determine the divisor to use for a given bus frequency
uint16_t spiCalculateDivider(uint32_t freq);
// Return the SPI clock based on the given divisor
uint32_t spiCalculateClock(uint16_t spiClkDivisor);
// Set the clock divisor to be used for accesses by the given device
void spiSetClkDivisor(const extDevice_t *dev, uint16_t divider);
// Set the clock phase/polarity to be used for accesses by the given device
void spiSetClkPhasePolarity(const extDevice_t *dev, bool leadingEdge);
// Enable/disable DMA on a specific device. Enabled by default.
void spiDmaEnable(const extDevice_t *dev, bool enable);

// DMA transfer setup and start
void spiSequence(const extDevice_t *dev, busSegment_t *segments);
// Wait for DMA completion
void spiWait(const extDevice_t *dev);
// Negate CS if held asserted after a transfer
void spiRelease(const extDevice_t *dev);
// Return true if DMA engine is busy
bool spiIsBusy(const extDevice_t *dev);

// Link two segment lists
void spiLinkSegments(const extDevice_t *dev, busSegment_t *firstSegment, busSegment_t *secondSegment);

/*
 * Routine naming convention is:
 *  spi[Read][Write][Reg][Msk][Buf][RB]
 *
 *      Read: Perform a read, returning the value read unless 'Buf' is specified
 *      Write Perform a write
 *      ReadWrite: Perform both a read and write, returning the value read unless 'Buf' is specified
 *      Reg: Register number 'reg' is written prior to the read being performed
 *      Msk: Register number is logically ORed with 0x80 as some devices indicate a read by accessing a register with bit 7 set
 *      Buf: Pass data of given length by reference
 *      RB:  Return false immediately if the bus is busy, otherwise complete the access and return true
 */
uint8_t spiReadReg(const extDevice_t *dev, uint8_t reg);
uint8_t spiReadRegMsk(const extDevice_t *dev, uint8_t reg);
void spiReadRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool spiReadRegBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool spiReadRegMskBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);

void spiWrite(const extDevice_t *dev, uint8_t data);
void spiWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool spiWriteRegRB(const extDevice_t *dev, uint8_t reg, uint8_t data);

uint8_t spiReadWrite(const extDevice_t *dev, uint8_t data);

void spiWriteRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint32_t length);
uint8_t spiReadWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data);
void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len);
bool spiReadWriteBufRB(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int length);

//
// Config
//

struct spiPinConfig_s;
void spiPinConfigure(const struct spiPinConfig_s *pConfig);
bool spiUseDMA(const extDevice_t *dev);
bool spiUseSDO_DMA(const extDevice_t *dev);
void spiBusDeviceRegister(const extDevice_t *dev);
uint8_t spiGetRegisteredDeviceCount(void);
uint8_t spiGetExtDeviceCount(const extDevice_t *dev);

