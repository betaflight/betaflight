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

#include "drivers/bus.h"
#include "drivers/io_types.h"
#include "drivers/bus.h"
#include "drivers/rcc_types.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#if defined(STM32F4) || defined(STM32F3)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#elif defined(STM32F7)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#elif defined(STM32F1)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_50MHz)
#define SPI_IO_AF_MOSI_CFG      IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_50MHz)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_Mode_Out_PP,      GPIO_Speed_50MHz)
#endif

/*
  Flash M25p16 tolerates 20mhz, SPI_CLOCK_FAST should sit around 20 or less.
*/
typedef enum {
    SPI_CLOCK_INITIALIZATON = 256,
#if defined(STM32F4)
    SPI_CLOCK_SLOW          = 128, //00.65625 MHz
    SPI_CLOCK_STANDARD      = 8,   //10.50000 MHz
    SPI_CLOCK_FAST          = 4,   //21.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //42.00000 MHz
#elif defined(STM32F7)
    SPI_CLOCK_SLOW          = 256, //00.42188 MHz
    SPI_CLOCK_STANDARD      = 16,  //06.57500 MHz
    SPI_CLOCK_FAST          = 4,   //27.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //54.00000 MHz
#else
    SPI_CLOCK_SLOW          = 128, //00.56250 MHz
    SPI_CLOCK_STANDARD      = 4,   //09.00000 MHz
    SPI_CLOCK_FAST          = 2,   //18.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //18.00000 MHz
#endif
} SPIClockDivider_e;

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_4
} SPIDevice;

#if defined(STM32F1)
#define SPIDEV_COUNT 2
#elif defined(STM32F3) || defined(STM32F4)
#define SPIDEV_COUNT 3
#elif defined(STM32F7)
#define SPIDEV_COUNT 4
#else
#define SPIDEV_COUNT 4

#endif

// Macros to convert between CLI bus number and SPIDevice.
#define SPI_CFG_TO_DEV(x)   ((x) - 1)
#define SPI_DEV_TO_CFG(x)   ((x) + 1)

void spiPreInitCs(ioTag_t iotag);
void spiPreInitCsOutPU(ioTag_t iotag);

bool spiInit(SPIDevice device);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data);
bool spiIsBusBusy(SPI_TypeDef *instance);

bool spiTransfer(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len);

uint16_t spiGetErrorCounter(SPI_TypeDef *instance);
void spiResetErrorCounter(SPI_TypeDef *instance);
SPIDevice spiDeviceByInstance(SPI_TypeDef *instance);
SPI_TypeDef *spiInstanceByDevice(SPIDevice device);

bool spiBusTransfer(const busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int length);

bool spiBusWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data);
bool spiBusReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t spiBusReadRegister(const busDevice_t *bus, uint8_t reg);
void spiBusSetInstance(busDevice_t *bus, SPI_TypeDef *instance);

struct spiPinConfig_s;
void spiPinConfigure(const struct spiPinConfig_s *pConfig);
