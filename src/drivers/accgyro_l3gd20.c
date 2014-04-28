/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/maths.h"

#include "system_common.h"
#include "gpio_common.h"
#include "bus_spi.h"

#include "accgyro_common.h"
#include "accgyro_l3gd20.h"

extern int16_t debug[4];

#define READ_CMD               ((uint8_t)0x80)
#define MULTIPLEBYTE_CMD       ((uint8_t)0x40)
#define DUMMY_BYTE             ((uint8_t)0x00)

#define CTRL_REG1_ADDR         0x20
#define CTRL_REG4_ADDR         0x23
#define CTRL_REG5_ADDR         0x24
#define OUT_TEMP_ADDR          0x26
#define OUT_X_L_ADDR           0x28



#define MODE_ACTIVE                   ((uint8_t)0x08)

#define OUTPUT_DATARATE_1             ((uint8_t)0x00)
#define OUTPUT_DATARATE_2             ((uint8_t)0x40)
#define OUTPUT_DATARATE_3             ((uint8_t)0x80)
#define OUTPUT_DATARATE_4             ((uint8_t)0xC0)

#define AXES_ENABLE                   ((uint8_t)0x07)

#define BANDWIDTH_1                   ((uint8_t)0x00)
#define BANDWIDTH_2                   ((uint8_t)0x10)
#define BANDWIDTH_3                   ((uint8_t)0x20)
#define BANDWIDTH_4                   ((uint8_t)0x30)

#define FULLSCALE_250                 ((uint8_t)0x00)
#define FULLSCALE_500                 ((uint8_t)0x10)
#define FULLSCALE_2000                ((uint8_t)0x20)

#define BLOCK_DATA_UPDATE_CONTINUOUS  ((uint8_t)0x00)

#define BLE_MSB	                      ((uint8_t)0x40)

#define BOOT                          ((uint8_t)0x80)



static volatile uint16_t spi1ErrorCount = 0;
static volatile uint16_t spi2ErrorCount = 0;
static volatile uint16_t spi3ErrorCount = 0;

#define SPI1_GPIO             GPIOA
#define SPI1_SCK_PIN          GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE   GPIO_PinSource5
#define SPI1_SCK_CLK          RCC_AHBPeriph_GPIOA
#define SPI1_MISO_PIN         GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE  GPIO_PinSource6
#define SPI1_MISO_CLK         RCC_AHBPeriph_GPIOA
#define SPI1_MOSI_PIN         GPIO_Pin_7
#define SPI1_MOSI_PIN_SOURCE  GPIO_PinSource7
#define SPI1_MOSI_CLK         RCC_AHBPeriph_GPIOA

uint32_t spiTimeoutUserCallback(SPI_TypeDef *SPIx)
{
    if (SPIx == SPI1)
    {
        spi1ErrorCount++;
        return spi1ErrorCount;
    }
    else if (SPIx == SPI2)
    {
        spi2ErrorCount++;
        return spi2ErrorCount;
    }
    else
    {    spi3ErrorCount++;
         return spi3ErrorCount;
    }
}

static void l3gd20SpiInit(SPI_TypeDef *SPIx)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHBPeriphClockCmd(SPI1_SCK_CLK | SPI1_MISO_CLK | SPI1_MOSI_CLK, ENABLE);

    GPIO_PinAFConfig(SPI1_GPIO, SPI1_SCK_PIN_SOURCE,  GPIO_AF_5);
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MISO_PIN_SOURCE, GPIO_AF_5);
    GPIO_PinAFConfig(SPI1_GPIO, SPI1_MOSI_PIN_SOURCE, GPIO_AF_5);

    // Init pins
    GPIO_InitStructure.GPIO_Pin   = SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

    RCC_AHBPeriphClockCmd(L3GD20_CS_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = L3GD20_CS_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(L3GD20_CS_GPIO, &GPIO_InitStructure);

    GPIO_SetBits(L3GD20_CS_GPIO, L3GD20_CS_PIN);

    SPI_I2S_DeInit(SPI1);

    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  // 36/4 = 9 MHz SPI Clock
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;

    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_RxFIFOThresholdConfig(L3GD20_SPI, SPI_RxFIFOThreshold_QF);

    SPI_Cmd(SPI1, ENABLE);
}


static uint8_t spiTransfer(SPI_TypeDef *SPIx, uint8_t data)
{
    uint16_t spiTimeout;

    spiTimeout = 0x1000;
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
      if ((spiTimeout--) == 0) return spiTimeoutUserCallback(SPIx);

    SPI_SendData8(SPIx, data);

    spiTimeout = 0x1000;
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
      if ((spiTimeout--) == 0) return spiTimeoutUserCallback(SPIx);

    return((uint8_t)SPI_ReceiveData8(SPIx));
}

void l3gd20GyroInit(void)
{

    l3gd20SpiInit(L3GD20_SPI);

	GPIO_ResetBits(L3GD20_CS_GPIO, L3GD20_CS_PIN);

	spiTransfer(L3GD20_SPI, CTRL_REG5_ADDR);
	spiTransfer(L3GD20_SPI, BOOT);

	GPIO_SetBits(L3GD20_CS_GPIO, L3GD20_CS_PIN);

	delayMicroseconds(100);

	GPIO_ResetBits(L3GD20_CS_GPIO, L3GD20_CS_PIN);

    spiTransfer(L3GD20_SPI, CTRL_REG1_ADDR);

    spiTransfer(L3GD20_SPI, MODE_ACTIVE | OUTPUT_DATARATE_3 | AXES_ENABLE | BANDWIDTH_3);
    //spiTransfer(L3GD20_SPI, MODE_ACTIVE | OUTPUT_DATARATE_4 | AXES_ENABLE | BANDWIDTH_4);

    GPIO_SetBits(L3GD20_CS_GPIO, L3GD20_CS_PIN);

    delayMicroseconds(1);

    GPIO_ResetBits(L3GD20_CS_GPIO, L3GD20_CS_PIN);

    spiTransfer(L3GD20_SPI, CTRL_REG4_ADDR);
    spiTransfer(L3GD20_SPI, BLOCK_DATA_UPDATE_CONTINUOUS | BLE_MSB | FULLSCALE_500);

    GPIO_SetBits(L3GD20_CS_GPIO, L3GD20_CS_PIN);

    delay(100);
}

static void l3gd20GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];

    GPIO_ResetBits(L3GD20_CS_GPIO, L3GD20_CS_PIN);
    spiTransfer(L3GD20_SPI, OUT_X_L_ADDR | READ_CMD |MULTIPLEBYTE_CMD);

    uint8_t index;
    for (index = 0; index < sizeof(buf); index++) {
        buf[index] = spiTransfer(L3GD20_SPI, DUMMY_BYTE);
    }

    GPIO_SetBits(L3GD20_CS_GPIO, L3GD20_CS_PIN);

    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) / 8;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) / 8;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) / 8;

#if 1
    debug[0] = (int16_t)((buf[1] << 8) | buf[0]);
    debug[1] = (int16_t)((buf[3] << 8) | buf[2]);
    debug[2] = (int16_t)((buf[5] << 8) | buf[4]);
#endif
}

bool l3gd20Detect(gyro_t *gyro, uint16_t lpf)
{
    gyro->init = l3gd20GyroInit;
    gyro->read = l3gd20GyroRead;

    // 16.4 dps/lsb scalefactor
    gyro->scale = L3GD20_GYRO_SCALE_FACTOR;
    gyro->scale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;

    return true;  // blindly assume it's present, for now.
}
