#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "dma_spi.h"
#include "sensors/gyro.h"
#include "drivers/accgyro/accgyro_mpu.h"


#ifdef USE_DMA_SPI_DEVICE

volatile dma_spi_read_status_t dmaSpiReadStatus = DMA_SPI_READ_UNKOWN;

//must be static to avoid overflow/corruption by DMA
uint8_t dmaTxBuffer[58];
uint8_t dmaRxBuffer[58];

static inline void gpio_write_pin(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint32_t pinState)
{
    if (pinState != 0)
    {
        GPIOx->BSRRL = (uint32_t)GPIO_Pin;
    }
    else
    {
        GPIOx->BSRRH = (uint32_t)GPIO_Pin;
    }
}

static inline void dmaSpiCsLo(void)
{
    gpio_write_pin(DMA_SPI_NSS_PORT, DMA_SPI_NSS_PIN, 0);
}

static inline void dmaSpiCsHi(void)
{
    gpio_write_pin(DMA_SPI_NSS_PORT, DMA_SPI_NSS_PIN, 1);
}

void dmaSpicleanupspi(void)
{
    //clear DMA flags
    DMA_ClearFlag(DMA_SPI_TX_DMA_STREAM, DMA_SPI_TX_DMA_FLAG_ALL);
    DMA_ClearFlag(DMA_SPI_RX_DMA_STREAM, DMA_SPI_RX_DMA_FLAG_ALL);

    //disable DMAs
    DMA_Cmd(DMA_SPI_TX_DMA_STREAM,DISABLE);
    DMA_Cmd(DMA_SPI_RX_DMA_STREAM,DISABLE);  

    //disable SPI DMA requests
    SPI_I2S_DMACmd(DMA_SPI_SPI, SPI_I2S_DMAReq_Tx, DISABLE);
    SPI_I2S_DMACmd(DMA_SPI_SPI, SPI_I2S_DMAReq_Rx, DISABLE);

    // Reset SPI (clears TXFIFO).

    //disable SPI
    SPI_Cmd(DMA_SPI_SPI, DISABLE);
}

void DMA_SPI_RX_DMA_HANDLER(void)
{
    dmaSpiCsHi();
    dmaSpicleanupspi();
    if(dmaSpiReadStatus != DMA_SPI_BLOCKING_READ_IN_PROGRESS)
    {
        dmaSpiReadStatus = DMA_SPI_READ_DONE;
        gyroDmaSpiFinishRead();
    }
    else
    {
        dmaSpiReadStatus = DMA_SPI_READ_DONE;
    }
    DMA_ClearITPendingBit(DMA_SPI_RX_DMA_STREAM, DMA_SPI_RX_DMA_FLAG_TC);         
}

void dmaSpiInit(void)
{
    GPIO_InitTypeDef gpioInitStruct;
    SPI_InitTypeDef spiInitStruct;
    DMA_InitTypeDef dmaInitStruct;
    NVIC_InitTypeDef nvicInitStruct;

    //reset SPI periphreal
    DMA_SPI_PER |= DMA_SPI_RST_MSK;
    DMA_SPI_PER &= ~DMA_SPI_RST_MSK;

    //config pins
    gpioInitStruct.GPIO_Pin = DMA_SPI_NSS_PIN;
    gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInitStruct.GPIO_OType = GPIO_OType_PP;
    gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(DMA_SPI_NSS_PORT, &gpioInitStruct);

    //set default CS state (high)
    GPIO_SetBits(DMA_SPI_NSS_PORT, DMA_SPI_NSS_PIN);

    gpioInitStruct.GPIO_Mode = GPIO_Mode_AF; 
    gpioInitStruct.GPIO_Pin = DMA_SPI_SCK_PIN;
    GPIO_Init(DMA_SPI_SCK_PORT, &gpioInitStruct);

    gpioInitStruct.GPIO_Pin = DMA_SPI_MISO_PIN;
    GPIO_Init(DMA_SPI_MISO_PORT, &gpioInitStruct);

    gpioInitStruct.GPIO_Pin = DMA_SPI_MOSI_PIN;
    GPIO_Init(DMA_SPI_MOSI_PORT, &gpioInitStruct);

    //set AF map
    GPIO_PinAFConfig(DMA_SPI_SCK_PORT,  DMA_SPI_SCK_PIN_SRC,  DMA_SPI_SCK_AF);
    GPIO_PinAFConfig(DMA_SPI_MISO_PORT, DMA_SPI_MISO_PIN_SRC, DMA_SPI_MISO_AF);
    GPIO_PinAFConfig(DMA_SPI_MOSI_PORT, DMA_SPI_MOSI_PIN_SRC, DMA_SPI_MOSI_AF);

    //config SPI
    SPI_StructInit(&spiInitStruct);
    spiInitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInitStruct.SPI_Mode = SPI_Mode_Master;
    spiInitStruct.SPI_DataSize = SPI_DataSize_8b;
    spiInitStruct.SPI_CPOL = DMA_SPI_CPOL;
    spiInitStruct.SPI_CPHA = DMA_SPI_CPHA;
    spiInitStruct.SPI_NSS = SPI_NSS_Soft;
    spiInitStruct.SPI_BaudRatePrescaler = DMA_SPI_BAUDRATE;
    spiInitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(DMA_SPI_SPI, &spiInitStruct);

    //set DMA to default state
    DMA_DeInit(DMA_SPI_TX_DMA_STREAM);
    DMA_DeInit(DMA_SPI_RX_DMA_STREAM);

    DMA_StructInit(&dmaInitStruct);
    dmaInitStruct.DMA_Channel = DMA_SPI_TX_DMA_CHANNEL;
    dmaInitStruct.DMA_Mode = DMA_Mode_Normal;
    dmaInitStruct.DMA_Priority = DMA_Priority_VeryHigh;
    dmaInitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;

    dmaInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStruct.DMA_PeripheralBaseAddr = (uint32_t)&DMA_SPI_SPI->DR;

    dmaInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitStruct.DMA_Memory0BaseAddr = 0; //this is set later when we fire the DMA

    dmaInitStruct.DMA_BufferSize = 1;     //this is set later when we fire the DMA, can't be 0

    DMA_Init(DMA_SPI_TX_DMA_STREAM, &dmaInitStruct);

    dmaInitStruct.DMA_Channel = DMA_SPI_RX_DMA_CHANNEL;
    dmaInitStruct.DMA_Priority = DMA_Priority_High;
    dmaInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;

    DMA_Init(DMA_SPI_RX_DMA_STREAM, &dmaInitStruct);

    //setup interrupt
    nvicInitStruct.NVIC_IRQChannel = DMA_SPI_RX_DMA_IRQn;
    nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    nvicInitStruct.NVIC_IRQChannelSubPriority = 1;
    nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInitStruct);
    DMA_ITConfig(DMA_SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
}

void dmaSpiTransmitReceive(uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t size, uint32_t blockingRead)
{
    //set buffer size
    DMA_SetCurrDataCounter(DMA_SPI_TX_DMA_STREAM, size);
    DMA_SetCurrDataCounter(DMA_SPI_RX_DMA_STREAM, size);

    //set buffer
    DMA_SPI_TX_DMA_STREAM->M0AR = (uint32_t)txBuffer;
    DMA_SPI_RX_DMA_STREAM->M0AR = (uint32_t)rxBuffer;

    //enable DMA SPI streams
    DMA_Cmd(DMA_SPI_TX_DMA_STREAM, ENABLE);
    DMA_Cmd(DMA_SPI_RX_DMA_STREAM, ENABLE);

    //enable  CS
    dmaSpiCsLo();

    //enable DMA SPI requests
    SPI_I2S_DMACmd(DMA_SPI_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_I2S_DMACmd(DMA_SPI_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

    //enable and send
    SPI_Cmd(DMA_SPI_SPI, ENABLE);

    if(!blockingRead)
        dmaSpiReadStatus = DMA_SPI_READ_IN_PROGRESS;
    else
        dmaSpiReadStatus = DMA_SPI_BLOCKING_READ_IN_PROGRESS;
}
#endif