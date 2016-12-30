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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MAX7456

#include "common/printf.h"

#include "drivers/bus_spi.h"
#include "drivers/light_led.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "drivers/nvic.h"
#include "drivers/dma.h"

#include "max7456.h"
#include "max7456_symbols.h"

//on shared SPI buss we want to change clock for OSD chip and restore for other devices
#ifdef MAX7456_SPI_CLK
    #define ENABLE_MAX7456        {spiSetDivisor(MAX7456_SPI_INSTANCE, MAX7456_SPI_CLK);IOLo(max7456CsPin);}
#else
    #define ENABLE_MAX7456        IOLo(max7456CsPin)
#endif

#ifdef MAX7456_RESTORE_CLK
    #define DISABLE_MAX7456       {IOHi(max7456CsPin);spiSetDivisor(MAX7456_SPI_INSTANCE, MAX7456_RESTORE_CLK);}
#else
    #define DISABLE_MAX7456       IOHi(max7456CsPin)
#endif

uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

// we write everything in screenBuffer and then comapre
// screenBuffer with shadowBuffer to upgrade only changed chars
// this solution is faster then redraw all screen
static uint8_t screenBuffer[VIDEO_BUFFER_CHARS_PAL+40]; //for faster writes we use memcpy so we need some space to don't overwrite buffer
static uint8_t shadowBuffer[VIDEO_BUFFER_CHARS_PAL];

//max chars to update in one idle
#define MAX_CHARS2UPDATE    100
#ifdef MAX7456_DMA_CHANNEL_TX
volatile bool dmaTransactionInProgress = false;
#endif

static uint8_t spiBuff[MAX_CHARS2UPDATE*6];

static uint8_t  videoSignalCfg   = 0;
static uint8_t  videoSignalReg   = VIDEO_MODE_PAL | OSD_ENABLE; //PAL by default
static bool  max7456Lock        = false;
static bool fontIsLoading       = false;
static IO_t max7456CsPin        = IO_NONE;


static uint8_t max7456Send(uint8_t add, uint8_t data)
{
    spiTransferByte(MAX7456_SPI_INSTANCE, add);
    return spiTransferByte(MAX7456_SPI_INSTANCE, data);
}

#ifdef MAX7456_DMA_CHANNEL_TX
static void max7456SendDma(void* tx_buffer, void* rx_buffer, uint16_t buffer_size)
{
    DMA_InitTypeDef DMA_InitStructure;
#ifdef MAX7456_DMA_CHANNEL_RX
    static uint16_t dummy[] = {0xffff};
#else
    UNUSED(rx_buffer);
#endif
    while (dmaTransactionInProgress); // Wait for prev DMA transaction

    DMA_DeInit(MAX7456_DMA_CHANNEL_TX);
#ifdef MAX7456_DMA_CHANNEL_RX
    DMA_DeInit(MAX7456_DMA_CHANNEL_RX);
#endif

    // Common to both channels
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(MAX7456_SPI_INSTANCE->DR));
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_BufferSize = buffer_size;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;

#ifdef MAX7456_DMA_CHANNEL_RX
    // Rx Channel
#ifdef STM32F4
    DMA_InitStructure.DMA_Memory0BaseAddr = rx_buffer ? (uint32_t)rx_buffer : (uint32_t)(dummy);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
#else
    DMA_InitStructure.DMA_MemoryBaseAddr = rx_buffer ? (uint32_t)rx_buffer : (uint32_t)(dummy);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
#endif
    DMA_InitStructure.DMA_MemoryInc = rx_buffer ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;

    DMA_Init(MAX7456_DMA_CHANNEL_RX, &DMA_InitStructure);
    DMA_Cmd(MAX7456_DMA_CHANNEL_RX, ENABLE);
#endif
    // Tx channel

#ifdef STM32F4
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tx_buffer; //max7456_screen;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
#else
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)tx_buffer; //max7456_screen;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
#endif
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    DMA_Init(MAX7456_DMA_CHANNEL_TX, &DMA_InitStructure);
    DMA_Cmd(MAX7456_DMA_CHANNEL_TX, ENABLE);

#ifdef MAX7456_DMA_CHANNEL_RX
    DMA_ITConfig(MAX7456_DMA_CHANNEL_RX, DMA_IT_TC, ENABLE);
#else
    DMA_ITConfig(MAX7456_DMA_CHANNEL_TX, DMA_IT_TC, ENABLE);
#endif

    // Enable SPI TX/RX request
    ENABLE_MAX7456;
    dmaTransactionInProgress = true;

    SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE,
#ifdef MAX7456_DMA_CHANNEL_RX
            SPI_I2S_DMAReq_Rx |
#endif
            SPI_I2S_DMAReq_Tx, ENABLE);
}

void max7456_dma_irq_handler(dmaChannelDescriptor_t* descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
#ifdef MAX7456_DMA_CHANNEL_RX
        DMA_Cmd(MAX7456_DMA_CHANNEL_RX, DISABLE);
#endif
        // make sure spi dmd transfer is complete
        while (SPI_I2S_GetFlagStatus (MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET) {};
        while (SPI_I2S_GetFlagStatus (MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_BSY) == SET) {};

        //Empty RX buffer. RX DMA takes care of it if enabled
        //this should be done after transmission finish!!!
        while (SPI_I2S_GetFlagStatus(MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == SET) {
            MAX7456_SPI_INSTANCE->DR;
        }

        DMA_Cmd(MAX7456_DMA_CHANNEL_TX, DISABLE);

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

        SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE,
#ifdef MAX7456_DMA_CHANNEL_RX
                SPI_I2S_DMAReq_Rx |
#endif
                SPI_I2S_DMAReq_Tx, DISABLE);

        DISABLE_MAX7456;
        dmaTransactionInProgress = false;
    }

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_HTIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    }
}

#endif

uint8_t max7456GetRowsCount(void)
{
    if (videoSignalReg & VIDEO_MODE_PAL)
        return VIDEO_LINES_PAL;

    return VIDEO_LINES_NTSC;
}

//because MAX7456 need some time to detect video system etc. we need to wait for a while to initialize it at startup
//and in case of restart we need to reinitialize chip
void max7456ReInit(void)
{
    uint8_t maxScreenRows;
    uint8_t srdata = 0;
    uint16_t x;
    static bool firstInit = true;

    //do not init MAX before camera power up correctly
    if (millis() < 1500)
        return;

    ENABLE_MAX7456;

    switch(videoSignalCfg) {
        case PAL:
            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
            break;
        case NTSC:
            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
            break;
        default:
            srdata = max7456Send(MAX7456ADD_STAT, 0x00);
            if ((0x02 & srdata) == 0x02)
                videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
    }

    if (videoSignalReg & VIDEO_MODE_PAL) { //PAL
        maxScreenSize = VIDEO_BUFFER_CHARS_PAL;
        maxScreenRows = VIDEO_LINES_PAL;
    } else {              // NTSC
        maxScreenSize = VIDEO_BUFFER_CHARS_NTSC;
        maxScreenRows = VIDEO_LINES_NTSC;
    }

    // set all rows to same charactor black/white level
    for(x = 0; x < maxScreenRows; x++) {
        max7456Send(MAX7456ADD_RB0 + x, BWBRIGHTNESS);
    }

    // make sure the Max7456 is enabled
    max7456Send(VM0_REG, videoSignalReg);
    max7456Send(DMM_REG, CLEAR_DISPLAY);
    DISABLE_MAX7456;

    //clear shadow to force redraw all screen in non-dma mode
    memset(shadowBuffer, 0, maxScreenSize);
    if (firstInit)
    {
        max7456RefreshAll();
        firstInit = false;
    }
}


//here we init only CS and try to init MAX for first time
void max7456Init(uint8_t video_system)
{
#ifdef MAX7456_SPI_CS_PIN
    max7456CsPin = IOGetByTag(IO_TAG(MAX7456_SPI_CS_PIN));
#endif
    IOInit(max7456CsPin, OWNER_OSD, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(max7456CsPin, SPI_IO_CS_CFG);

    spiSetDivisor(MAX7456_SPI_INSTANCE, SPI_CLOCK_STANDARD);
    // force soft reset on Max7456
    ENABLE_MAX7456;
    max7456Send(VM0_REG, MAX7456_RESET);
    DISABLE_MAX7456;
    videoSignalCfg = video_system;

#ifdef MAX7456_DMA_CHANNEL_TX
    dmaSetHandler(MAX7456_DMA_IRQ_HANDLER_ID, max7456_dma_irq_handler, NVIC_PRIO_MAX7456_DMA, 0);
#endif
    //real init will be made letter when driver idle detect
}

//just fill with spaces with some tricks
void max7456ClearScreen(void)
{
	uint16_t x;
	uint32_t *p = (uint32_t*)&screenBuffer[0];
	for (x = 0; x < VIDEO_BUFFER_CHARS_PAL/4; x++)
		p[x] = 0x20202020;
}

uint8_t* max7456GetScreenBuffer(void) {
    return screenBuffer;
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
	screenBuffer[y*30+x] = c;
}

void max7456Write(uint8_t x, uint8_t y, const char *buff)
{
	uint8_t i = 0;
	for (i = 0; *(buff+i); i++)
		if (x+i < 30) //do not write over screen
			screenBuffer[y*30+x+i] = *(buff+i);
}

#ifdef MAX7456_DMA_CHANNEL_TX
bool max7456DmaInProgres(void)
{
    return dmaTransactionInProgress;
}
#endif

void max7456DrawScreen(void)
{
    uint8_t check;
    static uint16_t pos = 0;
    int k = 0, buff_len=0;

    if (!max7456Lock && !fontIsLoading) {
        //-----------------detect MAX7456 fail, or initialize it at startup when it is ready--------
        max7456Lock = true;
        ENABLE_MAX7456;
        check = max7456Send(VM0_REG | 0x80, 0x00);
        DISABLE_MAX7456;

        if ( check != videoSignalReg)
            max7456ReInit();

        //------------   end of (re)init-------------------------------------

        for (k=0; k< MAX_CHARS2UPDATE; k++) {
            if (screenBuffer[pos] != shadowBuffer[pos]) {
                spiBuff[buff_len++] = MAX7456ADD_DMAH;
                spiBuff[buff_len++] = pos >> 8;
                spiBuff[buff_len++] = MAX7456ADD_DMAL;
                spiBuff[buff_len++] = pos & 0xff;
                spiBuff[buff_len++] = MAX7456ADD_DMDI;
                spiBuff[buff_len++] = screenBuffer[pos];
                shadowBuffer[pos] = screenBuffer[pos];
                k++;
            }

            if (++pos >= maxScreenSize) {
                pos = 0;
                break;
            }
        }

        if (buff_len) {
            #ifdef MAX7456_DMA_CHANNEL_TX
            if (buff_len > 0)
                max7456SendDma(spiBuff, NULL, buff_len);
            #else
            ENABLE_MAX7456;
            for (k=0; k < buff_len; k++)
                spiTransferByte(MAX7456_SPI_INSTANCE, spiBuff[k]);
            DISABLE_MAX7456;
            #endif // MAX7456_DMA_CHANNEL_TX
        }
        max7456Lock = false;
    }
}

// this funcktion refresh all and should not be used when copter is armed
void max7456RefreshAll(void)
{
    if (!max7456Lock) {
#ifdef MAX7456_DMA_CHANNEL_TX
    while (dmaTransactionInProgress);
#endif
        uint16_t xx;
        max7456Lock = true;
        ENABLE_MAX7456;
        max7456Send(MAX7456ADD_DMAH, 0);
        max7456Send(MAX7456ADD_DMAL, 0);
        max7456Send(MAX7456ADD_DMM, 1);

        for (xx = 0; xx < maxScreenSize; ++xx)
        {
            max7456Send(MAX7456ADD_DMDI, screenBuffer[xx]);
            shadowBuffer[xx] = screenBuffer[xx];
        }

        max7456Send(MAX7456ADD_DMDI, 0xFF);
        max7456Send(MAX7456ADD_DMM, 0);
        DISABLE_MAX7456;
        max7456Lock = false;
    }
}

void max7456WriteNvm(uint8_t char_address, const uint8_t *font_data)
{
    uint8_t x;

#ifdef MAX7456_DMA_CHANNEL_TX
    while (dmaTransactionInProgress);
#endif
    while (max7456Lock);
    max7456Lock = true;

    ENABLE_MAX7456;
    // disable display
    fontIsLoading = true;
    max7456Send(VM0_REG, 0);

    max7456Send(MAX7456ADD_CMAH, char_address); // set start address high

    for(x = 0; x < 54; x++) {
        max7456Send(MAX7456ADD_CMAL, x); //set start address low
        max7456Send(MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        LED1_TOGGLE;
#endif
    }

    // transfer 54 bytes from shadow ram to NVM
    max7456Send(MAX7456ADD_CMM, WRITE_NVR);

    // wait until bit 5 in the status register returns to 0 (12ms)
    while ((max7456Send(MAX7456ADD_STAT, 0x00) & STATUS_REG_NVR_BUSY) != 0x00);

    DISABLE_MAX7456;

    max7456Lock = false;
}


#endif
