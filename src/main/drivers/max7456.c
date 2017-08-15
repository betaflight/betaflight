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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

//#ifdef USE_MAX7456
#if 1

#include "common/printf.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "drivers/bus_spi.h"
#include "drivers/display.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/max7456.h"
#include "drivers/max7456_symbols.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/vcd.h"

static sbuf_t max7456StreamBuffer;
static sbuf_t *sbuf = &max7456StreamBuffer;


#ifdef MAX7456_DMA_CHANNEL_TX
volatile bool dmaTransactionInProgress = false;
#endif

//#define SPI_BUFF_SIZE 6+2+32 //MAX_CHARS2UPDATE*6];
//static uint8_t spiBuff[SPI_BUFF_SIZE];

static uint8_t  videoSignalCfg;
static uint8_t  videoSignalReg  = OSD_ENABLE; // OSD_ENABLE required to trigger first ReInit
static uint8_t  displayMemoryModeReg = 0;

static uint8_t  hosRegValue; // HOS (Horizontal offset register) value
static uint8_t  vosRegValue; // VOS (Vertical offset register) value

static bool  max7456Lock        = false;
static bool fontIsLoading       = false;
static IO_t max7456CsPin        = IO_NONE;

// allow writes of at least the line width
#define MAX7456_MAX_STRING_LENGTH             CHARS_PER_LINE
#define MAX7456_MAX_FRAME_LENGTH              (6 + 2 * MAX7456_MAX_STRING_LENGTH)*5  // FIXME: DEBUGGING
static uint8_t max7456Buffer[MAX7456_MAX_FRAME_LENGTH];

static void max7456ReloadProfilePrivate();

static uint8_t max7456ReadWriteRegister(uint8_t add, uint8_t data)
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
        // Make sure SPI DMA transfer is complete

        while (SPI_I2S_GetFlagStatus (MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET) {};
        while (SPI_I2S_GetFlagStatus (MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_BSY) == SET) {};

        // Empty RX buffer. RX DMA takes care of it if enabled.
        // This should be done after transmission finish!!!

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
    return (videoSignalReg & VIDEO_MODE_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
}

void max7456ReInit(void)
{
    //uint8_t maxScreenRows;
    uint8_t srdata = 0;
    static bool firstInit = true;

    ENABLE_MAX7456;

    switch (videoSignalCfg) {
        case VIDEO_SYSTEM_PAL:
            videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
            break;

        case VIDEO_SYSTEM_NTSC:
            videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
            break;

        case VIDEO_SYSTEM_AUTO:
            srdata = max7456ReadWriteRegister(MAX7456ADD_STAT, 0x00);

            if (VIN_IS_NTSC(srdata)) {
                videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
            } else if (VIN_IS_PAL(srdata)) {
                videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
            } else {
                // No valid input signal, fallback to default (XXX NTSC for now)
                videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE;
            }
            break;
    }

    if (videoSignalReg & VIDEO_MODE_PAL) { //PAL
        //maxScreenSize = VIDEO_BUFFER_CHARS_PAL;
        //maxScreenRows = VIDEO_LINES_PAL;
    } else {              // NTSC
        //maxScreenSize = VIDEO_BUFFER_CHARS_NTSC;
        //maxScreenRows = VIDEO_LINES_NTSC;
    }

    // Make sure the Max7456 is enabled
    max7456ReadWriteRegister(MAX7456ADD_VM0, videoSignalReg);
    max7456ReadWriteRegister(MAX7456ADD_HOS, hosRegValue);
    max7456ReadWriteRegister(MAX7456ADD_VOS, vosRegValue);
    max7456ReadWriteRegister(MAX7456ADD_DMM, displayMemoryModeReg | CLEAR_DISPLAY);
    DISABLE_MAX7456;

    // reload profile settings
    max7456ReloadProfilePrivate();

    // Clear shadow to force redraw all screen in non-dma mode.
    if (firstInit)
    {
        max7456RefreshAll();
        firstInit = false;
    }
}


// Here we init only CS and try to init MAX for first time.

void max7456Init(const vcdProfile_t *pVcdProfile)
{
    max7456HardwareReset();

#ifdef MAX7456_SPI_CS_PIN
    max7456CsPin = IOGetByTag(IO_TAG(MAX7456_SPI_CS_PIN));
#endif
    IOInit(max7456CsPin, OWNER_OSD_CS, 0);
    IOConfigGPIO(max7456CsPin, SPI_IO_CS_CFG);
    IOHi(max7456CsPin);

    spiSetDivisor(MAX7456_SPI_INSTANCE, SPI_CLOCK_STANDARD);
    // force soft reset on Max7456
    ENABLE_MAX7456;
    max7456ReadWriteRegister(MAX7456ADD_VM0, MAX7456_RESET);
    DISABLE_MAX7456;

    // Setup values to write to registers
    videoSignalCfg = pVcdProfile->video_system;
    hosRegValue = 32 - pVcdProfile->h_offset;
    vosRegValue = 16 - pVcdProfile->v_offset;

#ifdef MAX7456_DMA_CHANNEL_TX
    dmaSetHandler(MAX7456_DMA_IRQ_HANDLER_ID, max7456_dma_irq_handler, NVIC_PRIO_MAX7456_DMA, 0);
#endif

    // Real init will be made later when driver detect idle.
}

static void max7456ReloadProfilePrivate()
{
    if (displayPortProfile()->invert) {
        displayMemoryModeReg |= INVERT_PIXEL_COLOR;
    } else {
        displayMemoryModeReg &= ~INVERT_PIXEL_COLOR;
    }

    // max7456 brightness:
    // black: 0 =   0%  1 =  10%  2 =  20%  3 =  30%
    // white: 0 =  80%  1 =  90%  2 = 100%  3 = 120% (inverted in driver)
    // map 0..100 -> 0..3
    uint8_t black = (displayPortProfile()->blackBrightness) / 33;
    // map 0..100 -> 0..3
    uint8_t white = (displayPortProfile()->whiteBrightness) / 33;
    // register value
    uint8_t reg = (black << 2) | (3 - white);

    ENABLE_MAX7456;
    // set inverted/normal
    max7456ReadWriteRegister(MAX7456ADD_DMM, displayMemoryModeReg);

    // send brightness to max7456
    for (int i = MAX7456ADD_RB0; i <= MAX7456ADD_RB15; i++) {
        max7456ReadWriteRegister(i, reg);
    }
    DISABLE_MAX7456;
}

void max7456ReloadProfile()
{
    if (!max7456Lock){
        max7456Lock = true;
        max7456ReloadProfilePrivate();
        max7456Lock = false;
    }
}

//just fill with spaces with some tricks
void max7456ClearScreen(void)
{
    // FIXME: font character at address zero has to be changed to an empty char!
    if (!max7456Lock){
        max7456Lock = true;
        ENABLE_MAX7456;
        max7456ReadWriteRegister(MAX7456ADD_DMM, displayMemoryModeReg | CLEAR_DISPLAY);
        DISABLE_MAX7456;
        max7456Lock = false;
    }
}

void max7456FillRegion(uint8_t xs, uint8_t ys, uint8_t width, uint8_t height, uint8_t value)
{
    // shortcut for complete fill detection:
    if ((value == ' ') || (value == 0)) {
        if ((ys >= max7456GetRowsCount()) && (xs >= CHARS_PER_LINE)) {
            // not checking xs/ys to be zero as the height and width are enough
            // issue speed optimized clear all command
            max7456ClearScreen();
            return;
        }
    }

    // create null terminated "fill" string
    uint8_t buffer[width+1];
    memset(buffer, value, width);
    buffer[width] = 0;

    uint8_t y = ys;
    while (height > 0) {
        // output string:
        max7456WriteString(xs, y, (const char *)buffer);

        // keep track of position and count
        height--;
        y++;
    }
}


void max7456StallCheck(void)
{
    uint8_t stallCheck;
    uint8_t videoSense;
    static uint32_t lastSigCheckMs = 0;
    uint32_t nowMs;
    static uint32_t videoDetectTimeMs = 0;

    if (!max7456Lock){
        max7456Lock = true;

        ENABLE_MAX7456;
        stallCheck = max7456ReadWriteRegister(MAX7456ADD_VM0|MAX7456ADD_READ, 0x00);
        DISABLE_MAX7456;

        nowMs = millis();

        if (stallCheck != videoSignalReg) {
            max7456ReInit();

        } else if ((videoSignalCfg == VIDEO_SYSTEM_AUTO)
                  && ((nowMs - lastSigCheckMs) > MAX7456_SIGNAL_CHECK_INTERVAL_MS)) {

            // Adjust output format based on the current input format.

            ENABLE_MAX7456;
            videoSense = max7456ReadWriteRegister(MAX7456ADD_STAT, 0x00);
            DISABLE_MAX7456;

#ifdef DEBUG_MAX7456_SIGNAL
            debug[0] = videoSignalReg & VIDEO_MODE_MASK;
            debug[1] = videoSense & 0x7;
            debug[3] = max7456GetRowsCount();
#endif

            if (videoSense & STAT_LOS) {
                videoDetectTimeMs = 0;
            } else {
                if ((VIN_IS_PAL(videoSense) && VIDEO_MODE_IS_NTSC(videoSignalReg))
                  || (VIN_IS_NTSC_alt(videoSense) && VIDEO_MODE_IS_PAL(videoSignalReg))) {
                    if (videoDetectTimeMs) {
                        if (millis() - videoDetectTimeMs > VIDEO_SIGNAL_DEBOUNCE_MS) {
                            max7456ReInit();
#ifdef DEBUG_MAX7456_SIGNAL
                            debug[2]++;
#endif
                        }
                    } else {
                        // Wait for signal to stabilize
                        videoDetectTimeMs = millis();
                    }
                }
            }

            lastSigCheckMs = nowMs;
        }
        max7456Lock = false;
    }
}

void max7456SendBuffer(sbuf_t *buf)
{
    if (!max7456Lock) {
        // prepare to send buffer
        max7456Lock = true;

        // switch buffer to reader
        sbufSwitchToReader(buf, max7456Buffer);

        #ifdef MAX7456_DMA_CHANNEL_TX
        max7456SendDma(sbufPtr(buf), NULL, sbufBytesRemaining(buf));
        #else
        ENABLE_MAX7456;
        uint8_t todo = sbufBytesRemaining(buf);
        for (uint8_t k=0; k < todo; k++) {
            // transfer all bytes
            spiTransferByte(MAX7456_SPI_INSTANCE, sbufReadU8(buf));
        }

        DISABLE_MAX7456;
        #endif // MAX7456_DMA_CHANNEL_TX
        max7456Lock = false;
    }
}

void max7456WriteString(uint8_t x, uint8_t y, const char *buff)
{
    max7456StallCheck();

    // check for overflows, ignore strings that are to long
    uint32_t len = strlen(buff);
    if (len >= MAX7456_MAX_STRING_LENGTH) return;
    if (len == 0) return;

    // point to the buffer
    sbuf->ptr = max7456Buffer;
    sbuf->end = ARRAYEND(max7456Buffer);

    // calc framebuffer address
    uint16_t pos = y * CHARS_PER_LINE + x;

    // set address
    sbufWriteU8(sbuf, MAX7456ADD_DMAH);
    sbufWriteU8(sbuf, pos >> 8);
    sbufWriteU8(sbuf, MAX7456ADD_DMAL);
    sbufWriteU8(sbuf, pos & 0xFF);

    // use autoincrement mode?
    if (len != 1) {
        // activate auto increment
        sbufWriteU8(sbuf, MAX7456ADD_DMM);
        sbufWriteU8(sbuf, displayMemoryModeReg | 0x01);
    }

    // add string data
    while (*buff) {
        sbufWriteU8(sbuf, MAX7456ADD_DMDI);
        sbufWriteU8(sbuf, *buff++);
    }

    // end auto increment mode
    if (len != 1) {
        sbufWriteU8(sbuf, MAX7456ADD_DMDI);
        sbufWriteU8(sbuf, 0xFF);
    }

    // send streambuffer
    max7456SendBuffer(sbuf);
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    // point to the buffer
    sbuf->ptr = max7456Buffer;
    sbuf->end = ARRAYEND(max7456Buffer);

    // calc framebuffer address
    uint16_t pos = y * CHARS_PER_LINE + x;

    // set address
    sbufWriteU8(sbuf, MAX7456ADD_DMAH);
    sbufWriteU8(sbuf, pos >> 8);
    sbufWriteU8(sbuf, MAX7456ADD_DMAL);
    sbufWriteU8(sbuf, pos & 0xFF);

    // add data
    sbufWriteU8(sbuf, MAX7456ADD_DMDI);
    sbufWriteU8(sbuf, c);

    // send streambuffer
    max7456SendBuffer(sbuf);
}

bool max7456DmaInProgress(void)
{
#ifdef MAX7456_DMA_CHANNEL_TX
    return dmaTransactionInProgress;
#else
    return false;
#endif
}

#include "build/debug.h"

void max7456DrawScreenPartial(void)
{
    // nothing to do
}

// This funcktion refresh all and should not be used when copter is armed
void max7456RefreshAll(void)
{
    // nothing to do
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
    max7456ReadWriteRegister(MAX7456ADD_VM0, 0);

    max7456ReadWriteRegister(MAX7456ADD_CMAH, char_address); // set start address high



    for (x = 0; x < 54; x++) {
        max7456ReadWriteRegister(MAX7456ADD_CMAL, x); //set start address low
        max7456ReadWriteRegister(MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        LED1_TOGGLE;
#endif
    }

    // Transfer 54 bytes from shadow ram to NVM

    max7456ReadWriteRegister(MAX7456ADD_CMM, WRITE_NVR);

    // Wait until bit 5 in the status register returns to 0 (12ms)

    while ((max7456ReadWriteRegister(MAX7456ADD_STAT, 0x00) & STAT_NVR_BUSY) != 0x00);

    DISABLE_MAX7456;

    max7456Lock = false;
}

#ifdef MAX7456_NRST_PIN
static IO_t max7456ResetPin        = IO_NONE;
#endif

void max7456HardwareReset(void)
{
#ifdef MAX7456_NRST_PIN
#define IO_RESET_CFG      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)

    max7456ResetPin = IOGetByTag(IO_TAG(MAX7456_NRST_PIN));
    IOInit(max7456ResetPin, OWNER_OSD, 0);
    IOConfigGPIO(max7456ResetPin, IO_RESET_CFG);


    // RESET
    IOLo(max7456ResetPin);
    delay(100);
    IOHi(max7456ResetPin);
#endif
}

#endif
