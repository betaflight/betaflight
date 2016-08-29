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

#include "platform.h"
#include "version.h"

#ifdef USE_MAX7456

#include "common/printf.h"
#include "drivers/bus_spi.h"
#include "drivers/light_led.h"
#include "drivers/system.h"
#include "drivers/nvic.h"
#include "drivers/dma.h"

#include "max7456.h"
#include "max7456_symbols.h"

#define DISABLE_MAX7456       IOHi(max7456CsPin)
#define ENABLE_MAX7456        IOLo(max7456CsPin)

uint16_t max_screen_size;
static MAX7456_CHAR_TYPE max7456_screen[VIDEO_BUFFER_CHARS_PAL + 5];
#define SCREEN_BUFFER ((MAX7456_CHAR_TYPE*)&max7456_screen[3])

#ifdef MAX7456_DMA_CHANNEL_TX
volatile uint8_t dma_transaction_in_progress = 0;
#endif

static uint8_t  video_signal_type   = 0;
static uint8_t  max7456_lock        = 0;
static IO_t max7456CsPin            = IO_NONE;


MAX7456_CHAR_TYPE* max7456_get_screen_buffer(void) {
    return SCREEN_BUFFER;
}

static uint8_t max7456_send(uint8_t add, uint8_t data)
{
    spiTransferByte(MAX7456_SPI_INSTANCE, add);
    return spiTransferByte(MAX7456_SPI_INSTANCE, data);
}

#ifdef MAX7456_DMA_CHANNEL_TX
static void max7456_send_dma(void* tx_buffer, void* rx_buffer, uint16_t buffer_size) {
    DMA_InitTypeDef DMA_InitStructure;
#ifdef MAX7456_DMA_CHANNEL_RX
    static uint16_t dummy[] = {0xffff};
#else
    UNUSED(rx_buffer);
#endif
    while (dma_transaction_in_progress); // Wait for prev DMA transaction

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
    dma_transaction_in_progress = 1;

    SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE,
#ifdef MAX7456_DMA_CHANNEL_RX
            SPI_I2S_DMAReq_Rx |
#endif
            SPI_I2S_DMAReq_Tx, ENABLE);
}

void max7456_dma_irq_handler(dmaChannelDescriptor_t* descriptor) {
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
#ifdef MAX7456_DMA_CHANNEL_RX
        DMA_Cmd(MAX7456_DMA_CHANNEL_RX, DISABLE);
#else
        //Empty RX buffer. RX DMA takes care of it if enabled
        while (SPI_I2S_GetFlagStatus(MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == SET) {
            MAX7456_SPI_INSTANCE->DR;
        }
#endif
        DMA_Cmd(MAX7456_DMA_CHANNEL_TX, DISABLE);

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

        SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE,
#ifdef MAX7456_DMA_CHANNEL_RX
                SPI_I2S_DMAReq_Rx |
#endif
                SPI_I2S_DMAReq_Tx, DISABLE);

        DISABLE_MAX7456;
        for (uint16_t x = 0; x < max_screen_size; x++)
            max7456_screen[x + 3] = MAX7456_CHAR(' ');
        dma_transaction_in_progress = 0;
    }
}
#endif

void max7456_init(uint8_t video_system)
{
    uint8_t max_screen_rows;
    uint8_t srdata = 0;
    uint16_t x;

#ifdef MAX7456_SPI_CS_PIN
    max7456CsPin = IOGetByTag(IO_TAG(MAX7456_SPI_CS_PIN));
#endif
    IOInit(max7456CsPin, OWNER_OSD, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(max7456CsPin, SPI_IO_CS_CFG);

    //Minimum spi clock period for max7456 is 100ns (10Mhz)
    spiSetDivisor(MAX7456_SPI_INSTANCE, SPI_CLOCK_STANDARD);

    delay(1000);
    // force soft reset on Max7456
    ENABLE_MAX7456;
    max7456_send(VM0_REG, MAX7456_RESET);
    delay(100);

    srdata = max7456_send(0xA0, 0xFF);
    if ((0x01 & srdata) == 0x01) {     //PAL
          video_signal_type = VIDEO_MODE_PAL;
    }
    else if ((0x02 & srdata) == 0x02) { //NTSC
        video_signal_type = VIDEO_MODE_NTSC;
    }

    // Override detected type: 0-AUTO, 1-PAL, 2-NTSC
    switch(video_system) {
        case PAL:
            video_signal_type = VIDEO_MODE_PAL;
            break;
        case NTSC:
            video_signal_type = VIDEO_MODE_NTSC;
            break;
    }

    if (video_signal_type) { //PAL
        max_screen_size = VIDEO_BUFFER_CHARS_PAL;
        max_screen_rows = VIDEO_LINES_PAL;
    } else {                 // NTSC
        max_screen_size = VIDEO_BUFFER_CHARS_NTSC;
        max_screen_rows = VIDEO_LINES_NTSC;
    }

    // set all rows to same charactor black/white level
    for(x = 0; x < max_screen_rows; x++) {
        max7456_send(MAX7456ADD_RB0 + x, BWBRIGHTNESS);
    }

    // make sure the Max7456 is enabled
    max7456_send(VM0_REG, OSD_ENABLE | video_signal_type);

    DISABLE_MAX7456;
    delay(100);

    for (x = 0; x < max_screen_size; x++)
        SCREEN_BUFFER[x] = MAX7456_CHAR(' ');

#ifdef MAX7456_DMA_CHANNEL_TX
    max7456_screen[0] = (uint16_t)(MAX7456ADD_DMAH | (0 << 8));
    max7456_screen[1] = (uint16_t)(MAX7456ADD_DMAL | (0 << 8));
    max7456_screen[2] = (uint16_t)(MAX7456ADD_DMM | (1 << 8));
    max7456_screen[max_screen_size + 3] = (uint16_t)(MAX7456ADD_DMDI | (0xFF << 8));
    max7456_screen[max_screen_size + 4] = (uint16_t)(MAX7456ADD_DMM  | (0 << 8));

    dmaSetHandler(MAX7456_DMA_IRQ_HANDLER_ID, max7456_dma_irq_handler, NVIC_PRIO_MAX7456_DMA, 0);
#endif
}

// Copy string from ram into screen buffer
void max7456_write_string(const char *string, int16_t address) {
    MAX7456_CHAR_TYPE *dest;

    if (address >= 0)
        dest  = SCREEN_BUFFER + address;
    else
        dest  = SCREEN_BUFFER + (max_screen_size + address);

    while(*string && dest < (SCREEN_BUFFER + max_screen_size)) {
        *dest++ = MAX7456_CHAR(*string++);
    }
}

void max7456_draw_screen(void) {
    if (!max7456_lock) {
#ifdef MAX7456_DMA_CHANNEL_TX
        max7456_send_dma(max7456_screen, NULL, max_screen_size * 2 + 10);
#else
        uint16_t xx;
        max7456_lock = 1;

        ENABLE_MAX7456;
        max7456_send(MAX7456ADD_DMAH, 0);
        max7456_send(MAX7456ADD_DMAL, 0);
        max7456_send(MAX7456ADD_DMM, 1);
        for (xx = 0; xx < max_screen_size; ++xx) {
            max7456_send(MAX7456ADD_DMDI, SCREEN_BUFFER[xx]);
            SCREEN_BUFFER[xx] = MAX7456_CHAR(' ');
        }
        max7456_send(MAX7456ADD_DMDI, 0xFF);
        max7456_send(MAX7456ADD_DMM, 0);
        DISABLE_MAX7456;
        max7456_lock = 0;
#endif
    }
}

void max7456_write_nvm(uint8_t char_address, uint8_t *font_data) {
    uint8_t x;

#ifdef MAX7456_DMA_CHANNEL_TX
    while (dma_transaction_in_progress);
#endif
    while (max7456_lock);
    max7456_lock = 1;
    ENABLE_MAX7456;

    // disable display
    max7456_send(VM0_REG, video_signal_type);

    max7456_send(MAX7456ADD_CMAH, char_address); // set start address high

    for(x = 0; x < 54; x++) {
        max7456_send(MAX7456ADD_CMAL, x); //set start address low
        max7456_send(MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        LED1_TOGGLE;
#endif
    }

    // transfer 54 bytes from shadow ram to NVM
    max7456_send(MAX7456ADD_CMM, WRITE_NVR);

    // wait until bit 5 in the status register returns to 0 (12ms)
    while ((max7456_send(MAX7456ADD_STAT, 0) & STATUS_REG_NVR_BUSY) != 0x00);

    max7456_send(VM0_REG, video_signal_type | 0x0C);
    DISABLE_MAX7456;
    max7456_lock = 0;
}


#endif
