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

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include <debug.h>

#include "drivers/exti.h"
#include "drivers/video.h"
#include "drivers/video_textscreen.h"
#include "drivers/video_max7456.h"
#include "drivers/bus_spi.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/system.h"

#include "osd/fonts/font_max7456_12x18.h"

#define MAX7456_MODE_MASK_PAL 0x40
#define MAX7456_CENTER_PAL 0x8

#define MAX7456_MODE_MASK_NTSC 0x00
#define MAX7456_CENTER_NTSC 0x6

#define MAX7456_SHADOW_TO_NVM_MASK 0xA0

//MAX7456 read addresses
#define MAX7456_REG_OSDBL_READ 0xec //black level
#define MAX7456_REG_STAT_READ  0xa0 //0xa[X] Status
#define MAX7456_REG_VM0_READ  0x80
#define MAX7456_REG_DMM_READ   0x84

//MAX7456 write addresses
#define MAX7456_REG_VM0   0x00
#define MAX7456_REG_VM1   0x01
#define MAX7456_REG_DMM   0x04
#define MAX7456_REG_DMAH  0x05
#define MAX7456_REG_DMAL  0x06
#define MAX7456_REG_DMDI  0x07
#define MAX7456_REG_OSDM  0x0c
#define MAX7456_REG_OSDBL 0x6c

//MAX7456 write addresses
#define MAX7456_REG_CMM   0x08
#define MAX7456_REG_CMAH  0x09
#define MAX7456_REG_CMAL  0x0a
#define MAX7456_REG_CMDI  0x0b
#define MAX7456_REG_CMDO  0xc0

#define MAX7456_REG_RB0          0x10
#define MAX7456_REG_RB1          0x11
#define MAX7456_REG_RB2          0x12
#define MAX7456_REG_RB3          0x13
#define MAX7456_REG_RB4          0x14
#define MAX7456_REG_RB5          0x15
#define MAX7456_REG_RB6          0x16
#define MAX7456_REG_RB7          0x17
#define MAX7456_REG_RB8          0x18
#define MAX7456_REG_RB9          0x19
#define MAX7456_REG_RB10         0x1a
#define MAX7456_REG_RB11         0x1b
#define MAX7456_REG_RB12         0x1c
#define MAX7456_REG_RB13         0x1d
#define MAX7456_REG_RB14         0x1e
#define MAX7456_REG_RB15         0x1f

#define MAX7456_VM0_BIT_VIDEO_BUFFER_DISABLE    (0 << 0)
#define MAX7456_VM0_BIT_VIDEO_BUFFER_ENABLE     (1 << 0)
#define MAX7456_VM0_BIT_SOFTWARE_RESET          (1 << 1)
#define MAX7456_VM0_BIT_VSYNC_DISABLE           (0 << 2)
#define MAX7456_VM0_BIT_VSYNC_ENABLE            (1 << 2)
#define MAX7456_VM0_BIT_OSD_DISABLE             (0 << 3)
#define MAX7456_VM0_BIT_OSD_ENABLE              (1 << 3)
#define MAX7456_VM0_BIT_SYNC_MODE_EXTERNAL      ((1 << 5) | (0 << 4))
#define MAX7456_VM0_BIT_SYNC_MODE_INTERNAL      ((1 << 5) | (1 << 4))
#define MAX7456_VM0_BIT_VIDEO_MODE_PAL          (0 << 6)
#define MAX7456_VM0_BIT_VIDEO_MODE_NTSC         (1 << 6)

#define MAX7456_STAT_BIT_PAL_DETECTED           (1 << 0)
#define MAX7456_STAT_BIT_NTSC_DETECTED          (1 << 1)
#define MAX7456_STAT_BIT_LOS_OF_SYNC            (1 << 2)
#define MAX7456_STAT_BIT_HSYNC_INACTIVE         (1 << 3)
#define MAX7456_STAT_BIT_VSYNC_INACTIVE         (1 << 4)
#define MAX7456_STAT_BIT_CHAR_MEM_BUSY          (1 << 5)
#define MAX7456_STAT_BIT_RESET_MODE             (1 << 6)
#define MAX7456_STAT_BIT_NA                     (1 << 7)

#define MAX7456_DMM_BIT_8BIT_ENABLE             (1 << 6) // 16 bit when disabled
#define MAX7456_DMM_BIT_LBC                     (1 << 5) // only when in 16 bit mode
#define MAX7456_DMM_BIT_BLINK                   (1 << 4) // only when in 16 bit mode
#define MAX7456_DMM_BIT_INVERT                  (1 << 3) // only when in 16 bit mode
#define MAX7456_DMM_BIT_CLEAR                   (1 << 2)
#define MAX7456_DMM_BIT_VSYNC_CLEAR             (1 << 1)
#define MAX7456_DMM_BIT_AUTO_INCREMENT          (1 << 0)

#ifndef WHITEBRIGHTNESS
  #define WHITEBRIGHTNESS 0x01
#endif
#ifndef BLACKBRIGHTNESS
  #define BLACKBRIGHTNESS 0x00
#endif

#define BWBRIGHTNESS ((BLACKBRIGHTNESS << 2) | WHITEBRIGHTNESS)

uint8_t max7456_videoModeMask;
uint8_t max7456_screenRows;

#define DISABLE_MAX7456       GPIO_SetBits(MAX7456_CS_GPIO,   MAX7456_CS_PIN)
#define ENABLE_MAX7456        GPIO_ResetBits(MAX7456_CS_GPIO, MAX7456_CS_PIN)

textScreen_t max7456Screen;
max7456State_t max7456State;
static const extiConfig_t *max7456LOSExtiConfig;
static const extiConfig_t *max7456VSYNCExtiConfig;
static const extiConfig_t *max7456HSYNCExtiConfig;

#if 0
// for factory max7456 font
static const uint8_t max7456_defaultFont_asciiToFontMapping[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  0  -  15
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  16 -  31
    0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x00, 0x46, 0x3F, 0x40, 0x00, 0x00, 0x45, 0x49, 0x41, 0x47, //  32 -  47  " !"#$%&'()*+,-./"
    0x0A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x44, 0x43, 0x4A, 0x00, 0x4B, 0x42, //  48 -  63  "0123456789:;<=>?"
    0x4C, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, //  64 -  79  "@ABCDEFGHIJKLMNO"
    0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x00, 0x46, 0x00, 0x00, 0x00, //  80 -  95  "PQRSTUVWXYZ[\]^_"
    0x00, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, //  96 - 111  "`abcdefghijklmno"
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, // 112 - 127  "pqrstuvwxyz{|}~ "
};

static const uint8_t max7456_defaultFont_fontToASCIIMapping[] = {
    ' ', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'A', 'B', 'C', 'D', 'E', // 0x00 - 0x0F
    'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', // 0x10 - 0x1F
    'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', // 0x20 - 0x2F
    'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '(', // 0x30 - 0x3F
    ')', '.', '?', ';', ':', ',', '\'', '/', '"', '-', '<','>', '@'                 // 0x40 - 0x4F
};
#endif


static bool max7456ExtiHandler(const extiConfig_t *extiConfig)
{
    if (EXTI_GetITStatus(extiConfig->exti_line) == RESET) {
        return false;
    }

    EXTI_ClearITPendingBit(extiConfig->exti_line);

    return true;
}


void max7456_updateLOSState(void)
{
    // "LOS goes high when the VIN sync pulse is lost for 32 consecutive lines. LOS goes low when 32 consecutive valid sync pulses are received."
    uint8_t status = GPIO_ReadInputDataBit(max7456LOSExtiConfig->gpioPort, max7456LOSExtiConfig->gpioPin);

    max7456State.los = status != 0;

    //debug[0] = max7456State.los;
}

void LOS_EXTI_Handler(void)
{
    static uint32_t callCount = 0;
    bool set = max7456ExtiHandler(max7456LOSExtiConfig);
    callCount++;
    if (!set) {
        return;
    }

    max7456State.losCounter++;

    max7456_updateLOSState();
}

void VSYNC_EXTI_Handler(void)
{
    static uint32_t callCount = 0;
    bool set = max7456ExtiHandler(max7456VSYNCExtiConfig);
    callCount++;
    if (!set) {
        return;
    }

    max7456State.vSyncDetected = true;

    max7456State.frameCounter++;
    //debug[1] = max7456State.frameCounter;

    /* Time between VSYNC pulses can be measured using the code below.
     * NTSC: ~16673us (60 FPS), PAL: ~19990us (50FPS)
     */
#if DEBUG_MAX7456_VSYNC_TRIGGER
    static bool shouldStartMeasuring = true;

    if (shouldStartMeasuring) {
        TIME_SECTION_BEGIN(2);
    } else {
        TIME_SECTION_END(2);
    }
    shouldStartMeasuring = !shouldStartMeasuring;
#endif
}

void HSYNC_EXTI_Handler(void)
{
    static uint32_t callCount = 0;
    bool set = max7456ExtiHandler(max7456HSYNCExtiConfig);
    callCount++;
    if (!set) {
        return;
    }

    max7456State.hSyncDetected = true;
}


typedef void (*handlerFuncPtr)(void);

void max7456_extiInit(const extiConfig_t *extiConfig, handlerFuncPtr handlerFn, EXTITrigger_TypeDef trigger)
{
    gpio_config_t gpio;

#ifdef STM32F303
    if (extiConfig->gpioAHBPeripherals) {
        RCC_AHBPeriphClockCmd(extiConfig->gpioAHBPeripherals, ENABLE);
    }
#endif
#ifdef STM32F10X
    if (extiConfig->gpioAPB2Peripherals) {
        RCC_APB2PeriphClockCmd(extiConfig->gpioAPB2Peripherals, ENABLE);
    }
#endif

    gpio.pin = extiConfig->gpioPin;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(extiConfig->gpioPort, &gpio);

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

#ifdef STM32F10X
    gpioExtiLineConfig(extiConfig->exti_port_source, extiConfig->exti_pin_source);
#endif

#ifdef STM32F303xC
    gpioExtiLineConfig(extiConfig->exti_port_source, extiConfig->exti_pin_source);
#endif

    registerExtiCallbackHandler(extiConfig->exti_irqn, handlerFn);

    EXTI_ClearITPendingBit(extiConfig->exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = extiConfig->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = trigger;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);
}

void max7456_extiConfigure(
    const extiConfig_t *losExtiConfig,
    const extiConfig_t *vSyncExtiConfig,
    const extiConfig_t *hSyncExtiConfig
)
{
    max7456LOSExtiConfig = losExtiConfig;
    max7456VSYNCExtiConfig = vSyncExtiConfig;
    max7456HSYNCExtiConfig = hSyncExtiConfig;

    max7456_extiInit(max7456LOSExtiConfig, LOS_EXTI_Handler, EXTI_Trigger_Rising_Falling);
    max7456_extiInit(max7456VSYNCExtiConfig, VSYNC_EXTI_Handler, EXTI_Trigger_Falling);
    max7456_extiInit(max7456HSYNCExtiConfig, HSYNC_EXTI_Handler, EXTI_Trigger_Falling);
}

textScreen_t *max7456_getTextScreen(void)
{
    return &max7456Screen;
}

static void max7456_write(uint8_t address, uint8_t data)
{
    ENABLE_MAX7456;

    spiTransferByte(MAX7456_SPI_INSTANCE, address);
    spiTransferByte(MAX7456_SPI_INSTANCE, data);

    DISABLE_MAX7456;
}

static uint8_t max7456_read(uint8_t address)
{
    uint8_t result;

    ENABLE_MAX7456;

    spiTransferByte(MAX7456_SPI_INSTANCE, address);
    result = spiTransferByte(MAX7456_SPI_INSTANCE, 0xff);

    DISABLE_MAX7456;

    return result;
}

static void max7456_setVideoMode(videoMode_e videoMode)
{
    switch(videoMode)
    {
        case VIDEO_NTSC:
            max7456_videoModeMask = MAX7456_MODE_MASK_NTSC;
            max7456Screen.height = MAX7456_NTSC_ROW_COUNT;
            break;
         case VIDEO_PAL:
            max7456_videoModeMask = MAX7456_MODE_MASK_PAL;
            max7456Screen.height = MAX7456_PAL_ROW_COUNT;
            break;
    }

    max7456Screen.width = MAX7456_COLUMN_COUNT;
}

bool max7456_isOSDEnabled(void)
{
    uint8_t result = max7456_read(MAX7456_REG_VM0_READ);
    //debug[3] = result;

    bool osdEnabled = (result & MAX7456_VM0_BIT_OSD_ENABLE) != 0;

    return osdEnabled;
}

static bool max7456_isResetComplete(void)
{
    uint8_t result = max7456_read(MAX7456_REG_STAT_READ);
    //debug[3] = result;

    bool resetComplete = (result & MAX7456_STAT_BIT_RESET_MODE) == 0;

    return resetComplete;
}

static void max7456_softReset()
{
    static uint32_t resetWait = 0;

    // force soft reset on Max7456
    max7456_write(MAX7456_REG_VM0, MAX7456_VM0_BIT_SOFTWARE_RESET); // without video mode
    delayMicroseconds(100);

    while(!max7456_isResetComplete()) {
        resetWait++;
        delayMicroseconds(100);
    }
}

void max7456_hardwareReset(void)
{
    gpio_config_t cfg = { MAX7456_NRST_PIN, Mode_Out_PP, Speed_2MHz };

    RCC_AHBPeriphClockCmd(MAX7456_NRST_GPIO_PERIPHERAL, ENABLE);

    gpioInit(MAX7456_NRST_GPIO, &cfg);

    // RESET
    digitalLo(MAX7456_NRST_GPIO, MAX7456_NRST_PIN);
    delay(100);
    digitalHi(MAX7456_NRST_GPIO, MAX7456_NRST_PIN);
}

void max7456_disableOSD(void)
{
    max7456_write(MAX7456_REG_VM0, max7456_videoModeMask); // MAX7456_VM0_BIT_OSD_ENABLE unset.
}

void max7456_init(videoMode_e videoMode)
{
    // device is rated for 10mhz max
    spiSetDivisor(MAX7456_SPI_INSTANCE, SPI_9MHZ_CLOCK_DIVIDER);

    max7456_setVideoMode(videoMode);

    max7456_softReset();

    max7456_write(MAX7456_REG_OSDM, 0x00);
    //read black level register
    uint8_t blackLevelResult = max7456_read(MAX7456_REG_OSDBL_READ);

    // set all rows to same charactor black/white level
    uint8_t row;
    for(row = 0; row < max7456_screenRows; row++) {
        max7456_write(MAX7456_REG_RB0 + row, BWBRIGHTNESS);
    }

    max7456_write(MAX7456_REG_VM0, MAX7456_VM0_BIT_OSD_ENABLE | max7456_videoModeMask);
    delay(100);

    // Set black level
    uint8_t blackLevelValue = (blackLevelResult & 0xef); // Set bit 4 to zero 11101111 (bit is 0 based index)

    max7456_write(MAX7456_REG_OSDBL, blackLevelValue);
}

#define MAX7456_CHARACTER_BUFFER_SIZE 54
static void max7456_setFontCharacter(uint8_t characterIndex, const uint8_t *characterBitmap)
{
    // cannot update font NVM with OSD enabled.
    max7456_disableOSD();

    // a short delay is required after disabling the OSD before sending the character index otherwise the CMAH register is not updated.
    delay(20);
    max7456_write(MAX7456_REG_CMAH, characterIndex);

    // transfer character bitmap to character buffer in shadow ram
    for(uint8_t i = 0; i < MAX7456_CHARACTER_BUFFER_SIZE; i++)
    {
      max7456_write(MAX7456_REG_CMAL, i); // set start address low
      max7456_write(MAX7456_REG_CMDI, characterBitmap[i]);
    }

    // transfer character buffer from shadow ram to NVM
    max7456_write(MAX7456_REG_CMM, MAX7456_SHADOW_TO_NVM_MASK);

    // wait until bit 5 in the status register returns to 0 (12ms)
    while ((max7456_read(MAX7456_REG_STAT_READ) & MAX7456_STAT_BIT_CHAR_MEM_BUSY) != 0x00);

    delay(20);

    max7456_write(MAX7456_REG_VM0, MAX7456_VM0_BIT_OSD_ENABLE | MAX7456_VM0_BIT_VSYNC_ENABLE | max7456_videoModeMask);

}

void max7456_resetFont(void)
{
    LED0_ON;

    const uint8_t *characterBitmap;

    for(int index = 0; index < 256; index++){
        characterBitmap = &font_max7456_12x18[index * FONT_MAX7456_12x18_BYTES_PER_CHARACTER];

        LED0_TOGGLE;
        max7456_setFontCharacter(index, characterBitmap);
        delay(20);
      }
    LED0_OFF;

}

uint8_t max7456_readStatus(void)
{
    uint8_t result = max7456_read(MAX7456_REG_STAT_READ);

    return result;
}

//#define DEBUG_MAX7456_DM_UPDATE

#ifdef DEBUG_MAX7456_DM_UPDATE
#define MAX7456_TIME_SECTION_END(index) TIME_SECTION_END(index)
#define MAX7456_TIME_SECTION_BEGIN(index) TIME_SECTION_BEGIN(index)
#else
#define MAX7456_TIME_SECTION_END(index) do {} while(0)
#define MAX7456_TIME_SECTION_BEGIN(index) do {} while(0)
#endif

void max7456_writeScreen(textScreen_t *textScreen, char *screenBuffer)
{
    ENABLE_MAX7456;

    spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_REG_DMM);
    spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_DMM_BIT_8BIT_ENABLE | MAX7456_DMM_BIT_AUTO_INCREMENT);

    spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_REG_DMAH); // set start address high
    spiTransferByte(MAX7456_SPI_INSTANCE, 0);

    spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_REG_DMAL); // set start address low
    spiTransferByte(MAX7456_SPI_INSTANCE, 0);

    //
    // It takes about 4600us to transfer the contents of the screen buffer via SPI to the MAX7456
    // (See DEBUG_MAX7456_DM_UPDATE)

    max7456State.vSyncDetected = false;

    for (int y = 0; y < textScreen->height; y++) {
        unsigned int rowOffset = (y * textScreen->width);
        char *buffer = &screenBuffer[rowOffset];

        if (y == 8) {
            MAX7456_TIME_SECTION_END(1);
            // wait for next frame to send the remaining rows
            max7456State.vSyncDetected = false;
        }

        while (!max7456State.vSyncDetected) {
            // Wait for VSYNC pulse and ISR to update the state.
        }

        if (y == 0) {
            MAX7456_TIME_SECTION_BEGIN(1);
        } else if (y == 8) {
            MAX7456_TIME_SECTION_BEGIN(2);
        }

        for (int x = 0; x < textScreen->width; x++) {
            // TODO ensure 0xFF is replaced with ' ' to avoid early termination of auto-increment mode.
            spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_REG_DMDI);
            spiTransferByte(MAX7456_SPI_INSTANCE, *buffer++);
        }
    }

    spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_REG_DMDI);
    spiTransferByte(MAX7456_SPI_INSTANCE, 0xFF); // terminate auto-increment

    spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_REG_DMM);
    spiTransferByte(MAX7456_SPI_INSTANCE, 0x00); // disable 8bit mode and auto increment, enabled above.

    MAX7456_TIME_SECTION_END(2);

    DISABLE_MAX7456;


}

// show the entire font in the middle of the screen.


void max7456_setCharacterAtPosition(uint8_t x, uint8_t y, uint8_t c)
{
    uint32_t linepos;
    uint8_t char_address_hi, char_address_lo;

    //find start address position
    linepos = y * MAX7456_COLUMN_COUNT + x;

    // divide 16 bits into hi & lo uint8_t
    char_address_hi = linepos >> 8;
    char_address_lo = linepos;

    ENABLE_MAX7456;

    spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_REG_DMAH); // set start address high
    spiTransferByte(MAX7456_SPI_INSTANCE, char_address_hi);

    spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_REG_DMAL); // set start address low
    spiTransferByte(MAX7456_SPI_INSTANCE, char_address_lo);

    spiTransferByte(MAX7456_SPI_INSTANCE, MAX7456_REG_DMDI);
    spiTransferByte(MAX7456_SPI_INSTANCE, c);

    DISABLE_MAX7456;
}

void max7456_showFont(void)
{
    int c = 0, y = 2;
    while (c < 256) {
        for (uint8_t x = 0; x < 24 && c < 256; x++) {
            max7456_setCharacterAtPosition(x + 3, y, c++);
        }
        y++;
    }
}

void max7456_clearScreen(void)
{
    uint8_t dmmStatus = max7456_read(MAX7456_REG_DMM_READ);

    dmmStatus |= MAX7456_DMM_BIT_CLEAR;
    max7456_write(MAX7456_REG_DMM, dmmStatus);
    delayMicroseconds(20);
}

void max7456_clearScreenAtNextVSync(void)
{
    uint8_t dmmStatus = max7456_read(MAX7456_REG_DMM_READ);

    dmmStatus |= MAX7456_DMM_BIT_CLEAR;
    dmmStatus |= MAX7456_DMM_BIT_VSYNC_CLEAR;
    max7456_write(MAX7456_REG_DMM, dmmStatus);
}
