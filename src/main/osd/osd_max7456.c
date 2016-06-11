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

/*
 * Note: this code is very early code that justs gets basic information on the screen.
 *
 * This code is currently dependent on the max7465 chip but that is NOT the final goal.  Display driver abstraction is required.
 * The idea is that basic textual information should be able to be displayed by all OSD video hardware and all hardware layers should
 * translate an in-memory character buffer to the display as required.  In the case of the max7456 chip we will just copy the in-memory display
 * buffer to the max7456 character memory.
 *
 * Later on when the code is more mature support can be added for non-character based display drivers.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <platform.h>
#include "debug.h"

#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/video.h"
#include "drivers/video_textscreen.h"
#include "drivers/video_max7456.h"

#include "osd/config.h"

#include "osd/fonts/font_max7456_12x18.h"

#include "osd/osd.h"

char textScreenBuffer[MAX7456_PAL_CHARACTER_COUNT]; // PAL has more characters than NTSC.
const uint8_t *asciiToFontMapping = &font_max7456_12x18_asciiToFontMapping[0];

#ifdef STM32F303
static const extiConfig_t max7456LOSExtiConfig = {
        .gpioAHBPeripherals = RCC_AHBPeriph_GPIOC,
        .gpioPort = GPIOC,
        .gpioPin = Pin_13,
        .exti_port_source = EXTI_PortSourceGPIOC,
        .exti_pin_source = EXTI_PinSource13,
        .exti_line = EXTI_Line13,
        .exti_irqn = EXTI15_10_IRQn
};

static const extiConfig_t max7456VSYNCExtiConfig = {
        .gpioAHBPeripherals = RCC_AHBPeriph_GPIOC,
        .gpioPort = GPIOC,
        .gpioPin = Pin_14,
        .exti_port_source = EXTI_PortSourceGPIOC,
        .exti_pin_source = EXTI_PinSource14,
        .exti_line = EXTI_Line14,
        .exti_irqn = EXTI15_10_IRQn
};

static const extiConfig_t max7456HSYNCExtiConfig = {
        .gpioAHBPeripherals = RCC_AHBPeriph_GPIOC,
        .gpioPort = GPIOC,
        .gpioPin = Pin_15,
        .exti_port_source = EXTI_PortSourceGPIOC,
        .exti_pin_source = EXTI_PinSource15,
        .exti_line = EXTI_Line15,
        .exti_irqn = EXTI15_10_IRQn
};
#endif

#ifdef STM32F10X
static const extiConfig_t max7456LOSExtiConfig = {
        .gpioAPB2Peripherals = RCC_APB2Periph_GPIOC,
        .gpioPort = GPIOC,
        .gpioPin = Pin_13,
        .exti_port_source = GPIO_PortSourceGPIOC,
        .exti_pin_source = GPIO_PinSource13,
        .exti_line = EXTI_Line13,
        .exti_irqn = EXTI15_10_IRQn
};

static const extiConfig_t max7456VSYNCExtiConfig = {
        .gpioAPB2Peripherals = RCC_APB2Periph_GPIOC,
        .gpioPort = GPIOC,
        .gpioPin = Pin_14,
        .exti_port_source = GPIO_PortSourceGPIOC,
        .exti_pin_source = GPIO_PinSource14,
        .exti_line = EXTI_Line14,
        .exti_irqn = EXTI15_10_IRQn
};

static const extiConfig_t max7456HSYNCExtiConfig = {
        .gpioAPB2Peripherals = RCC_APB2Periph_GPIOC,
        .gpioPort = GPIOC,
        .gpioPin = Pin_15,
        .exti_port_source = GPIO_PortSourceGPIOC,
        .exti_pin_source = GPIO_PinSource15,
        .exti_line = EXTI_Line15,
        .exti_irqn = EXTI15_10_IRQn
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

typedef struct max7456State_s {
    bool los;
    uint32_t losCounter;
    uint32_t frameCounter;
    uint16_t lineCounter;
    uint16_t maxLinesDetected;
} max7456State_t;

max7456State_t max7456State;

void updateLOSState(void)
{
    // "LOS goes high when the VIN sync pulse is lost for 32 consecutive lines. LOS goes low when 32 consecutive valid sync pulses are received."
    uint8_t status = GPIO_ReadInputDataBit(max7456LOSExtiConfig.gpioPort, max7456LOSExtiConfig.gpioPin);

    max7456State.los = status != 0;

    //debug[0] = max7456State.los;
}

void LOS_EXTI_Handler(void)
{
    static uint32_t callCount = 0;
    bool set = max7456ExtiHandler(&max7456LOSExtiConfig);
    callCount++;
    if (!set) {
        return;
    }

    max7456State.losCounter++;

    updateLOSState();
}

void VSYNC_EXTI_Handler(void)
{
    static uint32_t callCount = 0;
    bool set = max7456ExtiHandler(&max7456VSYNCExtiConfig);
    callCount++;
    if (!set) {
        return;
    }
    max7456State.frameCounter++;
    if (max7456State.lineCounter > max7456State.maxLinesDetected) {
        max7456State.maxLinesDetected = max7456State.lineCounter;
    }
    max7456State.lineCounter = 0;


    //debug[1] = max7456State.frameCounter;
    //debug[3] = max7456State.maxLinesDetected;
}

void HSYNC_EXTI_Handler(void)
{
    static uint32_t callCount = 0;
    bool set = max7456ExtiHandler(&max7456HSYNCExtiConfig);
    callCount++;
    if (!set) {
        return;
    }
    max7456State.lineCounter++;

    //debug[2] = max7456State.lineCounter;
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

void osdHardwareApplyConfiguration(void)
{
    max7456_init(osdVideoConfig()->videoMode);

    textScreen_t *max7456TextScreen = max7456_getTextScreen();
    osdSetTextScreen(max7456TextScreen);
}

void osdHardwareInit(void)
{
    LED0_ON;
    delay(500);
    max7456_hardwareReset();
    LED0_OFF;

    osdHardwareApplyConfiguration();

    max7456_extiInit(&max7456LOSExtiConfig, LOS_EXTI_Handler, EXTI_Trigger_Rising_Falling);
    max7456_extiInit(&max7456VSYNCExtiConfig, VSYNC_EXTI_Handler, EXTI_Trigger_Rising);
    max7456_extiInit(&max7456HSYNCExtiConfig, HSYNC_EXTI_Handler, EXTI_Trigger_Rising);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_InitStructure.NVIC_IRQChannel = max7456LOSExtiConfig.exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_OSD_LOS);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_OSD_LOS);
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = max7456VSYNCExtiConfig.exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_OSD_VSYNC);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_OSD_VSYNC);
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = max7456HSYNCExtiConfig.exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_OSD_HSYNC);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_OSD_HSYNC);
    NVIC_Init(&NVIC_InitStructure);

    if (osdFontConfig()->fontVersion != FONT_VERSION) {
        // before
        max7456_showFont();
        delay(5000); // give the user a chance to power off before changing

        max7456_resetFont();

        // after
        max7456_showFont();
        delay(5000); // give the user a chance to power off after changing

        osdFontConfig()->fontVersion = FONT_VERSION;
        writeEEPROM();

        max7456_clearScreen();
    }
}

void osdHardwareUpdate(void)
{
#if 0
    debug[3] = max7456_readStatus();
#endif

    max7456_writeScreen(&osdTextScreen, textScreenBuffer);
}

void osdHardwareCheck(void)
{
    static int checkCount = 0;

    checkCount++;

    if (!max7456_isOSDEnabled()) {
        max7456_init(osdVideoConfig()->videoMode);
    }

#ifdef FACTORY_TEST
    if (checkCount == 10) {
        max7456_init(osdVideoConfig()->videoMode);
    }
#endif

    updateLOSState();
}

void osdHardwareDrawLogo(void)
{
    osdSetRawCharacterAtPosition(14, 5, FONT_CHARACTER_CF_LOGO_W3xH2__1x1);
    osdSetRawCharacterAtPosition(15, 5, FONT_CHARACTER_CF_LOGO_W3xH2__1x2);
    osdSetRawCharacterAtPosition(16, 5, FONT_CHARACTER_CF_LOGO_W3xH2__1x3);
    osdSetRawCharacterAtPosition(14, 6, FONT_CHARACTER_CF_LOGO_W3xH2__2x1);
    osdSetRawCharacterAtPosition(15, 6, FONT_CHARACTER_CF_LOGO_W3xH2__2x2);
    osdSetRawCharacterAtPosition(16, 6, FONT_CHARACTER_CF_LOGO_W3xH2__2x3);
}

void osdHardwareDisplayMotor(uint8_t x, uint8_t y, uint8_t percent)
{
    uint8_t c = FONT_CHARACTER_MOTOR_OFF - (MIN(percent, 99) / 10);

    osdSetRawCharacterAtPosition(13 + x, osdTextScreen.height - 4 + y, c);
}

bool osdIsCameraConnected(void)
{
    return !max7456State.los;
}
