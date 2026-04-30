/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/io_impl.h"
#include "platform/rcc.h"

#include "common/utils.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(X32M7)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB5_1(GPIOA) },
    { RCC_AHB5_1(GPIOB) },
    { RCC_AHB5_1(GPIOC) },
    { RCC_AHB5_1(GPIOD) },
    { RCC_AHB5_1(GPIOE) },
    { RCC_AHB5_1(GPIOF) },
    { RCC_AHB5_1(GPIOG) },
    { RCC_AHB5_1(GPIOH) },
    { RCC_AHB5_2(GPIOI) },
    { RCC_AHB5_2(GPIOJ) },
    { RCC_AHB5_2(GPIOK) },
};
#else
# error "IO PortDefs not defined for MCU"
#endif

uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io) {
        return 0;
    }

    return 1 << IO_GPIOPinIdx(io);
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }

    return (IO_GPIO(io)->PID & IO_Pin(io));
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }

    IO_GPIO(io)->PBSC = IO_Pin(io) << (hi ? 0 : 16);
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }

    IO_GPIO(io)->PBSC = IO_Pin(io);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }

    IO_GPIO(io)->PBC = IO_Pin(io);
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }

    IO_GPIO(io)->POD ^= IO_Pin(io);
}

#if defined(X32M7)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    IOConfigGPIOAF(io, cfg, 0);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    GPIO_InitType GPIO_InitStructure;
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin             = IO_Pin(io);
    GPIO_InitStructure.GPIO_Mode       = IO_CONFIG_GET_MODE(cfg);
    GPIO_InitStructure.GPIO_Slew_Rate  = IO_CONFIG_GET_SPEED(cfg);
    GPIO_InitStructure.GPIO_Pull       = IO_CONFIG_GET_PULL(cfg);
    GPIO_InitStructure.GPIO_Current    = IO_CONFIG_GET_DS(cfg);
    GPIO_InitStructure.GPIO_Alternate  = af;
    GPIO_InitPeripheral(IO_GPIO(io), &GPIO_InitStructure);
}

static const struct {
    GPIO_TypeDef* port;
    __IO uint32_t* cfg_reg;
    uint32_t bit[16];
    uint32_t pin[16];
} gpio_anaen_map[] = {
    {GPIOA, &AFIO->ANAEN_CFG0,{AFIO_ANAEN_PA0_C, AFIO_ANAEN_PA0, AFIO_ANAEN_PA1_C, AFIO_ANAEN_PA1,   AFIO_ANAEN_PA2, AFIO_ANAEN_PA3,  AFIO_ANAEN_PA4,  AFIO_ANAEN_PA5, AFIO_ANAEN_PA6, AFIO_ANAEN_PA7}, \
                              {GPIO_PIN_0,       GPIO_PIN_0,     GPIO_PIN_1,       GPIO_PIN_1,       GPIO_PIN_2,     GPIO_PIN_3,      GPIO_PIN_4,      GPIO_PIN_5,     GPIO_PIN_6,     GPIO_PIN_7} },
    {GPIOB, &AFIO->ANAEN_CFG1,{AFIO_ANAEN_PB0 ,  AFIO_ANAEN_PB1 },\
                              {GPIO_PIN_0 ,      GPIO_PIN_1 }  }, 
    {GPIOC, &AFIO->ANAEN_CFG2,{AFIO_ANAEN_PC0,   AFIO_ANAEN_PC1, AFIO_ANAEN_PC2,   AFIO_ANAEN_PC2_C, AFIO_ANAEN_PC3, AFIO_ANAEN_PC3_C, AFIO_ANAEN_PC4, AFIO_ANAEN_PC5, AFIO_ANAEN_PC6, AFIO_ANAEN_PC8},\
                              {GPIO_PIN_0,       GPIO_PIN_1,     GPIO_PIN_2,       GPIO_PIN_2,       GPIO_PIN_3,     GPIO_PIN_3,       GPIO_PIN_4,     GPIO_PIN_5,     GPIO_PIN_6,     GPIO_PIN_8}},
    {GPIOF, &AFIO->ANAEN_CFG3,{AFIO_ANAEN_PF3,   AFIO_ANAEN_PF4, AFIO_ANAEN_PF5,   AFIO_ANAEN_PF6,   AFIO_ANAEN_PF7, AFIO_ANAEN_PF8,   AFIO_ANAEN_PF9, AFIO_ANAEN_PF10, AFIO_ANAEN_PF11, AFIO_ANAEN_PF12, AFIO_ANAEN_PF13, AFIO_ANAEN_PF14},\
                              {GPIO_PIN_3,       GPIO_PIN_4,     GPIO_PIN_5,       GPIO_PIN_6,       GPIO_PIN_7,     GPIO_PIN_8,       GPIO_PIN_9,     GPIO_PIN_10,    GPIO_PIN_11,    GPIO_PIN_12,    GPIO_PIN_13,    GPIO_PIN_14}},
    {GPIOH, &AFIO->ANAEN_CFG4,{AFIO_ANAEN_PH2,   AFIO_ANAEN_PH3, AFIO_ANAEN_PH4,   AFIO_ANAEN_PH5},\
                              {GPIO_PIN_2,       GPIO_PIN_3,     GPIO_PIN_4,       GPIO_PIN_5} },
    {GPIOI, &AFIO->ANAEN_CFG5,{AFIO_ANAEN_PI8,   AFIO_ANAEN_PI15},\
                              {GPIO_PIN_8,       GPIO_PIN_15} },  
    {GPIOJ, &AFIO->ANAEN_CFG6,{AFIO_ANAEN_PJ0,   AFIO_ANAEN_PJ3,   AFIO_ANAEN_PJ4,   AFIO_ANAEN_PJ5,   AFIO_ANAEN_PJ6,   AFIO_ANAEN_PJ7},\
                              {GPIO_PIN_0,       GPIO_PIN_3,     GPIO_PIN_4,       GPIO_PIN_5,       GPIO_PIN_6,     GPIO_PIN_7} },
    {NULL, NULL, {0},{0}}
};

void AFIO_ConfigPinAnalogSignalChannel(IO_t io, FunctionalState cmd)
{
    int i,pin_index;
    for (i = 0; gpio_anaen_map[i].port != NULL; i++) {
        if (gpio_anaen_map[i].port == IO_GPIO(io)) {
            break;
        }
    }
    for (pin_index = 0; pin_index < 16; pin_index++) {
        if( gpio_anaen_map[i].pin[pin_index] == IO_Pin(io)) {
            if (cmd != DISABLE)
                *gpio_anaen_map[i].cfg_reg |= gpio_anaen_map[i].bit[pin_index];
            else
                *gpio_anaen_map[i].cfg_reg &= ~gpio_anaen_map[i].bit[pin_index];
        }
    }
}


#else
# warning MCU not set
#endif
