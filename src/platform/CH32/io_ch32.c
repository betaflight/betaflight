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
/*
 * porting for ch32h41x by Temperslee
 */
#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "platform/rcc.h"
#include "platform/io_impl.h"
#include "common/utils.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

const struct ioPortDef_s ioPortDefs[] = {
    { RCC_HB2(GPIOA) },
    { RCC_HB2(GPIOB) },
    { RCC_HB2(GPIOC) },
    { RCC_HB2(GPIOD) },
    { RCC_HB2(GPIOE) },
    { RCC_HB2(GPIOF) }
};

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
    return (IO_GPIO(io)->INDR & IO_Pin(io));
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->BSHR = IO_Pin(io) << (hi ? 0 : 16); 
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->BSHR = IO_Pin(io);
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
    IO_GPIO(io)->BCR = IO_Pin(io);
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }

    uint32_t mask = IO_Pin(io);

    if (IO_GPIO(io)->OUTDR & mask) {
        mask <<= 16; // bit is set, shift mask to reset half
    }
    IO_GPIO(io)->BSHR = mask;
}

/* 
    We expect that during the dshot input and output switching process, 
    it will be as fast as possible. 
*/
__attribute__((optimize("Ofast")))
void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }

    //all clock have been configed
    // const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    // RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init;

//  bit 0   1      2   3      4    5       6  7  
//  DIR[1:0]   MODE[1:0]   SPEED[1:0]   PD=0,PU=1  
// #define IO_CONFIG(dir, mode, speed, pupd) ((dir) | ((mode) << 2) | ((speed) << 4) | ((pupd) << 6))
    init.GPIO_Pin = IO_Pin(io);
    init.GPIO_Speed = GPIO_Speed_Very_High;

    if(cfg & 0x3) //output 
    {
        if(((cfg >> 2) & 0x3) == GPIO_MODE_OUT_PP) init.GPIO_Mode = GPIO_Mode_Out_PP;
        else if(((cfg >> 2) & 0x3) == GPIO_MODE_OUT_OD) init.GPIO_Mode = GPIO_Mode_Out_OD;
        else if(((cfg >> 2) & 0x3) == GPIO_MODE_OUT_AF_PP)  init.GPIO_Mode = GPIO_Mode_AF_PP;
        else if(((cfg >> 2) & 0x3) == GPIO_MODE_OUT_AF_OD)  init.GPIO_Mode = GPIO_Mode_AF_OD;
        else init.GPIO_Mode = GPIO_Mode_Out_PP;
    }
    else //input
    { 
        if(((cfg >> 2) & 0x3) == GPIO_MODE_IN_AN)  init.GPIO_Mode = GPIO_Mode_AIN;
        else if(((cfg >> 2) & 0x3) == GPIO_MODE_IN_FLOAT) init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        else if(((cfg >> 2) & 0x3) == GPIO_MODE_IN_PULL)
        {
             if(((cfg >> 6) & 0x3) == GPIO_PULL_DOWN) init.GPIO_Mode = GPIO_Mode_IPD;
             else if(((cfg >> 6) & 0x3) == GPIO_PULL_UP) init.GPIO_Mode = GPIO_Mode_IPU;
             else  init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        }
    }
    GPIO_Init(IO_GPIO(io), &init);
}

__attribute__((optimize("Ofast")))
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }
    IOConfigGPIO(io, cfg);
    // gpio_pin_mux_config(IO_GPIO(io), IO_GPIO_PinSource(io), af);
    GPIO_PinAFConfig(IO_GPIO(io), IO_GPIO_PinSource(io), af);
}
