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

#include "platform.h"

#ifdef USE_RTC6705

#include "drivers/bus_spi.h"
#include "drivers/system.h"
#include "drivers/gpio.h"

#include "vtx_soft_spi_rtc6705.h"

#define RTC6705_SPICLK_ON        GPIO_SetBits(RTC6705_SPICLK_GPIO,   RTC6705_SPICLK_PIN)
#define RTC6705_SPICLK_OFF       GPIO_ResetBits(RTC6705_SPICLK_GPIO, RTC6705_SPICLK_PIN)

#define RTC6705_SPIDATA_ON       GPIO_SetBits(RTC6705_SPIDATA_GPIO,   RTC6705_SPIDATA_PIN)
#define RTC6705_SPIDATA_OFF      GPIO_ResetBits(RTC6705_SPIDATA_GPIO, RTC6705_SPIDATA_PIN)

#define RTC6705_SPILE_ON         GPIO_SetBits(RTC6705_SPILE_GPIO,   RTC6705_SPILE_PIN)
#define RTC6705_SPILE_OFF        GPIO_ResetBits(RTC6705_SPILE_GPIO, RTC6705_SPILE_PIN)

char *vtx_bands[] = {
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
};

uint16_t vtx_freq[] =
{
    5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725,
    5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866,
    5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945,
    5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880,
    5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917,
};

uint16_t current_vtx_channel;

void rtc6705_soft_spi_init(void) {
    gpio_config_t gpio;

#ifdef STM32F303
    #ifdef RTC6705_SPICLK_PERIPHERAL
    RCC_AHBPeriphClockCmd(RTC6705_SPICLK_PERIPHERAL, ENABLE);
    #endif
    #ifdef RTC6705_SPILE_PERIPHERAL
    RCC_AHBPeriphClockCmd(RTC6705_SPILE_PERIPHERAL, ENABLE);
    #endif
    #ifdef RTC6705_SPIDATA_PERIPHERAL
    RCC_AHBPeriphClockCmd(RTC6705_SPIDATA_PERIPHERAL, ENABLE);
    #endif
#endif
#ifdef STM32F10X
    #ifdef RTC6705_SPICLK_PERIPHERAL
    RCC_APB2PeriphClockCmd(RTC6705_SPICLK_PERIPHERAL, ENABLE);
    #endif
    #ifdef RTC6705_SPILE_PERIPHERAL
    RCC_APB2PeriphClockCmd(RTC6705_SPILE_PERIPHERAL, ENABLE);
    #endif
    #ifdef RTC6705_SPIDATA_PERIPHERAL
    RCC_APB2PeriphClockCmd(RTC6705_SPIDATA_PERIPHERAL, ENABLE);
    #endif
#endif

    gpio.pin = RTC6705_SPICLK_PIN;
    gpio.speed = Speed_50MHz;
    gpio.mode = Mode_Out_PP;
    gpioInit(RTC6705_SPICLK_GPIO, &gpio);

    gpio.pin = RTC6705_SPILE_PIN;
    gpio.speed = Speed_50MHz;
    gpio.mode = Mode_Out_PP;
    gpioInit(RTC6705_SPILE_GPIO, &gpio);

    gpio.pin = RTC6705_SPIDATA_PIN;
    gpio.speed = Speed_50MHz;
    gpio.mode = Mode_Out_PP;
    gpioInit(RTC6705_SPIDATA_GPIO, &gpio);
}

static void rtc6705_write_register(uint8_t addr, uint32_t data) {
    uint8_t i;

    RTC6705_SPILE_OFF;
    delay(1);
    // send address
    for (i=0; i<4; i++) {
        if ((addr >> i) & 1)
            RTC6705_SPIDATA_ON;
        else
            RTC6705_SPIDATA_OFF;

        RTC6705_SPICLK_ON;
        delay(1);
        RTC6705_SPICLK_OFF;
        delay(1);
    }
    // Write bit

    RTC6705_SPIDATA_ON;
    RTC6705_SPICLK_ON;
    delay(1);
    RTC6705_SPICLK_OFF;
    delay(1);
    for (i=0; i<20; i++) {
        if ((data >> i) & 1)
            RTC6705_SPIDATA_ON;
        else
            RTC6705_SPIDATA_OFF;
        RTC6705_SPICLK_ON;
        delay(1);
        RTC6705_SPICLK_OFF;
        delay(1);
    }
    RTC6705_SPILE_ON;
}


void rtc6705_soft_spi_set_channel(uint16_t channel_freq) {

    uint32_t freq = (uint32_t)channel_freq * 1000;
    uint32_t N, A;

    freq /= 40;
    N = freq / 64;
    A = freq % 64;
    rtc6705_write_register(0, 400);
    rtc6705_write_register(1, (N << 7) | A);
}
#endif
