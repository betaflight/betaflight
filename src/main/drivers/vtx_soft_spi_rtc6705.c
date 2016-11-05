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

#ifdef USE_RTC6705

#include "bus_spi.h"
#include "io.h"
#include "system.h"
#include "light_led.h"

#include "vtx_soft_spi_rtc6705.h"

#define RTC6705_SPICLK_ON     IOHi(rtc6705ClkPin)
#define RTC6705_SPICLK_OFF    IOLo(rtc6705ClkPin)

#define RTC6705_SPIDATA_ON    IOHi(rtc6705DataPin)
#define RTC6705_SPIDATA_OFF   IOLo(rtc6705DataPin)

#define RTC6705_SPILE_ON      IOHi(rtc6705LePin)
#define RTC6705_SPILE_OFF     IOLo(rtc6705LePin)

const uint16_t vtx_freq[] =
{
    5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Boacam A
    5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Boscam B
    5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Boscam E
    5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // FatShark
    5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, // RaceBand
};

uint16_t current_vtx_channel;

static IO_t rtc6705DataPin = IO_NONE;
static IO_t rtc6705LePin = IO_NONE;
static IO_t rtc6705ClkPin = IO_NONE;

void rtc6705_soft_spi_init(void)
{
    rtc6705DataPin = IOGetByTag(IO_TAG(RTC6705_SPIDATA_PIN));
    rtc6705LePin   = IOGetByTag(IO_TAG(RTC6705_SPILE_PIN));
    rtc6705ClkPin  = IOGetByTag(IO_TAG(RTC6705_SPICLK_PIN));

    IOInit(rtc6705DataPin, OWNER_SPI_MOSI, RESOURCE_SOFT_OFFSET);
    IOConfigGPIO(rtc6705DataPin, IOCFG_OUT_PP);

    IOInit(rtc6705LePin, OWNER_SPI_CS, RESOURCE_SOFT_OFFSET);
    IOConfigGPIO(rtc6705LePin, IOCFG_OUT_PP);

    IOInit(rtc6705ClkPin, OWNER_SPI_SCK, RESOURCE_SOFT_OFFSET);
    IOConfigGPIO(rtc6705ClkPin, IOCFG_OUT_PP);
}

static void rtc6705_write_register(uint8_t addr, uint32_t data)
{
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


void rtc6705_soft_spi_set_channel(uint16_t channel_freq)
{

    uint32_t freq = (uint32_t)channel_freq * 1000;
    uint32_t N, A;

    freq /= 40;
    N = freq / 64;
    A = freq % 64;
    rtc6705_write_register(0, 400);
    rtc6705_write_register(1, (N << 7) | A);
}

void rtc6705_soft_spi_set_rf_power(uint8_t reduce_power)
{
    rtc6705_write_register(7, (reduce_power ? (PA_CONTROL_DEFAULT | PD_Q5G_MASK) & (~(PA5G_PW_MASK | PA5G_BS_MASK)) : PA_CONTROL_DEFAULT));
}

#endif
