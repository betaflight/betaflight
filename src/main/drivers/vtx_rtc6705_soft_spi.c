/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_VTX_RTC6705) && defined(USE_VTX_RTC6705_SOFTSPI)

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/time.h"
#include "drivers/vtx_rtc6705.h"

#define DP_5G_MASK                  0x7000
#define PA5G_BS_MASK                0x0E00
#define PA5G_PW_MASK                0x0180
#define PD_Q5G_MASK                 0x0040
#define QI_5G_MASK                  0x0038
#define PA_BS_MASK                  0x0007

#define PA_CONTROL_DEFAULT          0x4FBD

#define RTC6705_SPICLK_ON     IOHi(rtc6705ClkPin)
#define RTC6705_SPICLK_OFF    IOLo(rtc6705ClkPin)

#define RTC6705_SPIDATA_ON    IOHi(rtc6705DataPin)
#define RTC6705_SPIDATA_OFF   IOLo(rtc6705DataPin)

#define DISABLE_RTC6705       IOHi(rtc6705CsnPin)
#define ENABLE_RTC6705        IOLo(rtc6705CsnPin)

#ifdef RTC6705_POWER_PIN
static IO_t vtxPowerPin     = IO_NONE;
#endif

#define ENABLE_VTX_POWER()          IOLo(vtxPowerPin)
#define DISABLE_VTX_POWER()         IOHi(vtxPowerPin)

static IO_t rtc6705DataPin = IO_NONE;
static IO_t rtc6705CsnPin = IO_NONE;
static IO_t rtc6705ClkPin = IO_NONE;

void rtc6705IOInit(void)
{
#ifdef RTC6705_POWER_PIN
    vtxPowerPin = IOGetByTag(IO_TAG(RTC6705_POWER_PIN));
    IOInit(vtxPowerPin, OWNER_VTX, 0);

    DISABLE_VTX_POWER();
    IOConfigGPIO(vtxPowerPin, IOCFG_OUT_PP);
#endif

    rtc6705DataPin = IOGetByTag(IO_TAG(RTC6705_SPI_MOSI_PIN));
    rtc6705CsnPin  = IOGetByTag(IO_TAG(RTC6705_CS_PIN));
    rtc6705ClkPin  = IOGetByTag(IO_TAG(RTC6705_SPICLK_PIN));

    IOInit(rtc6705DataPin, OWNER_SPI_MOSI, RESOURCE_SOFT_OFFSET);
    IOConfigGPIO(rtc6705DataPin, IOCFG_OUT_PP);

    IOInit(rtc6705ClkPin, OWNER_SPI_SCK, RESOURCE_SOFT_OFFSET);
    IOConfigGPIO(rtc6705ClkPin, IOCFG_OUT_PP);

    // Important: The order of GPIO configuration calls are critical to ensure that incorrect signals are not briefly sent to the VTX.
    // GPIO bit is enabled so here so the CS/LE pin output is not pulled low when the GPIO is set in output mode.
    DISABLE_RTC6705;
    IOInit(rtc6705CsnPin, OWNER_SPI_CS, RESOURCE_SOFT_OFFSET);
    IOConfigGPIO(rtc6705CsnPin, IOCFG_OUT_PP);
}

static void rtc6705_write_register(uint8_t addr, uint32_t data)
{
    ENABLE_RTC6705;
    delay(1);
    // send address
    for (int i = 0; i < 4; i++) {
        if ((addr >> i) & 1) {
            RTC6705_SPIDATA_ON;
        } else {
            RTC6705_SPIDATA_OFF;
        }

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
    for (int i = 0; i < 20; i++) {
        if ((data >> i) & 1) {
            RTC6705_SPIDATA_ON;
        } else {
            RTC6705_SPIDATA_OFF;
        }
        RTC6705_SPICLK_ON;
        delay(1);
        RTC6705_SPICLK_OFF;
        delay(1);
    }
    DISABLE_RTC6705;
}

void rtc6705SetFrequency(uint16_t channel_freq)
{
    uint32_t freq = (uint32_t)channel_freq * 1000;
    freq /= 40;
    const uint32_t N = freq / 64;
    const uint32_t A = freq % 64;
    rtc6705_write_register(0, 400);
    rtc6705_write_register(1, (N << 7) | A);
}

void rtc6705SetRFPower(uint8_t rf_power)
{
    rtc6705_write_register(7, (rf_power > 1 ? PA_CONTROL_DEFAULT : (PA_CONTROL_DEFAULT | PD_Q5G_MASK) & (~(PA5G_PW_MASK | PA5G_BS_MASK))));
}

void rtc6705Disable(void)
{
#ifdef RTC6705_POWER_PIN
    DISABLE_VTX_POWER();
#endif
}

void rtc6705Enable(void)
{
#ifdef RTC6705_POWER_PIN
    ENABLE_VTX_POWER();
#endif
}


#endif
