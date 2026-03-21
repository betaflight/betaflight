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

#include <stdint.h>
#include <string.h>
#include <reent.h>

#include "platform.h"

#include "common/time.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/systimer_ll.h"
#pragma GCC diagnostic pop
#include "esp_rom_sys.h"
#include "soc/systimer_struct.h"
#include "soc/efuse_reg.h"
#include "soc/soc.h"

// ESP32-S3 runs at 240 MHz by default
uint32_t SystemCoreClock = 240000000;

// Peripheral instance storage (port numbers for ESP-IDF)
esp32_peripheral_t esp32SpiDev0 = 0;
esp32_peripheral_t esp32SpiDev1 = 1;
esp32_peripheral_t esp32I2cDev0 = 0;
esp32_peripheral_t esp32I2cDev1 = 1;
esp32_peripheral_t esp32UartDev0 = 0;
esp32_peripheral_t esp32UartDev1 = 1;
esp32_peripheral_t esp32UartDev2 = 2;

uint32_t systemUniqueId[3] = { 0 };

static uint32_t usTicks = 0;
static float usTicksInv = 0.0f;

// ESP32-S3 systimer runs at 16 MHz (XTAL_CLK), each tick = 62.5 ns
#define SYSTIMER_TICKS_PER_US  16

void cycleCounterInit(void)
{
    usTicks = SystemCoreClock / 1000000;
    usTicksInv = 1e6f / SystemCoreClock;
}

void systemInit(void)
{
    cycleCounterInit();

    // Initialize systimer - should already be running from ROM bootloader,
    // but enable clock and counter 0 to be safe
    systimer_ll_enable_clock(&SYSTIMER, true);
    systimer_ll_enable_counter(&SYSTIMER, 0, true);

    // Read 6-byte MAC address from eFuse into systemUniqueId
    // REG0 contains MAC bytes [0..3] (low 32 bits), REG1[15:0] contains MAC bytes [4..5]
    uint32_t mac0 = REG_READ(EFUSE_RD_MAC_SPI_SYS_0_REG);
    uint32_t mac1 = REG_READ(EFUSE_RD_MAC_SPI_SYS_1_REG);
    systemUniqueId[0] = mac0;
    systemUniqueId[1] = mac1 & 0xFFFF;
    systemUniqueId[2] = 0;
}

void systemReset(void)
{
    esp_rom_software_reset_system();
    while (1);  // should not be reached
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);
    systemReset();
}

STATIC_ASSERT(sizeof(timeMs_t) == sizeof(uint32_t), timeMs_t_is_32_bit_failed);
STATIC_ASSERT(sizeof(timeUs_t) == sizeof(uint32_t), timeUs_t_is_32_bit_failed);

timeUs_t micros(void)
{
    // Take a snapshot of counter unit 0
    systimer_ll_counter_snapshot(&SYSTIMER, 0);

    // Wait for snapshot to be valid
    while (!systimer_ll_is_counter_value_valid(&SYSTIMER, 0)) {
    }

    // Read the 52-bit counter value
    uint32_t lo = systimer_ll_get_counter_value_low(&SYSTIMER, 0);
    uint32_t hi = systimer_ll_get_counter_value_high(&SYSTIMER, 0);
    uint64_t ticks = ((uint64_t)hi << 32) | lo;

    // Convert 16 MHz ticks to microseconds
    return (timeUs_t)(ticks / SYSTIMER_TICKS_PER_US);
}

timeMs_t millis(void)
{
    return micros() / 1000;
}

timeUs_t microsISR(void)
{
    return micros();
}

void delayMicroseconds(uint32_t us)
{
    esp_rom_delay_us(us);
}

void delay(uint32_t ms)
{
    esp_rom_delay_us(ms * 1000);
}

uint32_t getCycleCounter(void)
{
    uint32_t val;
    __asm__ __volatile__("rsr.ccount %0" : "=r"(val));
    return val;
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / usTicks;
}

float clockCyclesToMicrosf(int32_t clockCycles)
{
    return clockCycles * usTicksInv;
}

int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return 10 * clockCycles / (int32_t)usTicks;
}

int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return 100 * clockCycles / (int32_t)usTicks;
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * usTicks;
}

static void indicate(uint8_t count, uint16_t duration)
{
    if (count) {
        LED1_ON;
        LED0_OFF;

        while (count--) {
            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_ON;
            delay(duration);

            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_OFF;
            delay(duration);
        }
    }
}

void indicateFailure(failureMode_e mode, int codeRepeatsRemaining)
{
    while (codeRepeatsRemaining--) {
        indicate(WARNING_FLASH_COUNT, WARNING_FLASH_DURATION_MS);

        delay(WARNING_PAUSE_DURATION_MS);

        indicate(mode + 1, WARNING_CODE_DURATION_LONG_MS);

        delay(1000);
    }
}

void failureMode(failureMode_e mode)
{
    indicateFailure(mode, 10);
    systemReset();
}

void unusedPinsInit(void)
{
    // NOOP for now
}

// Newlib reentrant stub — single-threaded, no FreeRTOS
static struct _reent s_reent;

struct _reent *__getreent(void)
{
    return &s_reent;
}

// Newlib POSIX signal stubs — suppress linker warnings from libnosys
__attribute__((used, externally_visible)) int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    return -1;
}

__attribute__((used, externally_visible)) int _getpid(void)
{
    return 1;
}

const mcuTypeInfo_t *getMcuTypeInfo(void)
{
    static const mcuTypeInfo_t info = {
        .id = MCU_TYPE_ESP32S3, .name = "ESP32S3"
    };
    return &info;
}
