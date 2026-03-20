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

#include "platform.h"

#include "common/time.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"

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

// Stub: will use ESP-IDF esp_timer / CCOUNT register
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickValStamp = 0;

void cycleCounterInit(void)
{
    usTicks = SystemCoreClock / 1000000;
    usTicksInv = 1e6f / SystemCoreClock;
}

void systemInit(void)
{
    cycleCounterInit();

    // TODO: read ESP32 MAC address into systemUniqueId via esp_efuse_mac_get_default()
    systemUniqueId[0] = 0x00E53200;
    systemUniqueId[1] = 0x00000053;
    systemUniqueId[2] = 0x00000001;
}

void systemReset(void)
{
    // TODO: esp_restart()
    while (1);
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);
    systemReset();
}

STATIC_ASSERT(sizeof(timeMs_t) == sizeof(uint32_t), timeMs_t_is_32_bit_failed);
STATIC_ASSERT(sizeof(timeUs_t) == sizeof(uint32_t), timeUs_t_is_32_bit_failed);

timeMs_t millis(void)
{
    // TODO: use esp_timer_get_time() / 1000
    return sysTickUptime;
}

timeUs_t micros(void)
{
    // TODO: use esp_timer_get_time()
    return sysTickUptime * 1000;
}

timeUs_t microsISR(void)
{
    return micros();
}

void delayMicroseconds(uint32_t us)
{
    // TODO: use esp_rom_delay_us()
    UNUSED(us);
}

void delay(uint32_t ms)
{
    // TODO: use vTaskDelay() or busy loop
    UNUSED(ms);
}

uint32_t getCycleCounter(void)
{
    // TODO: use CCOUNT register via __asm__ __volatile__("rsr.ccount %0" : "=r"(val))
    return 0;
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

const mcuTypeInfo_t *getMcuTypeInfo(void)
{
    static const mcuTypeInfo_t info = {
        .id = MCU_TYPE_ESP32S3, .name = "ESP32S3"
    };
    return &info;
}
