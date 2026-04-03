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

#include "platform.h"
#include "platform/interrupt.h"

#include "esp_rom_sys.h"

// ROM functions for interrupt management.
// Symbols provided by the ESP-IDF ROM linker scripts referenced in ESP32S3.mk
// (components/esp_rom/esp32s3/ld/esp32s3.rom.ld).
typedef void (*ets_isr_t)(void *);
extern void ets_isr_attach(int i, ets_isr_t func, void *arg);
extern void ets_isr_mask(uint32_t mask);
extern void ets_isr_unmask(uint32_t unmask);

void esp32IntrRoute(int cpuIntrNum, int peripheralSource)
{
    esp_rom_route_intr_matrix(0, peripheralSource, cpuIntrNum);
}

void esp32IntrRegister(int cpuIntrNum, esp32IsrHandler_t handler, void *arg)
{
    ets_isr_attach(cpuIntrNum, handler, arg);
}

void esp32IntrEnable(int cpuIntrNum)
{
    ets_isr_unmask(1U << cpuIntrNum);
}

void esp32IntrDisable(int cpuIntrNum)
{
    ets_isr_mask(1U << cpuIntrNum);
}
