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

#pragma once

#include <stdint.h>

// CPU interrupt number assignments.  ESP32-S3 C handlers must use external,
// level-triggered lines at a maskable C-compatible level.  Keep this mapping
// S3-specific: the other ESP variants have different interrupt controllers and
// retain their existing bring-up assignments below.
#if defined(ESP32S3)
#define ESP32_CPU_INTR_GPIO      0
#define ESP32_CPU_INTR_UART0     1
#define ESP32_CPU_INTR_UART1     2
#define ESP32_CPU_INTR_UART2     3
#define ESP32_CPU_INTR_DMA_CH0   4
#define ESP32_CPU_INTR_DMA_CH1   5
#define ESP32_CPU_INTR_DMA_CH2   8
#define ESP32_CPU_INTR_DMA_CH3   9
#define ESP32_CPU_INTR_DMA_CH4  12
#define ESP32_CPU_INTR_USB      13
#else
#define ESP32_CPU_INTR_GPIO     19
#define ESP32_CPU_INTR_UART0    20
#define ESP32_CPU_INTR_UART1    21
#define ESP32_CPU_INTR_UART2    23
#define ESP32_CPU_INTR_DMA_CH0  24
#define ESP32_CPU_INTR_DMA_CH1  25
#define ESP32_CPU_INTR_DMA_CH2  26
#define ESP32_CPU_INTR_DMA_CH3  27
#define ESP32_CPU_INTR_DMA_CH4  28
#define ESP32_CPU_INTR_USB      29
#endif

typedef void (*esp32IsrHandler_t)(void *arg);

// Route a peripheral interrupt source to a CPU interrupt line
void esp32IntrRoute(int cpuIntrNum, int peripheralSource);

// Register an ISR handler for a CPU interrupt line
void esp32IntrRegister(int cpuIntrNum, esp32IsrHandler_t handler, void *arg);

// Enable a CPU interrupt line
void esp32IntrEnable(int cpuIntrNum);

// Disable a CPU interrupt line
void esp32IntrDisable(int cpuIntrNum);
