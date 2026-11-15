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

#include "fc/faults.h"

volatile uint32_t faultCaptureBuf[5] __attribute__((used));

void NMI_Handler(void);
void HardFault_Handler(void);

__FAST_INTERRUPT
void NMI_Handler(void)
{
    while (1) {
    }
}

__FAST_INTERRUPT
void HardFault_Handler(void)
{
    __asm__ volatile(
        "la    t1,   faultCaptureBuf \n"
        "csrr  t0,   mcause          \n"
        "sw    t0,   0(t1)           \n"
        "csrr  t0,   mtval           \n"
        "sw    t0,   4(t1)           \n"
        "csrr  t0,   mepc            \n"
        "sw    t0,   8(t1)           \n"
        "csrr  t0,   mstatus         \n"
        "sw    t0,   12(t1)          \n"
        "mv    t0,   sp              \n"
        "sw    t0,   16(t1)          \n"
        "fence                       \n"
        "fence.i                     \n"
        "j  systemFaultAction        \n"
        ::: "t0", "t1", "memory"
    );
}
