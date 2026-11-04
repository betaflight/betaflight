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
 * Hard fault capture. MemManage, BusFault and UsageFault escalate to HardFault.
 *
 * faultCaptureBuf is in DTCM. Read it over SWD with the .elf: x/9xw &faultCaptureBuf
 *   [0] 1, written last, so a zero buffer means no fault since the last reset
 *   [1] CFSR   0xE000ED28, which configurable fault fired
 *   [2] HFSR   0xE000ED2C, bit 30 set means an escalated configurable fault
 *   [3] BFAR   0xE000ED38, valid only while CFSR bit 15 (BFARVALID) is set
 *   [4] MMFAR  0xE000ED34, valid only while CFSR bit 7 (MMARVALID) is set
 *   [5] stacked PC, the faulting instruction
 *   [6] stacked LR
 *   [7] stack pointer at exception entry: r0-r3, r12, LR, PC, xPSR at 0 to 28
 *   [8] stacked xPSR, whose low bits name the exception that was running
 *
 * systemFaultAction() calls code in flash, so after a fault during a flash write
 * it hangs with the motors still running.
 */

#include <stdint.h>

#include "platform.h"

#include "fc/faults.h"

static volatile uint32_t faultCaptureBuf[9] __attribute__((used));

// Naked, so no prologue runs before the frame is read, and in ITCM with its
// literal pool so it runs during a flash write.
FAST_CODE __attribute__((naked, used)) void HardFault_Handler(void)
{
    __asm__ volatile (
        "ldr   r2, =faultCaptureBuf \n"
        "ldr   r3, =0xE000ED28      \n"     // CFSR
        "ldr   r3, [r3]             \n"
        "str   r3, [r2, #4]         \n"
        "ldr   r3, =0xE000ED2C      \n"     // HFSR
        "ldr   r3, [r3]             \n"
        "str   r3, [r2, #8]         \n"
        "ldr   r3, =0xE000ED38      \n"     // BFAR
        "ldr   r3, [r3]             \n"
        "str   r3, [r2, #12]        \n"
        "ldr   r3, =0xE000ED34      \n"     // MMFAR
        "ldr   r3, [r3]             \n"
        "str   r3, [r2, #16]        \n"
        "tst   lr, #4               \n"     // EXC_RETURN bit 2: which stack
        "ite   eq                   \n"
        "mrseq r0, msp              \n"
        "mrsne r0, psp              \n"
        "ldr   r3, [r0, #24]        \n"     // stacked PC
        "str   r3, [r2, #20]        \n"
        "ldr   r3, [r0, #20]        \n"     // stacked LR
        "str   r3, [r2, #24]        \n"
        "str   r0, [r2, #28]        \n"     // the faulting stack pointer
        "ldr   r3, [r0, #28]        \n"     // stacked xPSR
        "str   r3, [r2, #32]        \n"
        "movs  r3, #1               \n"     // last, so the flag means all of it
        "str   r3, [r2, #0]         \n"
        "dsb                        \n"
        "isb                        \n"
        "b     systemFaultAction    \n"
        ".ltorg                     \n"
        ::: "r0", "r2", "r3", "memory"
    );
}
