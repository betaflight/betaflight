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

#include "platform.h"

#include "drivers/light_led.h"

#include "fc/faults.h"

#if defined(STM32C5)
/* C5 fault frame capture: each configurable-fault handler writes its
 * flag + CFSR/HFSR/BFAR/MMFAR/stacked-PC/LR/SP to faultCaptureBuf,
 * then tail-branches to systemFaultAction. systemInit enables the
 * MemManage/BusFault/UsageFault handlers via SHCSR so the original
 * fault doesn't escalate to HardFault with CFSR cleared.
 *
 * Buffer layout (read via SWD after halting in the blink loop):
 *   [0] flag (1=HardFault, 2=MemManage, 3=BusFault, 4=UsageFault)
 *   [1] CFSR
 *   [2] HFSR
 *   [3] BFAR
 *   [4] MMFAR
 *   [5] stacked PC
 *   [6] stacked LR
 *   [7] faulting SP
 *
 * Naked so no prologue/epilogue clobbers the fault frame. Trailing
 * `.ltorg` forces a local literal pool — without it, the four
 * handlers share one pool that exceeds the PC-relative range.
 *
 * Compiled unconditionally on C5 (independent of DEBUG_HARDFAULTS): the
 * SHCSR enables in systemInit need a definition for the configurable
 * handlers or those exceptions vector to the weak Default_Handler. */
volatile uint32_t faultCaptureBuf[8] __attribute__((used));

#define FAULT_HANDLER_ASM(FLAG) \
    __asm__ volatile (                                    \
        "ldr   r2, =faultCaptureBuf \n"                   \
        "movs  r3, #" #FLAG "       \n"                   \
        "str   r3, [r2, #0]         \n"                   \
        "ldr   r3, =0xE000ED28      \n"                   \
        "ldr   r3, [r3]             \n"                   \
        "str   r3, [r2, #4]         \n"                   \
        "ldr   r3, =0xE000ED2C      \n"                   \
        "ldr   r3, [r3]             \n"                   \
        "str   r3, [r2, #8]         \n"                   \
        "ldr   r3, =0xE000ED38      \n"                   \
        "ldr   r3, [r3]             \n"                   \
        "str   r3, [r2, #12]        \n"                   \
        "ldr   r3, =0xE000ED34      \n"                   \
        "ldr   r3, [r3]             \n"                   \
        "str   r3, [r2, #16]        \n"                   \
        "tst   lr, #4               \n"                   \
        "ite   eq                   \n"                   \
        "mrseq r0, msp              \n"                   \
        "mrsne r0, psp              \n"                   \
        "ldr   r3, [r0, #24]        \n"                   \
        "str   r3, [r2, #20]        \n"                   \
        "ldr   r3, [r0, #20]        \n"                   \
        "str   r3, [r2, #24]        \n"                   \
        "str   r0, [r2, #28]        \n"                   \
        "dsb                        \n"                   \
        "isb                        \n"                   \
        "b     systemFaultAction    \n"                   \
        ".ltorg                     \n"                   \
        ::: "r0", "r2", "r3", "memory"                    \
    )

__attribute__((naked, used)) void MemManage_Handler(void)
{
    FAULT_HANDLER_ASM(2);
}

__attribute__((naked, used)) void BusFault_Handler(void)
{
    FAULT_HANDLER_ASM(3);
}

__attribute__((naked, used)) void UsageFault_Handler(void)
{
    FAULT_HANDLER_ASM(4);
}

__attribute__((naked, used)) void HardFault_Handler(void)
{
    FAULT_HANDLER_ASM(1);
}

#elif defined(DEBUG_HARDFAULTS)
// from: https://mcuoneclipse.com/2012/11/24/debugging-hard-faults-on-arm-cortex-m/
// Reads the fault stack frame and SCB fault registers into volatile locals so
// they are visible to a connected debugger. Requires a platform-provided
// HardFault_HandlerAsm trampoline that tail-calls into this function with r0
// pointing at the pre-fault stack frame.
void hard_fault_handler_c(unsigned long *hardfault_args)
{
  volatile unsigned long stacked_r0 ;
  volatile unsigned long stacked_r1 ;
  volatile unsigned long stacked_r2 ;
  volatile unsigned long stacked_r3 ;
  volatile unsigned long stacked_r12 ;
  volatile unsigned long stacked_lr ;
  volatile unsigned long stacked_pc ;
  volatile unsigned long stacked_psr ;
  volatile unsigned long _CFSR ;
  volatile unsigned long _HFSR ;
  volatile unsigned long _DFSR ;
  volatile unsigned long _AFSR ;
  volatile unsigned long _BFAR ;
  volatile unsigned long _MMAR ;

  stacked_r0 = ((unsigned long)hardfault_args[0]) ;
  stacked_r1 = ((unsigned long)hardfault_args[1]) ;
  stacked_r2 = ((unsigned long)hardfault_args[2]) ;
  stacked_r3 = ((unsigned long)hardfault_args[3]) ;
  stacked_r12 = ((unsigned long)hardfault_args[4]) ;
  stacked_lr = ((unsigned long)hardfault_args[5]) ;
  stacked_pc = ((unsigned long)hardfault_args[6]) ;
  stacked_psr = ((unsigned long)hardfault_args[7]) ;

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

  // Blink LEDs to indicate hard fault visually
  LED0_OFF;
  LED1_OFF;

  while (1) {
      // Simple busy-wait delay (approximate heartbeat)
      for (volatile int i = 0; i < 2000000; i++);
      LED0_TOGGLE;
      LED1_TOGGLE;
  }
}

#else

__attribute__((naked, used)) void HardFault_Handler(void)
{
#if ENABLE_BF_OBL
    __asm__ volatile (
        "ldr   r2, =0x24100010    \n"
        "movs  r3, #1             \n"
        "str   r3, [r2, #0]       \n"
        "ldr   r3, =0xE000ED28    \n"
        "ldr   r3, [r3]           \n"
        "str   r3, [r2, #4]       \n"
        "ldr   r3, =0xE000ED2C    \n"
        "ldr   r3, [r3]           \n"
        "str   r3, [r2, #8]       \n"
        "ldr   r3, =0xE000ED38    \n"
        "ldr   r3, [r3]           \n"
        "str   r3, [r2, #12]      \n"
        "ldr   r3, =0xE000ED34    \n"
        "ldr   r3, [r3]           \n"
        "str   r3, [r2, #16]      \n"
        "tst   lr, #4             \n"
        "ite   eq                 \n"
        "mrseq r0, msp            \n"
        "mrsne r0, psp            \n"
        "ldr   r3, [r0, #24]      \n"
        "str   r3, [r2, #20]      \n"
        "ldr   r3, [r0, #20]      \n"
        "str   r3, [r2, #24]      \n"
        "str   r0, [r2, #28]      \n"
        "dsb                      \n"
        "isb                      \n"
        ::: "r0", "r2", "r3", "memory"
    );
#endif
    __asm__ volatile (
        "b systemFaultAction      \n"
    );
}
#endif
