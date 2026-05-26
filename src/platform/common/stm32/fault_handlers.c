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

#ifdef DEBUG_HARDFAULTS
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
__attribute__((naked)) void HardFault_Handler(void)
{
#if ENABLE_BF_OBL
    /* BF↔OBL bring-up debug — capture fault state to AXISRAM2 NS at
     * 0x24100010..0x24100040 so the host can pull it via OBL's
     * @DBGRAM DFU alt after the post-fault reset. Naked to avoid the
     * compiler clobbering the stacked frame before we can read the
     * faulting PC/LR/SP. */
    __asm__ volatile (
        "ldr   r2, =0x24100010    \n"
        "movs  r3, #1             \n"
        "str   r3, [r2, #0]       \n"   // flag = 1 (hard fault)
        "ldr   r3, =0xE000ED28    \n"   // CFSR
        "ldr   r3, [r3]           \n"
        "str   r3, [r2, #4]       \n"
        "ldr   r3, =0xE000ED2C    \n"   // HFSR
        "ldr   r3, [r3]           \n"
        "str   r3, [r2, #8]       \n"
        "ldr   r3, =0xE000ED38    \n"   // BFAR
        "ldr   r3, [r3]           \n"
        "str   r3, [r2, #12]      \n"
        "ldr   r3, =0xE000ED34    \n"   // MMFAR
        "ldr   r3, [r3]           \n"
        "str   r3, [r2, #16]      \n"
        "tst   lr, #4             \n"   // MSP vs PSP from EXC_RETURN
        "ite   eq                 \n"
        "mrseq r0, msp            \n"
        "mrsne r0, psp            \n"
        "ldr   r3, [r0, #24]      \n"   // stacked PC
        "str   r3, [r2, #20]      \n"
        "ldr   r3, [r0, #20]      \n"   // stacked LR
        "str   r3, [r2, #24]      \n"
        "str   r0, [r2, #28]      \n"   // faulting SP
        "dsb                      \n"
        "isb                      \n"
        ::: "r0", "r2", "r3", "memory"
    );
#endif
    /* Tail-call into the C action handler (resets / blinks / etc.). */
    __asm__ volatile (
        "b systemFaultAction      \n"
    );
}
#endif
