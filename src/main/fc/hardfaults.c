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

#include "drivers/light_led.h"
#include "drivers/time.h"
#include "drivers/transponder_ir.h"

#include "fc/init.h"

#include "flight/mixer.h"

#ifdef STM32F7
void MemManage_Handler(void)
{
    LED2_ON;

    // fall out of the sky
    uint8_t requiredStateForMotors = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
    if ((systemState & requiredStateForMotors) == requiredStateForMotors) {
        stopMotors();
    }

#ifdef USE_TRANSPONDER
    // prevent IR LEDs from burning out.
    uint8_t requiredStateForTransponder = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_TRANSPONDER_ENABLED;
    if ((systemState & requiredStateForTransponder) == requiredStateForTransponder) {
        transponderIrDisable();
    }
#endif

    LED1_OFF;
    LED0_OFF;

    while (1) {
        delay(500);
        LED2_TOGGLE;
        delay(50);
        LED2_TOGGLE;
    }
}
#endif

#ifdef DEBUG_HARDFAULTS
//from: https://mcuoneclipse.com/2012/11/24/debugging-hard-faults-on-arm-cortex-m/
/**
 * hard_fault_handler_c:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
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

  // Dump fault info via UART3 TX on PC10 using direct register access.
  // USART3 is on APB1 at 0x40004800.
  // Configure for 115200 baud at 54MHz APB1: BRR = 54000000/115200 = 469
  volatile uint32_t *const USART3_CR1 = (volatile uint32_t *)0x40004800;
  volatile uint32_t *const USART3_BRR = (volatile uint32_t *)0x4000480C;
  volatile uint32_t *const USART3_ISR = (volatile uint32_t *)0x4000481C;
  volatile uint32_t *const USART3_TDR = (volatile uint32_t *)0x40004828;
  volatile uint32_t *const RCC_AHB1ENR = (volatile uint32_t *)0x40023830;
  volatile uint32_t *const RCC_APB1ENR = (volatile uint32_t *)0x40023840;
  volatile uint32_t *const GPIOC_MODER = (volatile uint32_t *)0x40020800;
  volatile uint32_t *const GPIOC_OSPEEDR = (volatile uint32_t *)0x40020808;
  volatile uint32_t *const GPIOC_AFRH = (volatile uint32_t *)0x40020824;

  *RCC_AHB1ENR |= (1 << 2);  // Enable GPIOC clock
  *RCC_APB1ENR |= (1 << 18); // Enable USART3 clock

  // Configure PC10 as AF7 (USART3_TX), push-pull, high speed
  *GPIOC_MODER = (*GPIOC_MODER & ~(3 << 20)) | (2 << 20);     // AF mode
  *GPIOC_OSPEEDR = (*GPIOC_OSPEEDR & ~(3 << 20)) | (3 << 20); // High speed
  *GPIOC_AFRH = (*GPIOC_AFRH & ~(0xF << 8)) | (7 << 8);       // AF7

  *USART3_CR1 = 0;           // Disable USART
  *USART3_BRR = 469;         // 115200 @ 54MHz APB1
  *USART3_CR1 = (1 << 0) | (1 << 3); // UE + TE

  // Simple polled hex output
  const char hex[] = "0123456789ABCDEF";
  #define FAULT_PUTC(c) do { while (!(*USART3_ISR & (1 << 7))); *USART3_TDR = (c); } while(0)
  #define FAULT_PUTS(s) do { for (const char *_p = (s); *_p; _p++) FAULT_PUTC(*_p); } while(0)
  #define FAULT_HEX32(v) do { \
      unsigned long _v = (v); \
      FAULT_PUTS("0x"); \
      for (int _i = 28; _i >= 0; _i -= 4) FAULT_PUTC(hex[(_v >> _i) & 0xF]); \
  } while(0)
  #define FAULT_REG(name, val) do { FAULT_PUTS(name "="); FAULT_HEX32(val); FAULT_PUTS("\r\n"); } while(0)

  FAULT_PUTS("\r\n\r\n*** HARD FAULT ***\r\n");
  FAULT_REG("PC ", stacked_pc);
  FAULT_REG("LR ", stacked_lr);
  FAULT_REG("R0 ", stacked_r0);
  FAULT_REG("R1 ", stacked_r1);
  FAULT_REG("R2 ", stacked_r2);
  FAULT_REG("R3 ", stacked_r3);
  FAULT_REG("R12", stacked_r12);
  FAULT_REG("PSR", stacked_psr);
  FAULT_REG("CFSR", _CFSR);
  FAULT_REG("HFSR", _HFSR);
  FAULT_REG("BFAR", _BFAR);
  FAULT_REG("MMAR", _MMAR);
  FAULT_PUTS("*** END ***\r\n");

  // Wait for last byte to finish transmitting
  while (!(*USART3_ISR & (1 << 6)));

  while (1);
}

#else
void HardFault_Handler(void)
{
    LED0_ON;
    LED1_ON;
    LED2_ON;

    // fall out of the sky
    uint8_t requiredStateForMotors = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
    if ((systemState & requiredStateForMotors) == requiredStateForMotors) {
        stopMotors();
    }

#ifdef USE_TRANSPONDER
    // prevent IR LEDs from burning out.
    uint8_t requiredStateForTransponder = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_TRANSPONDER_ENABLED;
    if ((systemState & requiredStateForTransponder) == requiredStateForTransponder) {
        transponderIrDisable();
    }
#endif

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;

    while (1) {
        delay(50);
        LED0_TOGGLE;
        LED1_TOGGLE;
        LED2_TOGGLE;
    }
}
#endif
