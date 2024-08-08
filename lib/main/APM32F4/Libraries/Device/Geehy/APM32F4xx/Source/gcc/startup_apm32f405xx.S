/**
 * @file        startup_apm32f405xx.S
 *
 * @brief       APM32F405xx Devices vector table for GCC based toolchains.
 *              This module performs:
 *                  - Set the initial SP
 *                  - Set the initial PC == Reset_Handler,
 *                  - Set the vector table entries with the exceptions ISR address
 *                  - Branches to main in the C library (which eventually
 *                  calls main()).
 *              After Reset the Cortex-M4 processor is in Thread mode,
 *              priority is Privileged, and the Stack is set to Main.
 *
 * @version     V1.0.0
 *
 * @date        2023-07-31
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  g_apm32_Vectors
.global  Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word  _start_address_init_data
/* start address for the .data section. defined in linker script */
.word  _start_address_data
/* end address for the .data section. defined in linker script */
.word  _end_address_data
/* start address for the .bss section. defined in linker script */
.word  _start_address_bss
/* end address for the .bss section. defined in linker script */
.word  _end_address_bss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

    .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
// Reset handler routine
Reset_Handler:
  ldr   sp, =_end_stack

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_start_address_data
  ldr r1, =_end_address_data
  ldr r2, =_start_address_init_data
  movs r3, #0
  b L_loop0_0

L_loop0:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

L_loop0_0:
  adds r4, r0, r3
  cmp r4, r1
  bcc L_loop0

  ldr r2, =_start_address_bss
  ldr r4, =_end_address_bss
  movs r3, #0
  b L_loop1

L_loop2:
  str  r3, [r2]
  adds r2, r2, #4

L_loop1:
  cmp r2, r4
  bcc L_loop2

  bl  SystemInit
  bl __libc_init_array
  bl  main
  bx  lr
.size  Reset_Handler, .-Reset_Handler

// This is the code that gets called when the processor receives an unexpected interrupt.
    .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler
// The minimal vector table for a Cortex M4.
   .section  .apm32_isr_vector,"a",%progbits
  .type  g_apm32_Vectors, %object
  .size  g_apm32_Vectors, .-g_apm32_Vectors

// Vector Table Mapped to Address 0 at Reset
g_apm32_Vectors:
  .word  _end_stack                         // Top of Stack
  .word  Reset_Handler                      // Reset Handler
  .word  NMI_Handler                        // NMI Handler
  .word  HardFault_Handler                  // Hard Fault Handler
  .word  MemManage_Handler                  // MPU Fault Handler
  .word  BusFault_Handler                   // Bus Fault Handler
  .word  UsageFault_Handler                 // Usage Fault Handler
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  0                                  // Reserved
  .word  SVC_Handler                        // SVCall Handler
  .word  DebugMon_Handler                   // Debug Monitor Handler
  .word  0                                  // Reserved
  .word  PendSV_Handler                     // PendSV Handler
  .word  SysTick_Handler                    // SysTick Handler

  /* External Interrupts */
  .word     WWDT_IRQHandler                 // Window WatchDog
  .word     PVD_IRQHandler                  // PVD through EINT Line detection
  .word     TAMP_STAMP_IRQHandler           // Tamper and TimeStamps through the EINT line
  .word     RTC_WKUP_IRQHandler             // RTC Wakeup through the EINT line
  .word     FLASH_IRQHandler                // FLASH
  .word     RCM_IRQHandler                  // RCM
  .word     EINT0_IRQHandler                // EINT Line0
  .word     EINT1_IRQHandler                // EINT Line1
  .word     EINT2_IRQHandler                // EINT Line2
  .word     EINT3_IRQHandler                // EINT Line3
  .word     EINT4_IRQHandler                // EINT Line4
  .word     DMA1_STR0_IRQHandler            // DMA1 Stream 0
  .word     DMA1_STR1_IRQHandler            // DMA1 Stream 1
  .word     DMA1_STR2_IRQHandler            // DMA1 Stream 2
  .word     DMA1_STR3_IRQHandler            // DMA1 Stream 3
  .word     DMA1_STR4_IRQHandler            // DMA1 Stream 4
  .word     DMA1_STR5_IRQHandler            // DMA1 Stream 5
  .word     DMA1_STR6_IRQHandler            // DMA1 Stream 6
  .word     ADC_IRQHandler                  // ADC1, ADC2 and ADC3s
  .word     CAN1_TX_IRQHandler              // CAN1 TX
  .word     CAN1_RX0_IRQHandler             // CAN1 RX0
  .word     CAN1_RX1_IRQHandler             // CAN1 RX1
  .word     CAN1_SCE_IRQHandler             // CAN1 SCE
  .word     EINT9_5_IRQHandler              // External Line[9:5]s
  .word     TMR1_BRK_TMR9_IRQHandler        // TMR1 Break and TMR9
  .word     TMR1_UP_TMR10_IRQHandler        // TMR1 Update and TMR10
  .word     TMR1_TRG_COM_TMR11_IRQHandler   // TMR1 Trigger and Commutation and TMR11
  .word     TMR1_CC_IRQHandler              // TMR1 Capture Compare
  .word     TMR2_IRQHandler                 // TMR2
  .word     TMR3_IRQHandler                 // TMR3
  .word     TMR4_IRQHandler                 // TMR4
  .word     I2C1_EV_IRQHandler              // I2C1 Event
  .word     I2C1_ER_IRQHandler              // I2C1 Error
  .word     I2C2_EV_IRQHandler              // I2C2 Event
  .word     I2C2_ER_IRQHandler              // I2C2 Error
  .word     SPI1_IRQHandler                 // SPI1
  .word     SPI2_IRQHandler                 // SPI2
  .word     USART1_IRQHandler               // USART1
  .word     USART2_IRQHandler               // USART2
  .word     USART3_IRQHandler               // USART3
  .word     EINT15_10_IRQHandler            // External Line[15:10]s
  .word     RTC_Alarm_IRQHandler            // RTC Alarm (A and B) through EINT Line
  .word     OTG_FS_WKUP_IRQHandler          // USB OTG FS Wakeup through EINT line
  .word     TMR8_BRK_TMR12_IRQHandler       // TMR8 Break and TMR12
  .word     TMR8_UP_TMR13_IRQHandler        // TMR8 Update and TMR13
  .word     TMR8_TRG_COM_TMR14_IRQHandler   // TMR8 Trigger and Commutation and TMR14
  .word     TMR8_CC_IRQHandler              // TMR8 Capture Compare
  .word     DMA1_STR7_IRQHandler            // DMA1 Stream7
  .word     EMMC_IRQHandler                 // EMMC
  .word     SDIO_IRQHandler                 // SDIO
  .word     TMR5_IRQHandler                 // TMR5
  .word     SPI3_IRQHandler                 // SPI3
  .word     UART4_IRQHandler                // UART4
  .word     UART5_IRQHandler                // UART5
  .word     TMR6_DAC_IRQHandler             // TMR6 and DAC1&2 underrun errors
  .word     TMR7_IRQHandler                 // TMR7
  .word     DMA2_STR0_IRQHandler            // DMA2 Stream 0
  .word     DMA2_STR1_IRQHandler            // DMA2 Stream 1
  .word     DMA2_STR2_IRQHandler            // DMA2 Stream 2
  .word     DMA2_STR3_IRQHandler            // DMA2 Stream 3
  .word     DMA2_STR4_IRQHandler            // DMA2 Stream 4
  .word     0                               // Reserved
  .word     0                               // Reserved
  .word     CAN2_TX_IRQHandler              // CAN2 TX
  .word     CAN2_RX0_IRQHandler             // CAN2 RX0
  .word     CAN2_RX1_IRQHandler             // CAN2 RX1
  .word     CAN2_SCE_IRQHandler             // CAN2 SCE
  .word     OTG_FS_IRQHandler               // USB OTG FS
  .word     DMA2_STR5_IRQHandler            // DMA2 Stream 5
  .word     DMA2_STR6_IRQHandler            // DMA2 Stream 6
  .word     DMA2_STR7_IRQHandler            // DMA2 Stream 7
  .word     USART6_IRQHandler               // USART6
  .word     I2C3_EV_IRQHandler              // I2C3 event
  .word     I2C3_ER_IRQHandler              // I2C3 error
  .word     OTG_HS1_EP1_OUT_IRQHandler      // USB OTG HS End Point 1 Out
  .word     OTG_HS1_EP1_IN_IRQHandler       // USB OTG HS End Point 1 In
  .word     OTG_HS1_WKUP_IRQHandler         // USB OTG HS Wakeup through EINT
  .word     OTG_HS1_IRQHandler              // USB OTG HS
  .word     0                               // Reserved
  .word     0                               // Reserved
  .word     HASH_RNG_IRQHandler             // Hash and Rng
  .word     FPU_IRQHandler                  // FPU
  .word     SM3_IRQHandler                  // SM3
  .word     SM4_IRQHandler                  // SM4
  .word     BN_IRQHandler                   // BN

// Default exception/interrupt handler

   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler

   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler

   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler

   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler

   .weak      WWDT_IRQHandler
   .thumb_set WWDT_IRQHandler,Default_Handler

   .weak      PVD_IRQHandler
   .thumb_set PVD_IRQHandler,Default_Handler

   .weak      TAMP_STAMP_IRQHandler
   .thumb_set TAMP_STAMP_IRQHandler,Default_Handler

   .weak      RTC_WKUP_IRQHandler
   .thumb_set RTC_WKUP_IRQHandler,Default_Handler

   .weak      FLASH_IRQHandler
   .thumb_set FLASH_IRQHandler,Default_Handler

   .weak      RCM_IRQHandler
   .thumb_set RCM_IRQHandler,Default_Handler

   .weak      EINT0_IRQHandler
   .thumb_set EINT0_IRQHandler,Default_Handler

   .weak      EINT1_IRQHandler
   .thumb_set EINT1_IRQHandler,Default_Handler

   .weak      EINT2_IRQHandler
   .thumb_set EINT2_IRQHandler,Default_Handler

   .weak      EINT3_IRQHandler
   .thumb_set EINT3_IRQHandler,Default_Handler

   .weak      EINT4_IRQHandler
   .thumb_set EINT4_IRQHandler,Default_Handler

   .weak      DMA1_STR0_IRQHandler
   .thumb_set DMA1_STR0_IRQHandler,Default_Handler

   .weak      DMA1_STR1_IRQHandler
   .thumb_set DMA1_STR1_IRQHandler,Default_Handler

   .weak      DMA1_STR2_IRQHandler
   .thumb_set DMA1_STR2_IRQHandler,Default_Handler

   .weak      DMA1_STR3_IRQHandler
   .thumb_set DMA1_STR3_IRQHandler,Default_Handler

   .weak      DMA1_STR4_IRQHandler
   .thumb_set DMA1_STR4_IRQHandler,Default_Handler

   .weak      DMA1_STR5_IRQHandler
   .thumb_set DMA1_STR5_IRQHandler,Default_Handler

   .weak      DMA1_STR6_IRQHandler
   .thumb_set DMA1_STR6_IRQHandler,Default_Handler

   .weak      ADC_IRQHandler
   .thumb_set ADC_IRQHandler,Default_Handler

   .weak      CAN1_TX_IRQHandler
   .thumb_set CAN1_TX_IRQHandler,Default_Handler

   .weak      CAN1_RX0_IRQHandler
   .thumb_set CAN1_RX0_IRQHandler,Default_Handler

   .weak      CAN1_RX1_IRQHandler
   .thumb_set CAN1_RX1_IRQHandler,Default_Handler

   .weak      CAN1_SCE_IRQHandler
   .thumb_set CAN1_SCE_IRQHandler,Default_Handler

   .weak      EINT9_5_IRQHandler
   .thumb_set EINT9_5_IRQHandler,Default_Handler

   .weak      TMR1_BRK_TMR9_IRQHandler
   .thumb_set TMR1_BRK_TMR9_IRQHandler,Default_Handler

   .weak      TMR1_UP_TMR10_IRQHandler
   .thumb_set TMR1_UP_TMR10_IRQHandler,Default_Handler

   .weak      TMR1_TRG_COM_TMR11_IRQHandler
   .thumb_set TMR1_TRG_COM_TMR11_IRQHandler,Default_Handler

   .weak      TMR1_CC_IRQHandler
   .thumb_set TMR1_CC_IRQHandler,Default_Handler

   .weak      TMR2_IRQHandler
   .thumb_set TMR2_IRQHandler,Default_Handler

   .weak      TMR3_IRQHandler
   .thumb_set TMR3_IRQHandler,Default_Handler

   .weak      TMR4_IRQHandler
   .thumb_set TMR4_IRQHandler,Default_Handler

   .weak      I2C1_EV_IRQHandler
   .thumb_set I2C1_EV_IRQHandler,Default_Handler

   .weak      I2C1_ER_IRQHandler
   .thumb_set I2C1_ER_IRQHandler,Default_Handler

   .weak      I2C2_EV_IRQHandler
   .thumb_set I2C2_EV_IRQHandler,Default_Handler

   .weak      I2C2_ER_IRQHandler
   .thumb_set I2C2_ER_IRQHandler,Default_Handler

   .weak      SPI1_IRQHandler
   .thumb_set SPI1_IRQHandler,Default_Handler

   .weak      SPI2_IRQHandler
   .thumb_set SPI2_IRQHandler,Default_Handler

   .weak      USART1_IRQHandler
   .thumb_set USART1_IRQHandler,Default_Handler

   .weak      USART2_IRQHandler
   .thumb_set USART2_IRQHandler,Default_Handler

   .weak      USART3_IRQHandler
   .thumb_set USART3_IRQHandler,Default_Handler

   .weak      EINT15_10_IRQHandler
   .thumb_set EINT15_10_IRQHandler,Default_Handler

   .weak      RTC_Alarm_IRQHandler
   .thumb_set RTC_Alarm_IRQHandler,Default_Handler

   .weak      OTG_FS_WKUP_IRQHandler
   .thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler

   .weak      TMR8_BRK_TMR12_IRQHandler
   .thumb_set TMR8_BRK_TMR12_IRQHandler,Default_Handler

   .weak      TMR8_UP_TMR13_IRQHandler
   .thumb_set TMR8_UP_TMR13_IRQHandler,Default_Handler

   .weak      TMR8_TRG_COM_TMR14_IRQHandler
   .thumb_set TMR8_TRG_COM_TMR14_IRQHandler,Default_Handler

   .weak      TMR8_CC_IRQHandler
   .thumb_set TMR8_CC_IRQHandler,Default_Handler

   .weak      DMA1_STR7_IRQHandler
   .thumb_set DMA1_STR7_IRQHandler,Default_Handler

   .weak      EMMC_IRQHandler
   .thumb_set EMMC_IRQHandler,Default_Handler

   .weak      SDIO_IRQHandler
   .thumb_set SDIO_IRQHandler,Default_Handler

   .weak      TMR5_IRQHandler
   .thumb_set TMR5_IRQHandler,Default_Handler

   .weak      SPI3_IRQHandler
   .thumb_set SPI3_IRQHandler,Default_Handler

   .weak      UART4_IRQHandler
   .thumb_set UART4_IRQHandler,Default_Handler

   .weak      UART5_IRQHandler
   .thumb_set UART5_IRQHandler,Default_Handler

   .weak      TMR6_DAC_IRQHandler
   .thumb_set TMR6_DAC_IRQHandler,Default_Handler

   .weak      TMR7_IRQHandler
   .thumb_set TMR7_IRQHandler,Default_Handler

   .weak      DMA2_STR0_IRQHandler
   .thumb_set DMA2_STR0_IRQHandler,Default_Handler

   .weak      DMA2_STR1_IRQHandler
   .thumb_set DMA2_STR1_IRQHandler,Default_Handler

   .weak      DMA2_STR2_IRQHandler
   .thumb_set DMA2_STR2_IRQHandler,Default_Handler

   .weak      DMA2_STR3_IRQHandler
   .thumb_set DMA2_STR3_IRQHandler,Default_Handler

   .weak      DMA2_STR4_IRQHandler
   .thumb_set DMA2_STR4_IRQHandler,Default_Handler

   .weak      CAN2_TX_IRQHandler
   .thumb_set CAN2_TX_IRQHandler,Default_Handler

   .weak      CAN2_RX0_IRQHandler
   .thumb_set CAN2_RX0_IRQHandler,Default_Handler

   .weak      CAN2_RX1_IRQHandler
   .thumb_set CAN2_RX1_IRQHandler,Default_Handler

   .weak      CAN2_SCE_IRQHandler
   .thumb_set CAN2_SCE_IRQHandler,Default_Handler

   .weak      OTG_FS_IRQHandler
   .thumb_set OTG_FS_IRQHandler,Default_Handler

   .weak      DMA2_STR5_IRQHandler
   .thumb_set DMA2_STR5_IRQHandler,Default_Handler

   .weak      DMA2_STR6_IRQHandler
   .thumb_set DMA2_STR6_IRQHandler,Default_Handler

   .weak      DMA2_STR7_IRQHandler
   .thumb_set DMA2_STR7_IRQHandler,Default_Handler

   .weak      USART6_IRQHandler
   .thumb_set USART6_IRQHandler,Default_Handler

   .weak      I2C3_EV_IRQHandler
   .thumb_set I2C3_EV_IRQHandler,Default_Handler

   .weak      I2C3_ER_IRQHandler
   .thumb_set I2C3_ER_IRQHandler,Default_Handler

   .weak      OTG_HS1_EP1_OUT_IRQHandler
   .thumb_set OTG_HS1_EP1_OUT_IRQHandler,Default_Handler

   .weak      OTG_HS1_EP1_IN_IRQHandler
   .thumb_set OTG_HS1_EP1_IN_IRQHandler,Default_Handler

   .weak      OTG_HS1_WKUP_IRQHandler
   .thumb_set OTG_HS1_WKUP_IRQHandler,Default_Handler

   .weak      OTG_HS1_IRQHandler
   .thumb_set OTG_HS1_IRQHandler,Default_Handler

   .weak      HASH_RNG_IRQHandler
   .thumb_set HASH_RNG_IRQHandler,Default_Handler

   .weak      FPU_IRQHandler
   .thumb_set FPU_IRQHandler,Default_Handler

   .weak      SM3_IRQHandler
   .thumb_set SM3_IRQHandler,Default_Handler

   .weak      SM4_IRQHandler
   .thumb_set SM4_IRQHandler,Default_Handler

   .weak      BN_IRQHandler
   .thumb_set BN_IRQHandler,Default_Handler
