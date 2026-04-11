/**
 ******************************************************************************
 * @file      startup_stm32n657xx.s
 * @author    GPM Application Team
 * @brief     STM32N657XX device vector table for GCC toolchain.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the initial PC == Reset_Handler,
 *                - Set the vector table entries with the exceptions ISR address
 *                - Copy firmware code from XSPI flash to RAM (LRUN)
 *                - Branches to main in the C library (which eventually
 *                  calls main()).
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

.syntax unified
.arch armv8.1-m.main
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .text section.
defined in linker script */
.word _sitext
/* start address for the .text section. defined in linker script */
.word _stext
/* end address for the .text section. defined in linker script */
.word _etext
/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 *
 *          This handler executes from XSPI flash (XIP) and copies all
 *          firmware code and data to RAM before jumping to main().
 *
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_sstack
  msr   MSPLIM, r0
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy firmware code from XSPI flash to RAM (.text, .rodata, .pg_registry, etc.) */
  ldr r0, =_stext
  ldr r1, =_etext
  ldr r2, =_sitext
  movs r3, #0
  b LoopCopyTextInit

CopyTextInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyTextInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyTextInit

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main

LoopForever:
  b LoopForever

  .size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

/******************************************************************************
*
* The STM32N657XX vector table. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word SecureFault_Handler
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  /* External Interrupts */
  .word PVD_PVM_IRQHandler                      /* PVD/PVM through EXTI Line detection               */
  .word 0                                       /* Reserved                                          */
  .word DTS_IRQHandler                          /* Digital Temperature Sensor                        */
  .word IWDG_IRQHandler                         /* Internal Watchdog                                 */
  .word 0                                       /* Reserved                                          */
  .word RCC_IRQHandler                          /* RCC global interrupts                             */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word EXTI0_IRQHandler                        /* EXTI Line0                                        */
  .word EXTI1_IRQHandler                        /* EXTI Line1                                        */
  .word EXTI2_IRQHandler                        /* EXTI Line2                                        */
  .word EXTI3_IRQHandler                        /* EXTI Line3                                        */
  .word EXTI4_IRQHandler                        /* EXTI Line4                                        */
  .word EXTI5_IRQHandler                        /* EXTI Line5                                        */
  .word EXTI6_IRQHandler                        /* EXTI Line6                                        */
  .word EXTI7_IRQHandler                        /* EXTI Line7                                        */
  .word EXTI8_IRQHandler                        /* EXTI Line8                                        */
  .word EXTI9_IRQHandler                        /* EXTI Line9                                        */
  .word EXTI10_IRQHandler                       /* EXTI Line10                                       */
  .word EXTI11_IRQHandler                       /* EXTI Line11                                       */
  .word EXTI12_IRQHandler                       /* EXTI Line12                                       */
  .word EXTI13_IRQHandler                       /* EXTI Line13                                       */
  .word EXTI14_IRQHandler                       /* EXTI Line14                                       */
  .word EXTI15_IRQHandler                       /* EXTI Line15                                       */
  .word RTC_IRQHandler                          /* RTC wakeup and alarm interrupts                   */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word TAMP_IRQHandler                         /* Tamper                                            */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word RISAF1S_IRQHandler                      /* RISAF1 secure global                              */
  .word IAC_IRQHandler                          /* IAC interrupt                                     */
  .word RCC_S_IRQHandler                        /* RCC secure                                        */
  .word 0                                       /* Reserved                                          */
  .word SDMMC1_IRQHandler                       /* SDMMC1 global interrupt                           */
  .word SDMMC2_IRQHandler                       /* SDMMC2 global interrupt                           */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word TIM1_BRK_IRQHandler                     /* TIM1 Break                                        */
  .word TIM1_UP_IRQHandler                      /* TIM1 Update                                       */
  .word TIM1_TRG_COM_IRQHandler                 /* TIM1 Trigger and Commutation                      */
  .word TIM1_CC_IRQHandler                      /* TIM1 Capture Compare                              */
  .word TIM2_IRQHandler                         /* TIM2                                              */
  .word TIM3_IRQHandler                         /* TIM3                                              */
  .word TIM4_IRQHandler                         /* TIM4                                              */
  .word TIM5_IRQHandler                         /* TIM5                                              */
  .word TIM6_IRQHandler                         /* TIM6                                              */
  .word TIM7_IRQHandler                         /* TIM7                                              */
  .word TIM8_BRK_IRQHandler                     /* TIM8 Break                                        */
  .word TIM8_UP_IRQHandler                      /* TIM8 Update                                       */
  .word TIM8_TRG_COM_IRQHandler                 /* TIM8 Trigger and Commutation                      */
  .word TIM8_CC_IRQHandler                      /* TIM8 Capture Compare                              */
  .word I2C1_EV_IRQHandler                      /* I2C1 Event                                        */
  .word I2C1_ER_IRQHandler                      /* I2C1 Error                                        */
  .word I2C2_EV_IRQHandler                      /* I2C2 Event                                        */
  .word I2C2_ER_IRQHandler                      /* I2C2 Error                                        */
  .word SPI1_IRQHandler                         /* SPI1                                              */
  .word SPI2_IRQHandler                         /* SPI2                                              */
  .word USART1_IRQHandler                       /* USART1                                            */
  .word USART2_IRQHandler                       /* USART2                                            */
  .word USART3_IRQHandler                       /* USART3                                            */
  .word UART4_IRQHandler                        /* UART4                                             */
  .word UART5_IRQHandler                        /* UART5                                             */
  .word LPUART1_IRQHandler                      /* LP UART1                                          */
  .word LPTIM1_IRQHandler                       /* LP TIM1                                           */
  .word LPTIM2_IRQHandler                       /* LP TIM2                                           */
  .word TIM15_IRQHandler                        /* TIM15                                             */
  .word TIM16_IRQHandler                        /* TIM16                                             */
  .word TIM17_IRQHandler                        /* TIM17                                             */
  .word 0                                       /* Reserved                                          */
  .word I2C3_EV_IRQHandler                      /* I2C3 Event                                        */
  .word I2C3_ER_IRQHandler                      /* I2C3 Error                                        */
  .word I2C4_EV_IRQHandler                      /* I2C4 Event                                        */
  .word I2C4_ER_IRQHandler                      /* I2C4 Error                                        */
  .word SPI3_IRQHandler                         /* SPI3                                              */
  .word SPI4_IRQHandler                         /* SPI4                                              */
  .word SPI5_IRQHandler                         /* SPI5                                              */
  .word SPI6_IRQHandler                         /* SPI6                                              */
  .word USART6_IRQHandler                       /* USART6                                            */
  .word UART7_IRQHandler                        /* UART7                                             */
  .word UART8_IRQHandler                        /* UART8                                             */
  .word UART9_IRQHandler                        /* UART9                                             */
  .word 0                                       /* Reserved                                          */
  .word USB1_OTG_HS_IRQHandler                  /* USB1 OTG HS global                                */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word FMC_IRQHandler                          /* FMC                                               */
  .word 0                                       /* Reserved                                          */
  .word RISAF5S_IRQHandler                      /* RISAF5 secure                                     */
  .word 0                                       /* Reserved                                          */
  .word DMA2D_IRQHandler                        /* DMA2D                                             */
  .word DCMIPP_IRQHandler                       /* DCMIPP global                                     */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word DMA1_Stream0_IRQHandler                 /* DMA1 Stream 0                                     */
  .word DMA1_Stream1_IRQHandler                 /* DMA1 Stream 1                                     */
  .word DMA1_Stream2_IRQHandler                 /* DMA1 Stream 2                                     */
  .word DMA1_Stream3_IRQHandler                 /* DMA1 Stream 3                                     */
  .word DMA1_Stream4_IRQHandler                 /* DMA1 Stream 4                                     */
  .word DMA1_Stream5_IRQHandler                 /* DMA1 Stream 5                                     */
  .word DMA1_Stream6_IRQHandler                 /* DMA1 Stream 6                                     */
  .word DMA1_Stream7_IRQHandler                 /* DMA1 Stream 7                                     */
  .word DMA1_Stream8_IRQHandler                 /* DMA1 Stream 8                                     */
  .word DMA1_Stream9_IRQHandler                 /* DMA1 Stream 9                                     */
  .word DMA1_Stream10_IRQHandler                /* DMA1 Stream 10                                    */
  .word DMA1_Stream11_IRQHandler                /* DMA1 Stream 11                                    */
  .word DMA1_Stream12_IRQHandler                /* DMA1 Stream 12                                    */
  .word DMA1_Stream13_IRQHandler                /* DMA1 Stream 13                                    */
  .word DMA1_Stream14_IRQHandler                /* DMA1 Stream 14                                    */
  .word DMA1_Stream15_IRQHandler                /* DMA1 Stream 15                                    */
  .word GPDMA1_IRQHandler                       /* GPDMA1 global                                     */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word FDCAN1_IT0_IRQHandler                   /* FDCAN1 interrupt line 0                           */
  .word FDCAN1_IT1_IRQHandler                   /* FDCAN1 interrupt line 1                           */
  .word FDCAN2_IT0_IRQHandler                   /* FDCAN2 interrupt line 0                           */
  .word FDCAN2_IT1_IRQHandler                   /* FDCAN2 interrupt line 1                           */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word LPTIM3_IRQHandler                       /* LP TIM3                                           */
  .word LPTIM4_IRQHandler                       /* LP TIM4                                           */
  .word LPTIM5_IRQHandler                       /* LP TIM5                                           */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word MDIOS_IRQHandler                        /* MDIOS global                                      */
  .word ETH1_SBD_IRQHandler                     /* Ethernet SBD                                      */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word GPU2D_IRQHandler                        /* GPU2D global                                      */
  .word GPU2D_ER_IRQHandler                     /* GPU2D error                                       */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word XSPI1_IRQHandler                        /* XSPI1 global interrupt                            */
  .word XSPI2_IRQHandler                        /* XSPI2 global interrupt                            */
  .word XSPI3_IRQHandler                        /* XSPI3 global interrupt                            */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */
  .word 0                                       /* Reserved                                          */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak SecureFault_Handler
  .thumb_set SecureFault_Handler,Default_Handler

  .weak SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak PVD_PVM_IRQHandler
  .thumb_set PVD_PVM_IRQHandler,Default_Handler

  .weak DTS_IRQHandler
  .thumb_set DTS_IRQHandler,Default_Handler

  .weak IWDG_IRQHandler
  .thumb_set IWDG_IRQHandler,Default_Handler

  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Default_Handler

  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Default_Handler

  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Default_Handler

  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Default_Handler

  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Default_Handler

  .weak EXTI5_IRQHandler
  .thumb_set EXTI5_IRQHandler,Default_Handler

  .weak EXTI6_IRQHandler
  .thumb_set EXTI6_IRQHandler,Default_Handler

  .weak EXTI7_IRQHandler
  .thumb_set EXTI7_IRQHandler,Default_Handler

  .weak EXTI8_IRQHandler
  .thumb_set EXTI8_IRQHandler,Default_Handler

  .weak EXTI9_IRQHandler
  .thumb_set EXTI9_IRQHandler,Default_Handler

  .weak EXTI10_IRQHandler
  .thumb_set EXTI10_IRQHandler,Default_Handler

  .weak EXTI11_IRQHandler
  .thumb_set EXTI11_IRQHandler,Default_Handler

  .weak EXTI12_IRQHandler
  .thumb_set EXTI12_IRQHandler,Default_Handler

  .weak EXTI13_IRQHandler
  .thumb_set EXTI13_IRQHandler,Default_Handler

  .weak EXTI14_IRQHandler
  .thumb_set EXTI14_IRQHandler,Default_Handler

  .weak EXTI15_IRQHandler
  .thumb_set EXTI15_IRQHandler,Default_Handler

  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler,Default_Handler

  .weak TAMP_IRQHandler
  .thumb_set TAMP_IRQHandler,Default_Handler

  .weak RISAF1S_IRQHandler
  .thumb_set RISAF1S_IRQHandler,Default_Handler

  .weak IAC_IRQHandler
  .thumb_set IAC_IRQHandler,Default_Handler

  .weak RCC_S_IRQHandler
  .thumb_set RCC_S_IRQHandler,Default_Handler

  .weak SDMMC1_IRQHandler
  .thumb_set SDMMC1_IRQHandler,Default_Handler

  .weak SDMMC2_IRQHandler
  .thumb_set SDMMC2_IRQHandler,Default_Handler

  .weak TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler,Default_Handler

  .weak TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler,Default_Handler

  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Default_Handler

  .weak TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler,Default_Handler

  .weak TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler,Default_Handler

  .weak TIM6_IRQHandler
  .thumb_set TIM6_IRQHandler,Default_Handler

  .weak TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler,Default_Handler

  .weak TIM8_BRK_IRQHandler
  .thumb_set TIM8_BRK_IRQHandler,Default_Handler

  .weak TIM8_UP_IRQHandler
  .thumb_set TIM8_UP_IRQHandler,Default_Handler

  .weak TIM8_TRG_COM_IRQHandler
  .thumb_set TIM8_TRG_COM_IRQHandler,Default_Handler

  .weak TIM8_CC_IRQHandler
  .thumb_set TIM8_CC_IRQHandler,Default_Handler

  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Default_Handler

  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Default_Handler

  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,Default_Handler

  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,Default_Handler

  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak USART3_IRQHandler
  .thumb_set USART3_IRQHandler,Default_Handler

  .weak UART4_IRQHandler
  .thumb_set UART4_IRQHandler,Default_Handler

  .weak UART5_IRQHandler
  .thumb_set UART5_IRQHandler,Default_Handler

  .weak LPUART1_IRQHandler
  .thumb_set LPUART1_IRQHandler,Default_Handler

  .weak LPTIM1_IRQHandler
  .thumb_set LPTIM1_IRQHandler,Default_Handler

  .weak LPTIM2_IRQHandler
  .thumb_set LPTIM2_IRQHandler,Default_Handler

  .weak TIM15_IRQHandler
  .thumb_set TIM15_IRQHandler,Default_Handler

  .weak TIM16_IRQHandler
  .thumb_set TIM16_IRQHandler,Default_Handler

  .weak TIM17_IRQHandler
  .thumb_set TIM17_IRQHandler,Default_Handler

  .weak I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler,Default_Handler

  .weak I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler,Default_Handler

  .weak I2C4_EV_IRQHandler
  .thumb_set I2C4_EV_IRQHandler,Default_Handler

  .weak I2C4_ER_IRQHandler
  .thumb_set I2C4_ER_IRQHandler,Default_Handler

  .weak SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler,Default_Handler

  .weak SPI4_IRQHandler
  .thumb_set SPI4_IRQHandler,Default_Handler

  .weak SPI5_IRQHandler
  .thumb_set SPI5_IRQHandler,Default_Handler

  .weak SPI6_IRQHandler
  .thumb_set SPI6_IRQHandler,Default_Handler

  .weak USART6_IRQHandler
  .thumb_set USART6_IRQHandler,Default_Handler

  .weak UART7_IRQHandler
  .thumb_set UART7_IRQHandler,Default_Handler

  .weak UART8_IRQHandler
  .thumb_set UART8_IRQHandler,Default_Handler

  .weak UART9_IRQHandler
  .thumb_set UART9_IRQHandler,Default_Handler

  .weak USB1_OTG_HS_IRQHandler
  .thumb_set USB1_OTG_HS_IRQHandler,Default_Handler

  .weak FMC_IRQHandler
  .thumb_set FMC_IRQHandler,Default_Handler

  .weak RISAF5S_IRQHandler
  .thumb_set RISAF5S_IRQHandler,Default_Handler

  .weak DMA2D_IRQHandler
  .thumb_set DMA2D_IRQHandler,Default_Handler

  .weak DCMIPP_IRQHandler
  .thumb_set DCMIPP_IRQHandler,Default_Handler

  .weak DMA1_Stream0_IRQHandler
  .thumb_set DMA1_Stream0_IRQHandler,Default_Handler

  .weak DMA1_Stream1_IRQHandler
  .thumb_set DMA1_Stream1_IRQHandler,Default_Handler

  .weak DMA1_Stream2_IRQHandler
  .thumb_set DMA1_Stream2_IRQHandler,Default_Handler

  .weak DMA1_Stream3_IRQHandler
  .thumb_set DMA1_Stream3_IRQHandler,Default_Handler

  .weak DMA1_Stream4_IRQHandler
  .thumb_set DMA1_Stream4_IRQHandler,Default_Handler

  .weak DMA1_Stream5_IRQHandler
  .thumb_set DMA1_Stream5_IRQHandler,Default_Handler

  .weak DMA1_Stream6_IRQHandler
  .thumb_set DMA1_Stream6_IRQHandler,Default_Handler

  .weak DMA1_Stream7_IRQHandler
  .thumb_set DMA1_Stream7_IRQHandler,Default_Handler

  .weak DMA1_Stream8_IRQHandler
  .thumb_set DMA1_Stream8_IRQHandler,Default_Handler

  .weak DMA1_Stream9_IRQHandler
  .thumb_set DMA1_Stream9_IRQHandler,Default_Handler

  .weak DMA1_Stream10_IRQHandler
  .thumb_set DMA1_Stream10_IRQHandler,Default_Handler

  .weak DMA1_Stream11_IRQHandler
  .thumb_set DMA1_Stream11_IRQHandler,Default_Handler

  .weak DMA1_Stream12_IRQHandler
  .thumb_set DMA1_Stream12_IRQHandler,Default_Handler

  .weak DMA1_Stream13_IRQHandler
  .thumb_set DMA1_Stream13_IRQHandler,Default_Handler

  .weak DMA1_Stream14_IRQHandler
  .thumb_set DMA1_Stream14_IRQHandler,Default_Handler

  .weak DMA1_Stream15_IRQHandler
  .thumb_set DMA1_Stream15_IRQHandler,Default_Handler

  .weak GPDMA1_IRQHandler
  .thumb_set GPDMA1_IRQHandler,Default_Handler

  .weak FDCAN1_IT0_IRQHandler
  .thumb_set FDCAN1_IT0_IRQHandler,Default_Handler

  .weak FDCAN1_IT1_IRQHandler
  .thumb_set FDCAN1_IT1_IRQHandler,Default_Handler

  .weak FDCAN2_IT0_IRQHandler
  .thumb_set FDCAN2_IT0_IRQHandler,Default_Handler

  .weak FDCAN2_IT1_IRQHandler
  .thumb_set FDCAN2_IT1_IRQHandler,Default_Handler

  .weak LPTIM3_IRQHandler
  .thumb_set LPTIM3_IRQHandler,Default_Handler

  .weak LPTIM4_IRQHandler
  .thumb_set LPTIM4_IRQHandler,Default_Handler

  .weak LPTIM5_IRQHandler
  .thumb_set LPTIM5_IRQHandler,Default_Handler

  .weak MDIOS_IRQHandler
  .thumb_set MDIOS_IRQHandler,Default_Handler

  .weak ETH1_SBD_IRQHandler
  .thumb_set ETH1_SBD_IRQHandler,Default_Handler

  .weak GPU2D_IRQHandler
  .thumb_set GPU2D_IRQHandler,Default_Handler

  .weak GPU2D_ER_IRQHandler
  .thumb_set GPU2D_ER_IRQHandler,Default_Handler

  .weak XSPI1_IRQHandler
  .thumb_set XSPI1_IRQHandler,Default_Handler

  .weak XSPI2_IRQHandler
  .thumb_set XSPI2_IRQHandler,Default_Handler

  .weak XSPI3_IRQHandler
  .thumb_set XSPI3_IRQHandler,Default_Handler
