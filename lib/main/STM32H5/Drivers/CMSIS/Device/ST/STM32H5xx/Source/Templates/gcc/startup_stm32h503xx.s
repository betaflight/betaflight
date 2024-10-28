/**
  ******************************************************************************
  * @file      startup_stm32h503xx.s
  * @author    MCD Application Team
  * @brief     STM32H503xx devices vector table GCC toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address,
  *                - Configure the clock system
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M33 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
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
	.cpu cortex-m33
	.fpu softvfp
	.thumb

.global	g_pfnVectors
.global	Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word	_sidata
/* start address for the .data section. defined in linker script */
.word	_sdata
/* end address for the .data section. defined in linker script */
.word	_edata
/* start address for the .bss section. defined in linker script */
.word	_sbss
/* end address for the .bss section. defined in linker script */
.word	_ebss

.equ  BootRAM,        0xF1E0F85F
/**
  * @brief  This is the code that gets called when the processor first
  *          starts execution following a reset event. Only the absolutely
  *          necessary set is performed, after which the application
  *          supplied main() routine is called.
  * @param  None
  * @retval : None
  */

    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
  ldr   sp, =_estack    /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr	r3, =_sidata
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4

LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	b	LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
	movs	r3, #0
	str	r3, [r2], #4

LoopFillZerobss:
	ldr	r3, = _ebss
	cmp	r2, r3
	bcc	FillZerobss

/* Call the clock system initialization function.*/
    bl  SystemInit
/* Call static constructors */
    bl __libc_init_array
/* Call the application's entry point.*/
	bl	main

LoopForever:
    b LoopForever

.size	Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section	.text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b	Infinite_Loop
	.size	Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex-M33.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
 	.section	.isr_vector,"a",%progbits
	.type	g_pfnVectors, %object


g_pfnVectors:
	.word	_estack
	.word	Reset_Handler
	.word	NMI_Handler
	.word	HardFault_Handler
	.word	MemManage_Handler
	.word	BusFault_Handler
	.word	UsageFault_Handler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	SVC_Handler
	.word	DebugMon_Handler
	.word	0
	.word	PendSV_Handler
	.word	SysTick_Handler
	.word	WWDG_IRQHandler
	.word	PVD_AVD_IRQHandler
	.word	RTC_IRQHandler
	.word	0
	.word	TAMP_IRQHandler
	.word	RAMCFG_IRQHandler
	.word	FLASH_IRQHandler
	.word	0
	.word	GTZC_IRQHandler
	.word	RCC_IRQHandler
	.word	0
	.word	EXTI0_IRQHandler
	.word	EXTI1_IRQHandler
	.word	EXTI2_IRQHandler
	.word	EXTI3_IRQHandler
	.word	EXTI4_IRQHandler
	.word	EXTI5_IRQHandler
	.word	EXTI6_IRQHandler
	.word	EXTI7_IRQHandler
	.word	EXTI8_IRQHandler
	.word	EXTI9_IRQHandler
	.word	EXTI10_IRQHandler
	.word	EXTI11_IRQHandler
	.word	EXTI12_IRQHandler
	.word	EXTI13_IRQHandler
	.word	EXTI14_IRQHandler
	.word	EXTI15_IRQHandler
	.word	GPDMA1_Channel0_IRQHandler
	.word	GPDMA1_Channel1_IRQHandler
	.word	GPDMA1_Channel2_IRQHandler
	.word	GPDMA1_Channel3_IRQHandler
	.word	GPDMA1_Channel4_IRQHandler
	.word	GPDMA1_Channel5_IRQHandler
	.word	GPDMA1_Channel6_IRQHandler
	.word	GPDMA1_Channel7_IRQHandler
	.word	IWDG_IRQHandler
	.word	0	
	.word	ADC1_IRQHandler
	.word	DAC1_IRQHandler
	.word	FDCAN1_IT0_IRQHandler
	.word	FDCAN1_IT1_IRQHandler
	.word	TIM1_BRK_IRQHandler
	.word	TIM1_UP_IRQHandler
	.word	TIM1_TRG_COM_IRQHandler
	.word	TIM1_CC_IRQHandler
	.word	TIM2_IRQHandler
	.word	TIM3_IRQHandler
	.word	0
	.word	0
	.word	TIM6_IRQHandler
	.word	TIM7_IRQHandler
	.word	I2C1_EV_IRQHandler
	.word	I2C1_ER_IRQHandler
	.word	I2C2_EV_IRQHandler
	.word	I2C2_ER_IRQHandler
	.word	SPI1_IRQHandler
	.word	SPI2_IRQHandler
	.word	SPI3_IRQHandler	
	.word	USART1_IRQHandler
	.word	USART2_IRQHandler
	.word	USART3_IRQHandler
	.word	0
	.word	0
	.word	LPUART1_IRQHandler
	.word	LPTIM1_IRQHandler
    .word	0
    .word	0
    .word	0
    .word	0
    .word	0
	.word	LPTIM2_IRQHandler
	.word	0
	.word	0
	.word	0
	.word	USB_DRD_FS_IRQHandler
	.word	CRS_IRQHandler
    .word	0
    .word	0
    .word	0
    .word	0
    .word	0
    .word	0	
    .word	0	
    .word	0	
    .word	0	
    .word	0	
    .word	0	
    .word	0	
    .word	0	
    .word	0	
	.word	GPDMA2_Channel0_IRQHandler
	.word	GPDMA2_Channel1_IRQHandler
	.word	GPDMA2_Channel2_IRQHandler
	.word	GPDMA2_Channel3_IRQHandler
	.word	GPDMA2_Channel4_IRQHandler
	.word	GPDMA2_Channel5_IRQHandler
	.word	GPDMA2_Channel6_IRQHandler
	.word	GPDMA2_Channel7_IRQHandler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	FPU_IRQHandler
	.word	ICACHE_IRQHandler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0	
	.word	DTS_IRQHandler	
	.word	RNG_IRQHandler
	.word	0
	.word	0
	.word	HASH_IRQHandler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0	
	.word	I3C1_EV_IRQHandler
	.word	I3C1_ER_IRQHandler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0	
	.word	I3C2_EV_IRQHandler
	.word	I3C2_ER_IRQHandler
	.word	COMP1_IRQHandler

	.size	g_pfnVectors, .-g_pfnVectors

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

	.weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler
	
	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

	.weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

	.weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler

	.weak	PVD_AVD_IRQHandler
	.thumb_set PVD_AVD_IRQHandler,Default_Handler

	.weak	RTC_IRQHandler
	.thumb_set RTC_IRQHandler,Default_Handler

	.weak	TAMP_IRQHandler
	.thumb_set TAMP_IRQHandler,Default_Handler

	.weak	RAMCFG_IRQHandler
	.thumb_set RAMCFG_IRQHandler,Default_Handler

	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler

	.weak	GTZC_IRQHandler
	.thumb_set GTZC_IRQHandler,Default_Handler

	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler

	.weak	EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler

	.weak	EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler

	.weak	EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler

	.weak	EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler

	.weak	EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler

	.weak	EXTI5_IRQHandler
	.thumb_set EXTI5_IRQHandler,Default_Handler

	.weak	EXTI6_IRQHandler
	.thumb_set EXTI6_IRQHandler,Default_Handler

	.weak	EXTI7_IRQHandler
	.thumb_set EXTI7_IRQHandler,Default_Handler

	.weak	EXTI8_IRQHandler
	.thumb_set EXTI8_IRQHandler,Default_Handler

	.weak	EXTI9_IRQHandler
	.thumb_set EXTI9_IRQHandler,Default_Handler

	.weak	EXTI10_IRQHandler
	.thumb_set EXTI10_IRQHandler,Default_Handler

	.weak	EXTI11_IRQHandler
	.thumb_set EXTI11_IRQHandler,Default_Handler

	.weak	EXTI12_IRQHandler
	.thumb_set EXTI12_IRQHandler,Default_Handler

	.weak	EXTI13_IRQHandler
	.thumb_set EXTI13_IRQHandler,Default_Handler

	.weak	EXTI14_IRQHandler
	.thumb_set EXTI14_IRQHandler,Default_Handler

	.weak	EXTI15_IRQHandler
	.thumb_set EXTI15_IRQHandler,Default_Handler

	.weak	GPDMA1_Channel0_IRQHandler
	.thumb_set GPDMA1_Channel0_IRQHandler,Default_Handler

	.weak	GPDMA1_Channel1_IRQHandler
	.thumb_set GPDMA1_Channel1_IRQHandler,Default_Handler
	
	.weak	GPDMA1_Channel2_IRQHandler
	.thumb_set GPDMA1_Channel2_IRQHandler,Default_Handler
	
	.weak	GPDMA1_Channel3_IRQHandler
	.thumb_set GPDMA1_Channel3_IRQHandler,Default_Handler

	.weak	GPDMA1_Channel4_IRQHandler
	.thumb_set GPDMA1_Channel4_IRQHandler,Default_Handler

	.weak	GPDMA1_Channel5_IRQHandler
	.thumb_set GPDMA1_Channel5_IRQHandler,Default_Handler

	.weak	GPDMA1_Channel6_IRQHandler
	.thumb_set GPDMA1_Channel6_IRQHandler,Default_Handler

	.weak	GPDMA1_Channel7_IRQHandler
	.thumb_set GPDMA1_Channel7_IRQHandler,Default_Handler

	.weak	IWDG_IRQHandler	
	.thumb_set IWDG_IRQHandler,Default_Handler

	.weak	ADC1_IRQHandler
	.thumb_set ADC1_IRQHandler,Default_Handler

	.weak	DAC1_IRQHandler
	.thumb_set DAC1_IRQHandler,Default_Handler

	.weak	FDCAN1_IT0_IRQHandler
	.thumb_set FDCAN1_IT0_IRQHandler,Default_Handler

	.weak	FDCAN1_IT1_IRQHandler
	.thumb_set FDCAN1_IT1_IRQHandler,Default_Handler

	.weak	TIM1_BRK_IRQHandler
	.thumb_set TIM1_BRK_IRQHandler,Default_Handler

	.weak	TIM1_UP_IRQHandler
	.thumb_set TIM1_UP_IRQHandler,Default_Handler

	.weak	TIM1_TRG_COM_IRQHandler
	.thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

	.weak	TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,Default_Handler

	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler

	.weak	TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,Default_Handler

	.weak	TIM6_IRQHandler
	.thumb_set TIM6_IRQHandler,Default_Handler

	.weak	TIM7_IRQHandler
	.thumb_set TIM7_IRQHandler,Default_Handler

	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler

	.weak	I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler

	.weak	I2C2_EV_IRQHandler
	.thumb_set I2C2_EV_IRQHandler,Default_Handler

	.weak	I2C2_ER_IRQHandler
	.thumb_set I2C2_ER_IRQHandler,Default_Handler

	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler

	.weak	SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler

	.weak	SPI3_IRQHandler
	.thumb_set SPI3_IRQHandler,Default_Handler

	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler

	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler

	.weak	USART3_IRQHandler
	.thumb_set USART3_IRQHandler,Default_Handler

	.weak	LPUART1_IRQHandler
	.thumb_set LPUART1_IRQHandler,Default_Handler

	.weak	LPTIM1_IRQHandler
	.thumb_set LPTIM1_IRQHandler,Default_Handler

	.weak	LPTIM2_IRQHandler
	.thumb_set LPTIM2_IRQHandler,Default_Handler

	.weak	USB_DRD_FS_IRQHandler
	.thumb_set USB_DRD_FS_IRQHandler,Default_Handler

	.weak	CRS_IRQHandler
	.thumb_set CRS_IRQHandler,Default_Handler

	.weak	GPDMA2_Channel0_IRQHandler
	.thumb_set GPDMA2_Channel0_IRQHandler,Default_Handler

	.weak	GPDMA2_Channel1_IRQHandler
	.thumb_set GPDMA2_Channel1_IRQHandler,Default_Handler

	.weak	GPDMA2_Channel2_IRQHandler
	.thumb_set GPDMA2_Channel2_IRQHandler,Default_Handler

	.weak	GPDMA2_Channel3_IRQHandler
	.thumb_set GPDMA2_Channel3_IRQHandler,Default_Handler

	.weak	GPDMA2_Channel4_IRQHandler
	.thumb_set GPDMA2_Channel4_IRQHandler,Default_Handler

	.weak	GPDMA2_Channel5_IRQHandler
	.thumb_set GPDMA2_Channel5_IRQHandler,Default_Handler

	.weak	GPDMA2_Channel6_IRQHandler
	.thumb_set GPDMA2_Channel6_IRQHandler,Default_Handler

	.weak	GPDMA2_Channel7_IRQHandler
	.thumb_set GPDMA2_Channel7_IRQHandler,Default_Handler

	.weak	FPU_IRQHandler	
	.thumb_set FPU_IRQHandler,Default_Handler
	.weak	ICACHE_IRQHandler	
	.thumb_set ICACHE_IRQHandler,Default_Handler

	.weak	DTS_IRQHandler	
	.thumb_set DTS_IRQHandler,Default_Handler

	.weak	RNG_IRQHandler
	.thumb_set RNG_IRQHandler,Default_Handler

	.weak	HASH_IRQHandler	
	.thumb_set HASH_IRQHandler,Default_Handler

	.weak	I3C1_EV_IRQHandler
	.thumb_set I3C1_EV_IRQHandler,Default_Handler

	.weak	I3C1_ER_IRQHandler	
	.thumb_set I3C1_ER_IRQHandler,Default_Handler

	.weak	I3C2_EV_IRQHandler
	.thumb_set I3C2_EV_IRQHandler,Default_Handler

	.weak	I3C2_ER_IRQHandler
	.thumb_set I3C2_ER_IRQHandler,Default_Handler

	.weak	COMP1_IRQHandler
	.thumb_set COMP1_IRQHandler,Default_Handler
