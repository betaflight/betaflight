/**
  ******************************************************************************
  * @file      startup_stm32c591xx.s
  * @brief     STM32C591xx devices vector table GCC toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address,
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  ******************************************************************************
  */

  .syntax unified
	.cpu cortex-m33
	.fpu softvfp
	.thumb

.global g_pfnVectors
.global Default_Handler
.global __Vectors
.set __Vectors, g_pfnVectors

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
  * @param  None
  * @retval : None
  */

    .section	.text.Reset_Handler
	.weak	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

  bl persistentObjectInit
/* Call the clock system initialization function.*/
  bl  SystemInit

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

/* Call the application's entry point.*/
  bl main
  bx lr

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
* The STM32C591xx vector table.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object

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
	/* External Interrupts */
	.word	WWDG_IRQHandler                   /* 0  */
	.word	PWR_PVD_IRQHandler                /* 1  */
	.word	RTC_IRQHandler                    /* 2  */
	.word	TAMP_IRQHandler                   /* 3  */
	.word	RAMCFG_IRQHandler                 /* 4  */
	.word	FLASH_IRQHandler                  /* 5  */
	.word	RCC_IRQHandler                    /* 6  */
	.word	EXTI0_IRQHandler                  /* 7  */
	.word	EXTI1_IRQHandler                  /* 8  */
	.word	EXTI2_IRQHandler                  /* 9  */
	.word	EXTI3_IRQHandler                  /* 10 */
	.word	EXTI4_IRQHandler                  /* 11 */
	.word	EXTI5_IRQHandler                  /* 12 */
	.word	EXTI6_IRQHandler                  /* 13 */
	.word	EXTI7_IRQHandler                  /* 14 */
	.word	EXTI8_IRQHandler                  /* 15 */
	.word	EXTI9_IRQHandler                  /* 16 */
	.word	EXTI10_IRQHandler                 /* 17 */
	.word	EXTI11_IRQHandler                 /* 18 */
	.word	EXTI12_IRQHandler                 /* 19 */
	.word	EXTI13_IRQHandler                 /* 20 */
	.word	EXTI14_IRQHandler                 /* 21 */
	.word	EXTI15_IRQHandler                 /* 22 */
	.word	LPDMA1_CH0_IRQHandler             /* 23 */
	.word	LPDMA1_CH1_IRQHandler             /* 24 */
	.word	LPDMA1_CH2_IRQHandler             /* 25 */
	.word	LPDMA1_CH3_IRQHandler             /* 26 */
	.word	LPDMA1_CH4_IRQHandler             /* 27 */
	.word	LPDMA1_CH5_IRQHandler             /* 28 */
	.word	LPDMA1_CH6_IRQHandler             /* 29 */
	.word	LPDMA1_CH7_IRQHandler             /* 30 */
	.word	IWDG_IRQHandler                   /* 31 */
	.word	ADC1_IRQHandler                   /* 32 */
	.word	ADC2_IRQHandler                   /* 33 */
	.word	0                                 /* 34 reserved */
	.word	0                                 /* 35 reserved */
	.word	TIM1_BRK_TERR_IERR_IRQHandler     /* 36 */
	.word	TIM1_UPD_IRQHandler               /* 37 */
	.word	TIM1_TRGI_COM_DIR_IDX_IRQHandler  /* 38 */
	.word	TIM1_CC_IRQHandler                /* 39 */
	.word	TIM2_IRQHandler                   /* 40 */
	.word	TIM5_IRQHandler                   /* 41 */
	.word	TIM6_IRQHandler                   /* 42 */
	.word	TIM7_IRQHandler                   /* 43 */
	.word	I2C1_EV_IRQHandler                /* 44 */
	.word	I2C1_ERR_IRQHandler               /* 45 */
	.word	I3C1_EV_IRQHandler                /* 46 */
	.word	I3C1_ERR_IRQHandler               /* 47 */
	.word	SPI1_IRQHandler                   /* 48 */
	.word	SPI2_IRQHandler                   /* 49 */
	.word	SPI3_IRQHandler                   /* 50 */
	.word	USART1_IRQHandler                 /* 51 */
	.word	USART2_IRQHandler                 /* 52 */
	.word	USART3_IRQHandler                 /* 53 */
	.word	UART4_IRQHandler                  /* 54 */
	.word	UART5_IRQHandler                  /* 55 */
	.word	LPUART1_IRQHandler                /* 56 */
	.word	LPTIM1_IRQHandler                 /* 57 */
	.word	TIM12_IRQHandler                  /* 58 */
	.word	TIM15_IRQHandler                  /* 59 */
	.word	TIM16_IRQHandler                  /* 60 */
	.word	TIM17_IRQHandler                  /* 61 */
	.word	USB_DRD_FS_IRQHandler             /* 62 */
	.word	CRS_IRQHandler                    /* 63 */
	.word	RNG_IRQHandler                    /* 64 */
	.word	FPU_IRQHandler                    /* 65 */
	.word	ICACHE_IRQHandler                 /* 66 */
	.word	CORDIC_IRQHandler                 /* 67 */
	.word	0                                 /* 68 reserved */
	.word	HASH_IRQHandler                   /* 69 */
	.word	I2C2_EV_IRQHandler                /* 70 */
	.word	I2C2_ERR_IRQHandler               /* 71 */
	.word	TIM8_BRK_TERR_IERR_IRQHandler     /* 72 */
	.word	TIM8_UPD_IRQHandler               /* 73 */
	.word	TIM8_TRGI_COM_DIR_IDX_IRQHandler  /* 74 */
	.word	TIM8_CC_IRQHandler                /* 75 */
	.word	COMP1_IRQHandler                  /* 76 */
	.word	DAC1_IRQHandler                   /* 77 */
	.word	LPDMA2_CH0_IRQHandler             /* 78 */
	.word	LPDMA2_CH1_IRQHandler             /* 79 */
	.word	LPDMA2_CH2_IRQHandler             /* 80 */
	.word	LPDMA2_CH3_IRQHandler             /* 81 */
	.word	LPDMA2_CH4_IRQHandler             /* 82 */
	.word	LPDMA2_CH5_IRQHandler             /* 83 */
	.word	LPDMA2_CH6_IRQHandler             /* 84 */
	.word	LPDMA2_CH7_IRQHandler             /* 85 */
	.word	0                                 /* 86 reserved */
	.word	0                                 /* 87 reserved */
	.word	0                                 /* 88 reserved */
	.word	TIM3_IRQHandler                   /* 89 */
	.word	TIM4_IRQHandler                   /* 90 */
	.word	XSPI1_IRQHandler                  /* 91 */
	.word	0                                 /* 92 reserved */
	.word	PKA_IRQHandler                    /* 93 */
	.word	0                                 /* 94 reserved */
	.word	0                                 /* 95 reserved */
	.word	USART6_IRQHandler                 /* 96 */
	.word	UART7_IRQHandler                  /* 97 */
	.word	ADC3_IRQHandler                   /* 98 */

  .size g_pfnVectors, .-g_pfnVectors

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

	.weak	PWR_PVD_IRQHandler
	.thumb_set PWR_PVD_IRQHandler,Default_Handler

	.weak	RTC_IRQHandler
	.thumb_set RTC_IRQHandler,Default_Handler

	.weak	TAMP_IRQHandler
	.thumb_set TAMP_IRQHandler,Default_Handler

	.weak	RAMCFG_IRQHandler
	.thumb_set RAMCFG_IRQHandler,Default_Handler

	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler

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

	.weak	LPDMA1_CH0_IRQHandler
	.thumb_set LPDMA1_CH0_IRQHandler,Default_Handler

	.weak	LPDMA1_CH1_IRQHandler
	.thumb_set LPDMA1_CH1_IRQHandler,Default_Handler

	.weak	LPDMA1_CH2_IRQHandler
	.thumb_set LPDMA1_CH2_IRQHandler,Default_Handler

	.weak	LPDMA1_CH3_IRQHandler
	.thumb_set LPDMA1_CH3_IRQHandler,Default_Handler

	.weak	LPDMA1_CH4_IRQHandler
	.thumb_set LPDMA1_CH4_IRQHandler,Default_Handler

	.weak	LPDMA1_CH5_IRQHandler
	.thumb_set LPDMA1_CH5_IRQHandler,Default_Handler

	.weak	LPDMA1_CH6_IRQHandler
	.thumb_set LPDMA1_CH6_IRQHandler,Default_Handler

	.weak	LPDMA1_CH7_IRQHandler
	.thumb_set LPDMA1_CH7_IRQHandler,Default_Handler

	.weak	IWDG_IRQHandler
	.thumb_set IWDG_IRQHandler,Default_Handler

	.weak	ADC1_IRQHandler
	.thumb_set ADC1_IRQHandler,Default_Handler

	.weak	ADC2_IRQHandler
	.thumb_set ADC2_IRQHandler,Default_Handler

	.weak	TIM1_BRK_TERR_IERR_IRQHandler
	.thumb_set TIM1_BRK_TERR_IERR_IRQHandler,Default_Handler

	.weak	TIM1_UPD_IRQHandler
	.thumb_set TIM1_UPD_IRQHandler,Default_Handler

	.weak	TIM1_TRGI_COM_DIR_IDX_IRQHandler
	.thumb_set TIM1_TRGI_COM_DIR_IDX_IRQHandler,Default_Handler

	.weak	TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,Default_Handler

	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler

	.weak	TIM5_IRQHandler
	.thumb_set TIM5_IRQHandler,Default_Handler

	.weak	TIM6_IRQHandler
	.thumb_set TIM6_IRQHandler,Default_Handler

	.weak	TIM7_IRQHandler
	.thumb_set TIM7_IRQHandler,Default_Handler

	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler

	.weak	I2C1_ERR_IRQHandler
	.thumb_set I2C1_ERR_IRQHandler,Default_Handler

	.weak	I3C1_EV_IRQHandler
	.thumb_set I3C1_EV_IRQHandler,Default_Handler

	.weak	I3C1_ERR_IRQHandler
	.thumb_set I3C1_ERR_IRQHandler,Default_Handler

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

	.weak	UART4_IRQHandler
	.thumb_set UART4_IRQHandler,Default_Handler

	.weak	UART5_IRQHandler
	.thumb_set UART5_IRQHandler,Default_Handler

	.weak	LPUART1_IRQHandler
	.thumb_set LPUART1_IRQHandler,Default_Handler

	.weak	LPTIM1_IRQHandler
	.thumb_set LPTIM1_IRQHandler,Default_Handler

	.weak	TIM12_IRQHandler
	.thumb_set TIM12_IRQHandler,Default_Handler

	.weak	TIM15_IRQHandler
	.thumb_set TIM15_IRQHandler,Default_Handler

	.weak	TIM16_IRQHandler
	.thumb_set TIM16_IRQHandler,Default_Handler

	.weak	TIM17_IRQHandler
	.thumb_set TIM17_IRQHandler,Default_Handler

	.weak	USB_DRD_FS_IRQHandler
	.thumb_set USB_DRD_FS_IRQHandler,Default_Handler

	.weak	CRS_IRQHandler
	.thumb_set CRS_IRQHandler,Default_Handler

	.weak	RNG_IRQHandler
	.thumb_set RNG_IRQHandler,Default_Handler

	.weak	FPU_IRQHandler
	.thumb_set FPU_IRQHandler,Default_Handler

	.weak	ICACHE_IRQHandler
	.thumb_set ICACHE_IRQHandler,Default_Handler

	.weak	CORDIC_IRQHandler
	.thumb_set CORDIC_IRQHandler,Default_Handler

	.weak	HASH_IRQHandler
	.thumb_set HASH_IRQHandler,Default_Handler

	.weak	I2C2_EV_IRQHandler
	.thumb_set I2C2_EV_IRQHandler,Default_Handler

	.weak	I2C2_ERR_IRQHandler
	.thumb_set I2C2_ERR_IRQHandler,Default_Handler

	.weak	TIM8_BRK_TERR_IERR_IRQHandler
	.thumb_set TIM8_BRK_TERR_IERR_IRQHandler,Default_Handler

	.weak	TIM8_UPD_IRQHandler
	.thumb_set TIM8_UPD_IRQHandler,Default_Handler

	.weak	TIM8_TRGI_COM_DIR_IDX_IRQHandler
	.thumb_set TIM8_TRGI_COM_DIR_IDX_IRQHandler,Default_Handler

	.weak	TIM8_CC_IRQHandler
	.thumb_set TIM8_CC_IRQHandler,Default_Handler

	.weak	COMP1_IRQHandler
	.thumb_set COMP1_IRQHandler,Default_Handler

	.weak	DAC1_IRQHandler
	.thumb_set DAC1_IRQHandler,Default_Handler

	.weak	LPDMA2_CH0_IRQHandler
	.thumb_set LPDMA2_CH0_IRQHandler,Default_Handler

	.weak	LPDMA2_CH1_IRQHandler
	.thumb_set LPDMA2_CH1_IRQHandler,Default_Handler

	.weak	LPDMA2_CH2_IRQHandler
	.thumb_set LPDMA2_CH2_IRQHandler,Default_Handler

	.weak	LPDMA2_CH3_IRQHandler
	.thumb_set LPDMA2_CH3_IRQHandler,Default_Handler

	.weak	LPDMA2_CH4_IRQHandler
	.thumb_set LPDMA2_CH4_IRQHandler,Default_Handler

	.weak	LPDMA2_CH5_IRQHandler
	.thumb_set LPDMA2_CH5_IRQHandler,Default_Handler

	.weak	LPDMA2_CH6_IRQHandler
	.thumb_set LPDMA2_CH6_IRQHandler,Default_Handler

	.weak	LPDMA2_CH7_IRQHandler
	.thumb_set LPDMA2_CH7_IRQHandler,Default_Handler

	.weak	TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,Default_Handler

	.weak	TIM4_IRQHandler
	.thumb_set TIM4_IRQHandler,Default_Handler

	.weak	XSPI1_IRQHandler
	.thumb_set XSPI1_IRQHandler,Default_Handler

	.weak	PKA_IRQHandler
	.thumb_set PKA_IRQHandler,Default_Handler

	.weak	USART6_IRQHandler
	.thumb_set USART6_IRQHandler,Default_Handler

	.weak	UART7_IRQHandler
	.thumb_set UART7_IRQHandler,Default_Handler

	.weak	ADC3_IRQHandler
	.thumb_set ADC3_IRQHandler,Default_Handler
