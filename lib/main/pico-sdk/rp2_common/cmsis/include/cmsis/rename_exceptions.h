/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _CMSIS_RENAME_EXCEPTIONS_H
#define _CMSIS_RENAME_EXCEPTIONS_H

#if LIB_CMSIS_CORE
// PICO_CONFIG: PICO_CMSIS_RENAME_EXCEPTIONS, Whether to rename SDK exceptions such as isr_nmi to their CMSIS equivalent i.e. NMI_Handler, type=bool, default=1, group=cmsis_core

// Note that since this header is included at the config stage, if you wish to override this you should do so via build compiler define
#ifndef PICO_CMSIS_RENAME_EXCEPTIONS
#define PICO_CMSIS_RENAME_EXCEPTIONS 1
#endif

#if PICO_CMSIS_RENAME_EXCEPTIONS
#if PICO_RP2040
#define isr_nmi NMI_Handler
#define isr_hardfault HardFault_Handler
#define isr_svcall SVC_Handler
#define isr_pendsv PendSV_Handler
#define isr_systick SysTick_Handler
#define isr_irq0 TIMER_IRQ_0_Handler
#define isr_irq1 TIMER_IRQ_1_Handler
#define isr_irq2 TIMER_IRQ_2_Handler
#define isr_irq3 TIMER_IRQ_3_Handler
#define isr_irq4 PWM_IRQ_WRAP_Handler
#define isr_irq5 USBCTRL_IRQ_Handler
#define isr_irq6 XIP_IRQ_Handler
#define isr_irq7 PIO0_IRQ_0_Handler
#define isr_irq8 PIO0_IRQ_1_Handler
#define isr_irq9 PIO1_IRQ_0_Handler
#define isr_irq10 PIO1_IRQ_1_Handler
#define isr_irq11 DMA_IRQ_0_Handler
#define isr_irq12 DMA_IRQ_1_Handler
#define isr_irq13 IO_IRQ_BANK0_Handler
#define isr_irq14 IO_IRQ_QSPI_Handler
#define isr_irq15 SIO_IRQ_PROC0_Handler
#define isr_irq16 SIO_IRQ_PROC1_Handler
#define isr_irq17 CLOCKS_IRQ_Handler
#define isr_irq18 SPI0_IRQ_Handler
#define isr_irq19 SPI1_IRQ_Handler
#define isr_irq20 UART0_IRQ_Handler
#define isr_irq21 UART1_IRQ_Handler
#define isr_irq22 ADC_IRQ_FIFO_Handler
#define isr_irq23 I2C0_IRQ_Handler
#define isr_irq24 I2C1_IRQ_Handler
#define isr_irq25 RTC_IRQ_Handler
#endif
#if PICO_RP2350
#define isr_nmi NMI_Handler
#define isr_hardfault HardFault_Handler
#define isr_svcall SVC_Handler
#define isr_pendsv PendSV_Handler
#define isr_systick SysTick_Handler
#define isr_irq0 TIMER0_IRQ_0_Handler
#define isr_irq1 TIMER0_IRQ_1_Handler
#define isr_irq2 TIMER0_IRQ_2_Handler
#define isr_irq3 TIMER0_IRQ_3_Handler
#define isr_irq4 TIMER1_IRQ_0_Handler
#define isr_irq5 TIMER1_IRQ_1_Handler
#define isr_irq6 TIMER1_IRQ_2_Handler
#define isr_irq7 TIMER1_IRQ_3_Handler
#define isr_irq8 PWM_IRQ_WRAP_0_Handler
#define isr_irq9 PWM_IRQ_WRAP_1_Handler
#define isr_irq10 DMA_IRQ_0_Handler
#define isr_irq11 DMA_IRQ_1_Handler
#define isr_irq12 DMA_IRQ_2_Handler
#define isr_irq13 DMA_IRQ_3_Handler
#define isr_irq14 USBCTRL_IRQ_Handler
#define isr_irq15 PIO0_IRQ_0_Handler
#define isr_irq16 PIO0_IRQ_1_Handler
#define isr_irq17 PIO1_IRQ_0_Handler
#define isr_irq18 PIO1_IRQ_1_Handler
#define isr_irq19 PIO2_IRQ_0_Handler
#define isr_irq20 PIO2_IRQ_1_Handler
#define isr_irq21 IO_IRQ_BANK0_Handler
#define isr_irq22 IO_IRQ_BANK0_NS_Handler
#define isr_irq23 IO_IRQ_QSPI_Handler
#define isr_irq24 IO_IRQ_QSPI_NS_Handler
#define isr_irq25 SIO_IRQ_FIFO_Handler
#define isr_irq26 SIO_IRQ_BELL_Handler
#define isr_irq27 SIO_IRQ_FIFO_NS_Handler
#define isr_irq28 SIO_IRQ_BELL_NS_Handler
#define isr_irq29 SIO_IRQ_MTIMECMP_Handler
#define isr_irq30 CLOCKS_IRQ_Handler
#define isr_irq31 SPI0_IRQ_Handler
#define isr_irq32 SPI1_IRQ_Handler
#define isr_irq33 UART0_IRQ_Handler
#define isr_irq34 UART1_IRQ_Handler
#define isr_irq35 ADC_IRQ_FIFO_Handler
#define isr_irq36 I2C0_IRQ_Handler
#define isr_irq37 I2C1_IRQ_Handler
#define isr_irq38 OTP_IRQ_Handler
#define isr_irq39 TRNG_IRQ_Handler
#define isr_irq42 PLL_SYS_IRQ_Handler
#define isr_irq43 PLL_USB_IRQ_Handler
#define isr_irq44 POWMAN_IRQ_POW_Handler
#define isr_irq45 POWMAN_IRQ_TIMER_Handler
#endif
#endif

#endif
#endif /* _CMSIS_RENAME_EXCEPTIONS_H */
