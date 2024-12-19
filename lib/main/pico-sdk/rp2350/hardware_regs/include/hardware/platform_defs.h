/*
 * Copyright (c) 2024 Raspberry Pi Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HARDWARE_PLATFORM_DEFS_H
#define _HARDWARE_PLATFORM_DEFS_H

// This header is included from C and assembler - intended mostly for #defines; guard other stuff with #ifdef __ASSEMBLER__

#ifndef _u
#ifdef __ASSEMBLER__
#define _u(x) x
#else
#define _u(x) x ## u
#endif
#endif

#define NUM_CORES _u(2)
#define NUM_DMA_CHANNELS _u(16)
#define NUM_DMA_TIMERS _u(4)
#define NUM_DMA_MPU_REGIONS _u(8)
#define NUM_DMA_IRQS _u(4)
#define NUM_IRQS _u(52)
#define NUM_USER_IRQS _u(6)
#define NUM_PIOS _u(3)
#define NUM_PIO_STATE_MACHINES _u(4)
#define NUM_PIO_IRQS _u(2)
#define NUM_PWM_SLICES _u(12)
#define NUM_PWM_IRQS _u(2)
#define NUM_SPIN_LOCKS _u(32)
#define NUM_UARTS _u(2)
#define NUM_I2CS _u(2)
#define NUM_SPIS _u(2)
#define NUM_GENERIC_TIMERS _u(2)
#define NUM_ALARMS _u(4)
#if PICO_RP2350A
#define NUM_ADC_CHANNELS _u(5)
#define ADC_BASE_PIN _u(26)
#else
#define NUM_ADC_CHANNELS _u(9)
#define ADC_BASE_PIN _u(40)
#endif
#define NUM_RESETS _u(28)
#define NUM_DOORBELLS _u(8)

#if PICO_RP2350A
#define NUM_BANK0_GPIOS _u(30)
#else
#define NUM_BANK0_GPIOS _u(48)
#endif
#define NUM_QSPI_GPIOS _u(6)

#define NUM_OTP_PAGES _u(64)
#define NUM_OTP_PAGE_ROWS _u(64)
#define NUM_OTP_ROWS (NUM_OTP_PAGES * NUM_OTP_PAGE_ROWS)

#define PIO_INSTRUCTION_COUNT _u(32)

#define NUM_MPU_REGIONS _u(8)
#define NUM_SAU_REGIONS _u(8)
#define NUM_BOOT_LOCKS _u(8)

#define BOOTRAM_SIZE _u(0x400)
#define USBCTRL_DPRAM_SIZE _u(4096)

#ifndef __riscv
#define HAS_GPIO_COPROCESSOR 1
#define HAS_DOUBLE_COPROCESSOR 1
#define HAS_REDUNDANCY_COPROCESSOR 1
#endif
#define HAS_POWMAN_TIMER 1
#define HAS_RP2350_TRNG 1
#define HAS_HSTX 1

// PICO_CONFIG: XOSC_HZ, Crystal oscillator frequency in Hz, type=int, default=12000000, advanced=true, group=hardware_base
// NOTE:  The system and USB clocks are generated from the frequency using two PLLs.
// If you override this define, or SYS_CLK_HZ/USB_CLK_HZ below, you will *also* need to add your own adjusted PLL set-up defines to
// override the defaults which live in src/rp2_common/hardware_clocks/include/hardware/clocks.h
// Please see the comments there about calculating the new PLL setting values.
#ifndef XOSC_HZ
#ifdef XOSC_KHZ
#define XOSC_HZ ((XOSC_KHZ) * _u(1000))
#elif defined(XOSC_MHZ)
#define XOSC_HZ ((XOSC_MHZ) * _u(1000000))
#else
#define XOSC_HZ _u(12000000)
#endif
#endif

// PICO_CONFIG: SYS_CLK_HZ, System operating frequency in Hz, type=int, default=150000000, advanced=true, group=hardware_base
#ifndef SYS_CLK_HZ
#ifdef SYS_CLK_KHZ
#define SYS_CLK_HZ ((SYS_CLK_KHZ) * _u(1000))
#elif defined(SYS_CLK_MHZ)
#define SYS_CLK_HZ ((SYS_CLK_MHZ) * _u(1000000))
#else
#define SYS_CLK_HZ _u(150000000)
#endif
#endif

// PICO_CONFIG: USB_CLK_HZ, USB clock frequency. Must be 48MHz for the USB interface to operate correctly, type=int, default=48000000, advanced=true, group=hardware_base
#ifndef USB_CLK_HZ
#ifdef USB_CLK_KHZ
#define USB_CLK_HZ ((USB_CLK_KHZ) * _u(1000))
#elif defined(USB_CLK_MHZ)
#define USB_CLK_HZ ((USB_CLK_MHZ) * _u(1000000))
#else
#define USB_CLK_HZ _u(48000000)
#endif
#endif

// For backwards compatibility define XOSC_KHZ if the frequency is indeed an integer number of Khz.
#if defined(XOSC_HZ) && !defined(XOSC_KHZ) && (XOSC_HZ % 1000 == 0)
#define XOSC_KHZ (XOSC_HZ / 1000)
#endif

// For backwards compatibility define XOSC_MHZ if the frequency is indeed an integer number of Mhz.
#if defined(XOSC_KHZ) && !defined(XOSC_MHZ) && (XOSC_KHZ % 1000 == 0)
#define XOSC_MHZ (XOSC_KHZ / 1000)
#endif

// For backwards compatibility define SYS_CLK_KHZ if the frequency is indeed an integer number of Khz.
#if defined(SYS_CLK_HZ) && !defined(SYS_CLK_KHZ) && (SYS_CLK_HZ % 1000 == 0)
#define SYS_CLK_KHZ (SYS_CLK_HZ / 1000)
#endif

// For backwards compatibility define SYS_CLK_MHZ if the frequency is indeed an integer number of Mhz.
#if defined(SYS_CLK_KHZ) && !defined(SYS_CLK_MHZ) && (SYS_CLK_KHZ % 1000 == 0)
#define SYS_CLK_MHZ (SYS_CLK_KHZ / 1000)
#endif

// For backwards compatibility define USB_CLK_KHZ if the frequency is indeed an integer number of Khz.
#if defined(USB_CLK_HZ) && !defined(USB_CLK_KHZ) && (USB_CLK_HZ % 1000 == 0)
#define USB_CLK_KHZ (USB_CLK_HZ / 1000)
#endif

// For backwards compatibility define USB_CLK_MHZ if the frequency is indeed an integer number of Mhz.
#if defined(USB_CLK_KHZ) && !defined(USB_CLK_MHZ) && (USB_CLK_KHZ % 1000 == 0)
#define USB_CLK_MHZ (USB_CLK_KHZ / 1000)
#endif

#define ACCESSCTRL_PASSWORD_BITS _u(0xacce0000)
#define POWMAN_PASSWORD_BITS _u(0x5afe0000)

#ifdef __riscv
// Note the soft-table dispatch code is between the hard and soft vector
// tables, as it's inlined into the last slot of the hard table:
#if defined(__riscv_c) || defined(__riscv_zca)
// RISC-V with compressed instructions: NOTE that this is dependent on the size of the code in crt0_riscv.S
#define VTABLE_FIRST_IRQ 0x34
#else
// RISC-V without compressed instructions:
#define VTABLE_FIRST_IRQ 0x48
#endif
#else
// Armv8-M:
#define VTABLE_FIRST_IRQ 16
#endif
#define FIRST_USER_IRQ (NUM_IRQS - NUM_USER_IRQS)

#endif
