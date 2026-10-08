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

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>

// The ESP-IDF HAL headers pull <stdio.h> in transitively (xtensa-esp-elf and the
// RISC-V ESP variants), so the shared platform.h must pre-include it before the
// sprintf/snprintf poison pragma, otherwise the later transitive declaration
// trips the poison. Opt into that pre-include here; other platforms don't need it.
#define ENABLE_STDIO_PREINCLUDE

// CMSIS compatibility layer for non-ARM ESP32
// These provide the types and functions expected by shared Betaflight code.
#ifndef __ASM
#define __ASM __asm__
#endif

typedef enum { ESP32_IRQ_0 = 0 } IRQn_Type;

// Xtensa has no ARM BASEPRI register.  On ESP32-S3 all Betaflight peripheral
// handlers are routed to level-1 CPU interrupts, so raising PS.INTLEVEL to one
// provides the critical-section semantics required by ATOMIC_BLOCK.  Preserve
// the old level so nested blocks and ISR callers restore it correctly.
#if defined(ESP32S3)
__attribute__((always_inline)) static inline uint32_t esp32GetInterruptLevel(void)
{
    uint32_t ps;
    __asm__ volatile ("rsr.ps %0" : "=a"(ps));
    return ps & 0x0FU;
}

__attribute__((always_inline)) static inline void esp32SetInterruptLevel(uint32_t level)
{
    uint32_t ps;
    __asm__ volatile ("rsr.ps %0" : "=a"(ps));
    ps = (ps & ~0x0FU) | (level & 0x0FU);
    __asm__ volatile ("wsr.ps %0; rsync" :: "a"(ps) : "memory");
}

__attribute__((always_inline)) static inline void __set_BASEPRI(uint32_t basePri)
{
    // Values saved by __get_BASEPRI are raw Xtensa levels.  Other callers pass
    // ARM-style encoded priorities; any non-zero value masks our level-1 IRQs.
    esp32SetInterruptLevel(basePri <= 0x0FU ? basePri : 1U);
}

__attribute__((always_inline)) static inline uint32_t __get_BASEPRI(void)
{
    return esp32GetInterruptLevel();
}

__attribute__((always_inline)) static inline void __set_BASEPRI_MAX(uint32_t basePri)
{
    // Normalise like __set_BASEPRI (raw Xtensa level, or 1 for an ARM-encoded
    // priority) and only ever raise the level, never lower it.
    const uint32_t requestedLevel = basePri <= 0x0FU ? basePri : 1U;
    if (esp32GetInterruptLevel() < requestedLevel) {
        esp32SetInterruptLevel(requestedLevel);
    }
}

static inline void __enable_irq(void) { esp32SetInterruptLevel(0); }
static inline void __disable_irq(void) { esp32SetInterruptLevel(15); }
#else
// Bring-up stubs retained for the other ESP variants until their native
// interrupt-controller critical sections are implemented.
__attribute__((always_inline)) static inline void __set_BASEPRI(uint32_t basePri) { (void)basePri; }
__attribute__((always_inline)) static inline uint32_t __get_BASEPRI(void) { return 0; }
__attribute__((always_inline)) static inline void __set_BASEPRI_MAX(uint32_t basePri) { (void)basePri; }
static inline void __enable_irq(void) { }
static inline void __disable_irq(void) { }
#endif

// atomic.h's default non-barrier helpers contain ARM MSR instructions.  Keep
// them native on every ESP target as well; on S3 these inherit the real Xtensa
// critical-section implementation above, while the bring-up targets retain
// their existing stub semantics.
#define PLATFORM_CUSTOM_BASEPRI_NB
__attribute__((always_inline)) static inline void __set_BASEPRI_nb(uint32_t basePri) { __set_BASEPRI(basePri); }
__attribute__((always_inline)) static inline void __set_BASEPRI_MAX_nb(uint32_t basePri) { __set_BASEPRI_MAX(basePri); }

// NVIC compatibility stub
static inline void NVIC_SystemReset(void) { while(1); }

#define NVIC_PriorityGroup_2         0x500
#define PLATFORM_NO_LIBC             0
#define DEFIO_PORT_PINS              64

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

// ESP32 peripheral type definitions
// ESP-IDF uses integer port numbers for peripherals.
// We define typedefs and instance pointers to match what shared code expects.

typedef int esp32_peripheral_t;

#define I2C_TypeDef          esp32_peripheral_t
#define I2C_INST(i2c)        (*(i2c))

#define DMA_TypeDef          void*
#define DMA_InitTypeDef      int

#define ADC_TypeDef          void*

#define USART_TypeDef        esp32_peripheral_t
#define UART_INST(uart)      (*(uart))

#define TIM_OCInitTypeDef    void*
#define TIM_ICInitTypeDef    void*

#define SPI_TypeDef          esp32_peripheral_t
#define SPI_INST(spi)        (*(spi))

#define QUADSPI_TypeDef      void
#define MAX_QUADSPI_PIN_SEL  1

// Peripheral instances (pointers to static peripheral id storage)
// SPI: ESP32 has SPI2 and SPI3 available for general use
extern esp32_peripheral_t esp32SpiDev0;
extern esp32_peripheral_t esp32SpiDev1;
#define SPI0 (&esp32SpiDev0)
#define SPI1 (&esp32SpiDev1)

// I2C: ESP32-S3 has I2C0 and I2C1
extern esp32_peripheral_t esp32I2cDev0;
extern esp32_peripheral_t esp32I2cDev1;
#define I2C0 (&esp32I2cDev0)
#define I2C1 (&esp32I2cDev1)

// UART: ESP32-S3 has UART0, UART1, UART2
extern esp32_peripheral_t esp32UartDev0;
extern esp32_peripheral_t esp32UartDev1;
extern esp32_peripheral_t esp32UartDev2;
#define UART0 (&esp32UartDev0)
#define UART1 (&esp32UartDev1)
#define UART2 (&esp32UartDev2)

// LEDC channel-0 / RMT channel-0 output signal indices in the GPIO matrix.
// Naming diverges across chips: S3 / C5 / WROOM use the un-suffixed form,
// P4 spells everything with a _PAD_ infix.
#if defined(ESP32P4)
#define ESP32_LEDC_LS_SIG_OUT0_IDX LEDC_LS_SIG_OUT_PAD_OUT0_IDX
#define ESP32_RMT_SIG_OUT0_IDX     RMT_SIG_PAD_OUT0_IDX
#else
#define ESP32_LEDC_LS_SIG_OUT0_IDX LEDC_LS_SIG_OUT0_IDX
#define ESP32_RMT_SIG_OUT0_IDX     RMT_SIG_OUT0_IDX
#endif

#if defined(ESP32S3)
// ESP32-S3 executes normal .text from flash through a small instruction cache.
// Keep Betaflight's existing hot-path annotations meaningful by placing
// FAST_CODE and interrupt entry points in the IRAM window reserved by the S3
// linker script.  FAST_CODE_NOINLINE intentionally remains outside that
// window, matching the STM32 meaning used to keep large/cold helpers out of
// scarce instruction RAM.
#define FAST_CODE                       __attribute__((section(".iram.hot")))
#define FAST_CODE_NOINLINE              __attribute__((noinline))
#define FAST_CODE_NOINLINE_CRITICAL     __attribute__((section(".iram.hot"), noinline))
#define FAST_IRQ_HANDLER                __attribute__((section(".iram.isr"), noinline))

// All statically allocated DMA buffers live in internal DRAM on this bare-metal
// port.  Word alignment satisfies GDMA descriptor/data access requirements
// without changing the layout of non-S3 ESP targets.
#define DMA_DATA_ZERO_INIT              __attribute__((aligned(4)))
#define DMA_DATA                        __attribute__((aligned(4)))
#define STATIC_DMA_DATA_AUTO            static __attribute__((aligned(4)))
#else
#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO            static
#define FAST_IRQ_HANDLER
#endif

#define DEFAULT_CPU_OVERCLOCK           0

// 125us = 8kHz
#define TASK_GYROPID_DESIRED_PERIOD     125
#define SCHEDULER_DELAY_LIMIT           10

// GPIO mode constants for IO_CONFIG macro
// Names prefixed with BF_ to avoid conflict with ESP-IDF gpio_types.h
// which defines GPIO_MODE_INPUT/OUTPUT as enum members.
#define BF_GPIO_MODE_INPUT     0
#define BF_GPIO_MODE_OUTPUT    1
#define BF_GPIO_PULLUP         1
#define BF_GPIO_PULLDOWN       2

#define IO_CONFIG(mode, speed, pupd) ((mode) | ((speed) << 2) | ((pupd) << 5))

#define IOCFG_OUT_PP          IO_CONFIG(BF_GPIO_MODE_OUTPUT, 0, 0)
#define IOCFG_OUT_OD          IO_CONFIG(BF_GPIO_MODE_OUTPUT, 0, 0)
#define IOCFG_AF_PP           0
#define IOCFG_AF_OD           0
#define IOCFG_IPD             IO_CONFIG(BF_GPIO_MODE_INPUT, 0, 0)
#define IOCFG_IPU             IO_CONFIG(BF_GPIO_MODE_INPUT, 0, 0)
#define IOCFG_IN_FLOATING     IO_CONFIG(BF_GPIO_MODE_INPUT, 0, 0)

#define SPI_IO_AF_CFG           0
#define SPI_IO_AF_SCK_CFG_HIGH  0
#define SPI_IO_AF_SCK_CFG_LOW   0
#define SPI_IO_AF_SDI_CFG       0
#define SPI_IO_CS_CFG           IO_CONFIG(BF_GPIO_MODE_OUTPUT, 0, 0)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(BF_GPIO_MODE_INPUT, 0, BF_GPIO_PULLUP)

#define SERIAL_TRAIT_PIN_CONFIG     1
#define SERIAL_UART_FIRST_INDEX     0

// This MCU names its bus peripherals from zero (UART0, SPI0, I2C0). Opt the CLI
// `resource` command into hardware-instance ordinals so each index matches the
// peripheral name: the bus peripherals here are 0-based (e.g. `resource SERIAL_RX
// 1` is UART1), while logical resources (motor 1, servo 1) and sensors (gyro 1)
// stay 1-based. See resourceDisplayBase() in cli.c.
#define USE_RESOURCE_INDEX_FROM_ZERO

extern uint32_t SystemCoreClock;
extern uint32_t systemUniqueId[3];

#define U_ID_0 (systemUniqueId[0])
#define U_ID_1 (systemUniqueId[1])
#define U_ID_2 (systemUniqueId[2])

#define UART_TX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE

#define USE_LATE_TASK_STATISTICS

#ifndef DEFAULT_VOLTAGE_METER_SCALE
#define DEFAULT_VOLTAGE_METER_SCALE   100
#endif

#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER

// NVIC priority utility macros
#define NVIC_PRIORITY_GROUPING NVIC_PriorityGroup_2
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
#define NVIC_PRIORITY_SUB(prio) (((prio)>>4)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))
