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

// CMSIS compatibility layer for non-ARM ESP32
// These provide the types and functions expected by shared Betaflight code.
#ifndef __ASM
#define __ASM __asm__
#endif

typedef enum { ESP32_IRQ_0 = 0 } IRQn_Type;

// BASEPRI stubs - ESP32 (Xtensa) has no BASEPRI register.
// These are no-ops to satisfy the shared atomic.h code.
__attribute__((always_inline)) static inline void __set_BASEPRI(uint32_t basePri) { (void)basePri; }
__attribute__((always_inline)) static inline uint32_t __get_BASEPRI(void) { return 0; }
__attribute__((always_inline)) static inline void __set_BASEPRI_MAX(uint32_t basePri) { (void)basePri; }

// NVIC stubs
static inline void __enable_irq(void) { }
static inline void __disable_irq(void) { }
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
// SPI: ESP32-S3 has SPI2 (FSPI) and SPI3 (HSPI) available for general use
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

#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO            static
#define FAST_IRQ_HANDLER

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
