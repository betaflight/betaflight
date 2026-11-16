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
/*
 * porting for ch32h41x by Temperslee
 */
#pragma once

#if defined(CH32H415)

#include <stdbool.h>

#include "ch32h417.h"
#include "ch32_debug.h"
#include "platform/priority.h"

// The paired H415 bootloader currently accepts only the hardware BOOT0 input.
#define PLATFORM_SUPPORTS_SOFTWARE_BOOTLOADER 0

// Chip Unique ID on H41X
#define U_ID_0 (*(uint32_t *)0x1ffff7e8)
#define U_ID_1 (*(uint32_t *)0x1ffff7ec)
#define U_ID_2 (*(uint32_t *)0x1ffff7f0)

#define FLASH_CONFIG_STREAMER_BUFFER_SIZE   256     // fast program is 256bytes

#define __FAST_INTERRUPT       __attribute__((interrupt("WCH-Interrupt-fast")))

#define PFIC_ITHRESDR (*(volatile uint32_t *)0xE000E040)

__attribute__((always_inline)) static inline uint32_t __get_BASEPRI(void)
{
    // ATOMIC_BLOCK saves and restores the raw PFIC threshold.
    return PFIC_ITHRESDR & 0xF0;
}

__attribute__((always_inline)) static inline void __set_BASEPRI_nb(uint32_t basePri)
{
    const uint32_t masked = basePri & 0xF0;
    PFIC_ITHRESDR = masked;
#ifdef __riscv
    if (masked != 0) {
        asm volatile ("fence.i" ::: "memory");
    }
#endif
}

__attribute__((always_inline)) static inline void __set_BASEPRI_MAX_nb(uint32_t basePri)
{
    const uint32_t priority = ch32AtomicPriority(basePri);
    const uint32_t current = __get_BASEPRI();
    if (priority != 0 && (current == 0 || priority < current)) {
        PFIC_ITHRESDR = priority;
#ifdef __riscv
        asm volatile ("fence.i" ::: "memory");
#endif
    }
}

__attribute__((always_inline)) static inline void ch32DmaBarrier(void)
{
    asm volatile ("fence iorw, iorw" ::: "memory");
}

__attribute__((always_inline)) static inline void ch32InvalidateFlashCache(void)
{
#if defined(__riscv) && defined(Core_V5F) && !defined(UNIT_TEST)
    // QingKe V5F caches reads from the internal flash window (0x08000000+) via
    // CSR 0xBC2. Invalidate cached lines via CSR 0xBD0 bit 2 after flash erase
    // or fast-page programming so readback verification observes physical flash.
    asm volatile (
        "fence iorw, iorw\n\t"
        "csrwi 0xbd0, 0x4\n\t"
        "fence.i"
        : : : "memory"
    );
#endif
}

__attribute__((always_inline)) static inline void __set_BASEPRI(uint32_t basePri)
{
    // Publish protected writes before restoring a less restrictive threshold.
    asm volatile ("fence iorw, iorw" ::: "memory");
    __set_BASEPRI_nb(basePri);
    asm volatile ("fence iorw, iorw" ::: "memory");
}

__attribute__((always_inline)) static inline void __set_BASEPRI_MAX(uint32_t basePri)
{
    __set_BASEPRI_MAX_nb(basePri);
    asm volatile ("fence iorw, iorw" ::: "memory");
}

#define PLATFORM_CUSTOM_BASEPRI_NB

// QingKe global interrupt enable is in CSR 0x800. Save and clear both
// enable bits atomically, then restore only those that were originally set.
__attribute__((always_inline)) static inline bool ch32GlobalInterruptsEnabled(void)
{
    uint32_t state;
    asm volatile ("csrr %0, 0x800" : "=r" (state) : : "memory");
    return (state & 0x08u) != 0;
}

__attribute__((always_inline)) static inline uint32_t ch32IrqSave(void)
{
    uint32_t state;
    asm volatile ("csrrc %0, 0x800, %1\n\tfence.i" : "=r" (state) : "r" (0x88u) : "memory");
    return state & 0x88u;
}

__attribute__((always_inline)) static inline void ch32IrqRestore(uint32_t *state)
{
    asm volatile ("fence iorw, iorw\n\tcsrs 0x800, %0" : : "r" (*state) : "memory");
}

#define PLATFORM_CUSTOM_ATOMIC_BLOCK_ALL
#define ATOMIC_BLOCK_ALL() for (uint32_t __irq_save __attribute__((__cleanup__(ch32IrqRestore), __unused__)) = ch32IrqSave(), \
                                      __todo = 1; __todo; __todo = 0)

#ifndef CH32H4
#define CH32H4
#endif

#define I2C_TRAIT_STATE 1

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#endif

#ifdef CH32H4

#ifndef USE_ITCM_RAM
#define USE_ITCM_RAM
#endif
#define ITCM_RAM_OPTIMISATION "-O2", "-freorder-blocks-algorithm=simple"
#ifndef USE_FAST_DATA
#define USE_FAST_DATA
#endif
#ifndef USE_RPM_FILTER
#define USE_RPM_FILTER
#endif
#ifndef USE_DYN_IDLE
#define USE_DYN_IDLE
#endif
#ifndef USE_DYN_NOTCH_FILTER
#define USE_DYN_NOTCH_FILTER
#endif
#ifndef USE_ADC_INTERNAL
#define USE_ADC_INTERNAL
#endif
#ifndef USE_PLATFORM_UART_IDLE_CALLBACK
#define USE_PLATFORM_UART_IDLE_CALLBACK
#endif
#define ADC_VOLTAGE_REFERENCE_MV            3300
#define PLATFORM_VCP_TX_STALL_TIMEOUT_MS    500
#ifndef USE_PERSISTENT_MSC_RTC
#define USE_PERSISTENT_MSC_RTC
#endif
#ifndef USE_DMA_SPEC
#define USE_DMA_SPEC
#endif
#ifndef USE_PERSISTENT_OBJECTS
#define USE_PERSISTENT_OBJECTS
#endif
#ifndef USE_LATE_TASK_STATISTICS
#define USE_LATE_TASK_STATISTICS
#endif

#define PLATFORM_TRAIT_ADC_DEVICE 1

#define CAMERA_CONTROL_HARDWARE_PWM_AVAILABLE
#define CAMERA_CONTROL_SOFTWARE_PWM_AVAILABLE
#define CAMERA_CONTROL_DAC_AVAILABLE
#ifndef DEFAULT_CAMERA_CONTROL_MODE
#define DEFAULT_CAMERA_CONTROL_MODE CAMERA_CONTROL_MODE_DAC
#endif
#endif

#ifdef CH32H4
#define TASK_GYROPID_DESIRED_PERIOD     125 // 1000us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10
#else
#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100
#endif

#define DEFAULT_CPU_OVERCLOCK 0

#ifdef CH32H4
// Move ISRs to fast ram to avoid flash latency.
#define FAST_IRQ_HANDLER FAST_CODE
#else
#define FAST_IRQ_HANDLER
#endif

// Load DMA data to DTCM
#define DMA_DATA_ZERO_INIT        __attribute__((aligned(32)))
#define DMA_DATA                  __attribute__((aligned(32)))
#define STATIC_DMA_DATA_AUTO      __attribute__((aligned(32))) static

#define DMA_RAM      __attribute__((aligned(32)))
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW

#ifndef USE_TIMER_MGMT
#define USE_TIMER_MGMT
#endif
#ifndef USE_TIMER_AF
#define USE_TIMER_AF
#endif

#if defined(CH32H4)
#define GPIO_PIN_RESET       0
#define DIR_OUT              0x03
#define DIR_IN               0x00

#define GPIO_MODE_IN_AN      0x00
#define GPIO_MODE_IN_FLOAT   0x01
#define GPIO_MODE_IN_PULL    0x02

#define GPIO_MODE_OUT_PP     0x00
#define GPIO_MODE_OUT_OD     0x01
#define GPIO_MODE_OUT_AF_PP  0x02
#define GPIO_MODE_OUT_AF_OD  0x03

#define GPIO_SPEED_LOW          0x00
#define GPIO_SPEED_MEDIUM       0x01
#define GPIO_SPEED_HIGH         0x02
#define GPIO_SPEED_VERY_HIGH    0x03

#define GPIO_PULL_NONE       0x00
#define GPIO_PULL_DOWN       0x01
#define GPIO_PULL_UP         0x02

// bit 0..1: DIR[1:0], bit 2..3: MODE[1:0], bit 4..5: SPEED[1:0], bit 6..7: PULL[1:0].
// Note: CH32H41x GPIO disconnects internal pull resistors while in output/AF modes
// (RM 9.2.7-9.2.8); PULL[1:0] applies in input mode or when drivers switch pins to input.
#define IO_CONFIG(dir, mode, speed, pupd) ((dir) | ((mode) << 2) | ((speed) << 4) | ((pupd) << 6))

#define IOCFG_OUT_PP         IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_OUT_PP_UP      IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)
#define IOCFG_OUT_PP_25      IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_OUT_OD         IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_OD, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)

#define IOCFG_AF_PP          IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_AF_PP_PD       IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)
#define IOCFG_AF_OD          IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_OD, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)

#define IOCFG_IPD            IO_CONFIG(DIR_IN, GPIO_MODE_IN_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_DOWN)
#define IOCFG_IPU            IO_CONFIG(DIR_IN, GPIO_MODE_IN_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(DIR_IN, GPIO_MODE_IN_FLOAT, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define IOCFG_IPU_25         IO_CONFIG(DIR_IN, GPIO_MODE_IN_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)

#define IO_CONFIG_GET_MODE(cfg)  (((cfg) >> 2) & 0x03)
#define IO_CONFIG_GET_SPEED(cfg) (((cfg) >> 4) & 0x03)
#define IO_CONFIG_GET_OTYPE(cfg) (((cfg) >> 0) & 0x03)
#define IO_CONFIG_GET_PULL(cfg)  (((cfg) >> 6) & 0x03)

#define SPI_IO_AF_CFG           IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(DIR_IN, GPIO_MODE_IN_FLOAT, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)

#define SPI_IO_CS_CFG           IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(DIR_IN, GPIO_MODE_IN_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)

#define SPIDEV_COUNT       4

#define CHECK_SPI_RX_DATA_AVAILABLE(instance)  (READ_BIT(instance->STATR, SPI_I2S_FLAG_RXNE) == (SPI_I2S_FLAG_RXNE))
#define SPI_RX_DATA_REGISTER(base) ((base)->DATAR)

#define MAX_SPI_PIN_SEL    5

#define UART_TX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE

#define PLATFORM_TRAIT_RCC      1
#define UART_TRAIT_AF_PIN       1
#define UART_TRAIT_TX_COMPLETE_PENDING 1
#define USE_PLATFORM_UART_IRQ_HANDLERS
#define SERIAL_TRAIT_PIN_CONFIG 1
#define I2C_TRAIT_AF_PIN        1
#define I2CDEV_COUNT            4
#define SPI_TRAIT_AF_PIN        1
#define UARTHARDWARE_MAX_PINS   5

#define UART_REG_RXD(base) (((USART_TypeDef *)(base))->DATAR)
#define UART_REG_TXD(base) (((USART_TypeDef *)(base))->DATAR)

#define DMA_TRAIT_MUX 1
#endif

#define DMA_CHANREQ_STRING "Request"

#define FLASH_CONFIG_BUFFER_TYPE      uint32_t
#define USB_DP_PIN PB8
