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

// HPMicro platform definitions: IO configs, CMSIS/NVIC compatibility stubs,
// BASEPRI emulation, FAST_CODE placement and DMA channel resources.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform_mcu.h"
#include "io_types.h"
#include "io.h"
// HPMicro IO Configuration
#define IO_CONFIG(mode, speed, pupd) ((mode) | ((speed) << 2) | ((pupd) << 5))
#define IO_CONFIG_AF(mode, speed, pupd, af, bpio_func) \
    ((mode) | ((speed) << 2) | ((pupd) << 5) | ((af) << 8) | (bpio_func) << 16)

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_AF_OD,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_OD_UP       IO_CONFIG(GPIO_MODE_AF_OD,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_ANALOG         IO_CONFIG(GPIO_MODE_ANALOG,    GPIO_SPEED_FREQ_LOW, GPIO_NOPULL)

// GPIO base address lookup for HPMicro
#define IO_GPIO(io) ((GPIO_TypeDef *)(HPM_GPIO0_BASE))

// SPI configuration
#ifndef SPIDEV_COUNT
#define SPIDEV_COUNT 4
#endif
#define MAX_SPI_PIN_SEL 4
#define SPI_IO_CS_CFG IOCFG_OUT_PP_UP
#define SPI_IO_AF_SCK_CFG IOCFG_AF_PP_UP
#define I2CDEV_COUNT 3

// ARM CMSIS compatibility stubs for HPMicro
#define __ASM                   __asm__
#define __NOP()                 __asm volatile ("nop")
#define __disable_irq()         __asm volatile ("csrci mstatus, 8")
#define __enable_irq()          __asm volatile ("csrsi mstatus, 8")
#define NVIC_SystemReset()      systemReset()

// NVIC priority utility macros (same encoding as ESP32/PICO platforms;
// values are only consumed by ATOMIC_BLOCK on this platform)
#define NVIC_PriorityGroup_2         0x500
#define NVIC_PRIORITY_GROUPING NVIC_PriorityGroup_2
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0)
#define NVIC_PRIORITY_BASE(prio) (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4)
#define NVIC_PRIORITY_SUB(prio) (((prio)>>4)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))

struct dmaResource_s {
    DMA_Type *base;
    uint8_t channel;
};

// BASEPRI stubs - HPMicro (RISC-V) has no BASEPRI register.
// Emulated with the global interrupt enable bit (mstatus.MIE), so
// ATOMIC_BLOCK() keeps working: "priority" is the saved MIE state,
// set = disable global interrupts, restore = re-enable if previously on.
//
// __get_BASEPRI returns the raw MIE bit (0 or 8).  NVIC_PRIORITY_BASE on
// this platform yields 0..7, so values 1..7 are priority-set requests
// (→ disable IRQs) while value 8 is the saved MIE=1 state being restored
// (→ re-enable IRQs).  Value 0 leaves IRQs unchanged.
#include "hpm_interrupt.h"

__attribute__((always_inline))
static inline void __set_BASEPRI(uint32_t basePri)
{
    if (basePri == 0) {
        return;
    }
    if (basePri & CSR_MSTATUS_MIE_MASK) {
        // Restoring saved state: MIE was set → re-enable interrupts
        enable_global_irq(CSR_MSTATUS_MIE_MASK);
    } else {
        // Priority value (1..7) → disable interrupts
        disable_global_irq(CSR_MSTATUS_MIE_MASK);
    }
}

__attribute__((always_inline))
static inline uint32_t __get_BASEPRI(void)
{
    return read_csr(CSR_MSTATUS) & CSR_MSTATUS_MIE_MASK;
}

__attribute__((always_inline))
static inline void __set_BASEPRI_MAX(uint32_t basePri)
{
    (void) basePri;
    disable_global_irq(CSR_MSTATUS_MIE_MASK);
}

// Platform traits for HPMicro
#define PLATFORM_TRAIT_RCC 1
#define I2C_TRAIT_AF_PIN 1

// Place a function into the .fast section (ILM). Required for code that runs
// while the XPI NOR flash is unavailable (flash erase/program, or the flash
// recovery sequence before a software reset), where fetching instructions
// from XIP would return garbage. Functions marked with FAST_CODE must not call
// flash-resident code.
#define FAST_CODE __attribute__((section(".fast")))
// FAST_CODE_NOINLINE: places the function in ILM (.fast) and prevents inlining,
// which is essential for large hot-path functions like DShot DMA buffer loaders
// and telemetry decoders that would otherwise bloat their ILM-resident callers.
// On HPMicro all three macros place code in the same .fast section (ILM).
#define FAST_CODE_NOINLINE __attribute__((section(".fast"), noinline))
// i2cResource_t — HPMicro I2C register pointer (I2C_TypeDef is the HPM I2C type)

#define I2C_TRAIT_STATE 1
#define I2C_START I2C_NO_START

// HPM DMA channel identifiers (used by dma_reqmap_mcu.c)
// Each HPM DMA channel is identified by its DMA engine + channel number
extern struct dmaResource_s hdma_channels[8];
extern struct dmaResource_s xdma_channels[8];
#define HPM_HDMA_Channel0 &hdma_channels[0]
#define HPM_HDMA_Channel1 &hdma_channels[1]
#define HPM_HDMA_Channel2 &hdma_channels[2]
#define HPM_HDMA_Channel3 &hdma_channels[3]
#define HPM_HDMA_Channel4 &hdma_channels[4]
#define HPM_HDMA_Channel5 &hdma_channels[5]
#define HPM_HDMA_Channel6 &hdma_channels[6]
#define HPM_HDMA_Channel7 &hdma_channels[7]
#define HPM_XDMA_Channel0 &xdma_channels[0]
#define HPM_XDMA_Channel1 &xdma_channels[1]
#define HPM_XDMA_Channel2 &xdma_channels[2]
#define HPM_XDMA_Channel3 &xdma_channels[3]
#define HPM_XDMA_Channel4 &xdma_channels[4]
#define HPM_XDMA_Channel5 &xdma_channels[5]
#define HPM_XDMA_Channel6 &xdma_channels[6]
#define HPM_XDMA_Channel7 &xdma_channels[7]
#define UART_TRAIT_AF_PIN 1
#define PLATFORM_TRAIT_ADC_DEVICE 1
#define FLASH_CONFIG_BUFFER_TYPE uint32_t
// Write 32 bytes (8 words) per flash program call; a single config
// save then needs ~128 calls instead of ~1024 at the default 4 bytes.
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE 32

// asyncfatfs DMA cache in non-cacheable aligned memory (matching STM32/X32)
#if !defined(ENABLE_AFATFS_DMA_CACHE)
#define ENABLE_AFATFS_DMA_CACHE 1
#endif

// Stubs for features not (yet) supported on HPMicro
#define GPIO_PIN_RESET 0

// UART DMA buffer attributes for HPMicro
#define UART_TX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE
#define UARTHARDWARE_MAX_PINS 16
void IOConfigBPIOC(IO_t io);

// The current HPM6360 ADC16 and HPM6750 ADC12 backends expose no common set of
// VREFINT, temperature and VBAT/4 channels required by Betaflight's internal
// ADC API.  Reject the feature explicitly instead of leaving partial symbols.
#if defined(USE_ADC_INTERNAL)
#error "HPMicro: USE_ADC_INTERNAL is not supported by the current ADC backends"
#endif
