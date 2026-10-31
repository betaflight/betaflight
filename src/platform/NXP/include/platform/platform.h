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
 * NavQ95 (NXP i.MX 95, Cortex-M7) platform header. Uses CMSIS-Core only, with
 * no vendor SDK.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// CMSIS-Core configuration, normally provided by the device header.
#ifndef __CM7_REV
#define __CM7_REV              0x0100U
#endif
#ifndef __MPU_PRESENT
#define __MPU_PRESENT          1U
#endif
#ifndef __ICACHE_PRESENT
#define __ICACHE_PRESENT       1U
#endif
#ifndef __DCACHE_PRESENT
#define __DCACHE_PRESENT       1U
#endif
#ifndef __DTCM_PRESENT
#define __DTCM_PRESENT         1U
#endif
#ifndef __NVIC_PRIO_BITS
#define __NVIC_PRIO_BITS       4U
#endif
#ifndef __Vendor_SysTickConfig
#define __Vendor_SysTickConfig 0U
#endif
#ifndef __FPU_PRESENT
#define __FPU_PRESENT          1U
#endif

// External IRQ numbers are NVIC indices. Only the IRQs in use are listed.
typedef enum IRQn
{
    NonMaskableInt_IRQn   = -14,
    HardFault_IRQn        = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn         = -11,
    UsageFault_IRQn       = -10,
    SVCall_IRQn           = -5,
    DebugMonitor_IRQn     = -4,
    PendSV_IRQn           = -2,
    SysTick_IRQn          = -1,

    LPUART1_IRQn          = 19,
    LPUART2_IRQn          = 20,
    LPUART3_IRQn          = 64,
    LPUART4_IRQn          = 65,
    LPUART5_IRQn          = 66,
    LPUART6_IRQn          = 67,
    LPUART7_IRQn          = 68,
    LPUART8_IRQn          = 69,
} IRQn_Type;

#include "core_cm7.h"

// Types referenced by common code; there is no DMA driver yet.
typedef struct { uint32_t placeholder; } DMA_InitTypeDef;
typedef struct { uint32_t placeholder; } DMA_HandleTypeDef;

typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;

extern uint32_t SystemCoreClock;

#define NVIC_PriorityGroup_2            0x500
#define NVIC_PRIORITY_GROUPING          NVIC_PriorityGroup_2
#define NVIC_BUILD_PRIORITY(base, sub)  (((((base) << (4-(7-(NVIC_PRIORITY_GROUPING>>8)))) | \
                                          ((sub) & (0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))) << 4) & 0xf0)
#define NVIC_PRIORITY_BASE(prio)        (((prio) >> (4-(7-(NVIC_PRIORITY_GROUPING>>8)))) >> 4)
#define NVIC_PRIORITY_SUB(prio)         (((prio) >> 4) & (0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))

// The unique ID is in OCOTP fuses, which are not read yet.
#ifndef U_ID_0
#define U_ID_0  (0U)
#define U_ID_1  (0U)
#define U_ID_2  (0U)
#endif

// I2CDEV_n matches LPI2Cn. Only I2CDEV_1 and I2CDEV_6 are used, at indices 0 and 5.
#ifndef I2CDEV_COUNT
#define I2CDEV_COUNT                    6
#endif
#ifndef SPIDEV_COUNT
#define SPIDEV_COUNT                    1
#endif

// Bus handles are the peripheral base addresses.
#ifndef SPI1
#define SPI1                            ((void *)0x44360000u)
#endif

#ifndef I2C1
#define I2C1                            ((void *)0x44340000u)
#endif
#ifndef I2C6
#define I2C6                            ((void *)0x426C0000u)
#endif

#ifndef MAX_SPI_PIN_SEL
#define MAX_SPI_PIN_SEL                 1
#endif
#ifndef MAX_QUADSPI_PIN_SEL
#define MAX_QUADSPI_PIN_SEL             1
#endif
#ifndef UARTHARDWARE_MAX_PINS
#define UARTHARDWARE_MAX_PINS           1
#endif

// Traits for drivers/serial_uart.c. The LPUART function is selected per port over SCMI.
#ifndef UART_TX_BUFFER_ATTRIBUTE
#define UART_TX_BUFFER_ATTRIBUTE
#endif
#ifndef UART_RX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE
#endif
#ifndef UART_TRAIT_AF_PORT
#define UART_TRAIT_AF_PORT              1
#endif
#ifndef SERIAL_TRAIT_PIN_CONFIG
#define SERIAL_TRAIT_PIN_CONFIG         1
#endif

#ifndef DEFAULT_CPU_OVERCLOCK
#define DEFAULT_CPU_OVERCLOCK           0
#endif

#ifndef TASK_GYROPID_DESIRED_PERIOD
#define TASK_GYROPID_DESIRED_PERIOD     125
#endif
#ifndef SCHEDULER_DELAY_LIMIT
#define SCHEDULER_DELAY_LIMIT           10
#endif

#ifndef USE_TIMER_AF
#define USE_TIMER_AF
#endif

#define USE_IO

// ioConfig_t packing, decoded by io_navq95.c.
#define GPIO_MODE_INPUT         0x00U
#define GPIO_MODE_OUTPUT        0x01U
#define GPIO_MODE_AF            0x02U
#define GPIO_MODE_ANALOG        0x03U

#define GPIO_SPEED_LOW          0x00U
#define GPIO_SPEED_HIGH         0x01U

#define GPIO_NOPULL             0x00U
#define GPIO_PULLUP             0x01U
#define GPIO_PULLDOWN           0x02U

#define IO_CONFIG(mode, speed, pupd)    ((mode) | ((speed) << 2) | ((pupd) << 5))

#define IOCFG_OUT_PP            IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH, GPIO_NOPULL)
#define IOCFG_OUT_OD            IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH, GPIO_NOPULL)
#define IOCFG_AF_PP             IO_CONFIG(GPIO_MODE_AF,     GPIO_SPEED_HIGH, GPIO_NOPULL)
#define IOCFG_AF_OD             IO_CONFIG(GPIO_MODE_AF,     GPIO_SPEED_HIGH, GPIO_NOPULL)
#define IOCFG_IPD               IO_CONFIG(GPIO_MODE_INPUT,  GPIO_SPEED_LOW,  GPIO_PULLDOWN)
#define IOCFG_IPU               IO_CONFIG(GPIO_MODE_INPUT,  GPIO_SPEED_LOW,  GPIO_PULLUP)
#define IOCFG_IN_FLOATING       IO_CONFIG(GPIO_MODE_INPUT,  GPIO_SPEED_LOW,  GPIO_NOPULL)

#define SPI_IO_CS_CFG           IOCFG_OUT_PP
#define SPI_IO_CS_HIGH_CFG      IOCFG_IPU

#define GPIO_PIN_RESET          (0U)
#define GPIO_PIN_SET            (1U)

#define DEFIO_PORT_PINS         32

// No ITCM/DTCM placement yet.
#ifndef FAST_CODE
#define FAST_CODE
#endif
#ifndef FAST_DATA
#define FAST_DATA
#endif
#ifndef FAST_CODE_NOINLINE
#define FAST_CODE_NOINLINE
#endif
#ifndef DMA_DATA
#define DMA_DATA
#endif
#ifndef DMA_DATA_ZERO_INIT
#define DMA_DATA_ZERO_INIT
#endif
#ifndef STATIC_DMA_DATA_AUTO
#define STATIC_DMA_DATA_AUTO    static DMA_DATA
#endif
#ifndef FAST_IRQ_HANDLER
#define FAST_IRQ_HANDLER
#endif
