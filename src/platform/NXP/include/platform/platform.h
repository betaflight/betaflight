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

// Vendor CMSIS device header (pulls in core_cm7.h) plus the feature header that
// names the die's options and errata, which the device header does not include.
#if defined(MIMXRT1064)
#include "MIMXRT1064.h"
#include "MIMXRT1064_features.h"
#elif defined(MIMXRT1062)
#include "MIMXRT1062.h"
#include "MIMXRT1062_features.h"
#elif defined(MIMXRT1176)
// This port runs on the CM7 and holds the CM4 in reset.
#include "MIMXRT1176_cm7.h"
#include "MIMXRT1176_cm7_features.h"
#else
#error "Unsupported i.MXRT device"
#endif

#include "platform/nxp_family.h"

#define PLATFORM_NO_LIBC             0

// STM32 standard peripheral library encoding: the high byte holds 7 minus the
// number of preemption bits, so 0x500 is two preemption and two subpriority bits.
#define NVIC_PriorityGroup_2         0x500

// Each GPIO port holds 32 pads: ports A..E on the 10xx, A..G on the 11xx.
#define DEFIO_PORT_PINS              32

// Leftover STM32-era declarations that generic src/main code still names. None
// of them implies the corresponding STM32 peripheral exists.
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

#define GPIO_PIN_RESET       0

#define DMA_TypeDef          void*
#define DMA_InitTypeDef      void*

// Generic code holds these as opaque resource pointers; the drivers cast back
// to the vendor LPSPI_Type / LPI2C_Type / LPUART_Type.
#define SPI_INST(spi)        ((LPSPI_Type *)(spi))
#define I2C_INST(i2c)        ((LPI2C_Type *)(i2c))
#define UART_INST(uart)      ((LPUART_Type *)(uart))

#define SPI1                 ((spiResource_t *)LPSPI1)
#define SPI2                 ((spiResource_t *)LPSPI2)
#define SPI3                 ((spiResource_t *)LPSPI3)
#define SPI4                 ((spiResource_t *)LPSPI4)

#define I2C1                 ((i2cResource_t *)LPI2C1)
#define I2C2                 ((i2cResource_t *)LPI2C2)
#define I2C3                 ((i2cResource_t *)LPI2C3)
#define I2C4                 ((i2cResource_t *)LPI2C4)

// A board enables a FlexCAN controller by naming CANx_TX_PIN and CANx_RX_PIN.
#define ENABLE_CAN 1

#if defined(IMXRT11XX)
// The 11xx headers name the single DMAMUX after its index.
#define DMAMUX               DMAMUX0

// The die also has LPSPI5/6 and LPI2C5/6; SPIDEV_COUNT and I2CDEV_COUNT are 4.
#endif

// ioConfig_t packs mode, open drain (pad ODE), pull, drive and hysteresis into
// one byte; IOConfigGPIO() in io_imxrt.c expands it into a pad control word.
#define IOCFG_MODE_IN        0
#define IOCFG_MODE_OUT       1
#define IOCFG_MODE_AF        2
#define IOCFG_MODE_ANALOG    3

#define IOCFG_PULL_NONE      0
#define IOCFG_PULL_UP        1
#define IOCFG_PULL_DOWN      2
#define IOCFG_PULL_KEEPER    3

#define IOCFG_DRIVE_SLOW     0
#define IOCFG_DRIVE_MEDIUM   1
#define IOCFG_DRIVE_FAST     2
#define IOCFG_DRIVE_MAX      3

#define IO_CONFIG(mode, drive, pull, od, hys) \
    ((ioConfig_t)(((mode) & 0x03) | (((od) & 0x01) << 2) | (((pull) & 0x03) << 3) | \
                  (((drive) & 0x03) << 5) | (((hys) & 0x01) << 7)))

#define IOCFG_MODE(cfg)      ((cfg) & 0x03)
#define IOCFG_OD(cfg)        (((cfg) >> 2) & 0x01)
#define IOCFG_PULL(cfg)      (((cfg) >> 3) & 0x03)
#define IOCFG_DRIVE(cfg)     (((cfg) >> 5) & 0x03)
#define IOCFG_HYS(cfg)       (((cfg) >> 7) & 0x01)

#define IOCFG_OUT_PP         IO_CONFIG(IOCFG_MODE_OUT, IOCFG_DRIVE_FAST, IOCFG_PULL_NONE, 0, 0)
#define IOCFG_OUT_OD         IO_CONFIG(IOCFG_MODE_OUT, IOCFG_DRIVE_MEDIUM, IOCFG_PULL_NONE, 1, 0)
#define IOCFG_AF_PP          IO_CONFIG(IOCFG_MODE_AF, IOCFG_DRIVE_FAST, IOCFG_PULL_NONE, 0, 1)
#define IOCFG_AF_PP_UP       IO_CONFIG(IOCFG_MODE_AF, IOCFG_DRIVE_FAST, IOCFG_PULL_UP, 0, 1)
#define IOCFG_AF_PP_PD       IO_CONFIG(IOCFG_MODE_AF, IOCFG_DRIVE_FAST, IOCFG_PULL_DOWN, 0, 1)
#define IOCFG_AF_OD          IO_CONFIG(IOCFG_MODE_AF, IOCFG_DRIVE_MEDIUM, IOCFG_PULL_NONE, 1, 1)
#define IOCFG_IPU            IO_CONFIG(IOCFG_MODE_IN, IOCFG_DRIVE_SLOW, IOCFG_PULL_UP, 0, 1)
#define IOCFG_IPD            IO_CONFIG(IOCFG_MODE_IN, IOCFG_DRIVE_SLOW, IOCFG_PULL_DOWN, 0, 1)
#define IOCFG_IN_FLOATING    IO_CONFIG(IOCFG_MODE_IN, IOCFG_DRIVE_SLOW, IOCFG_PULL_NONE, 0, 1)
#define IOCFG_ANALOG         IO_CONFIG(IOCFG_MODE_ANALOG, IOCFG_DRIVE_SLOW, IOCFG_PULL_NONE, 0, 0)

#define SPI_IO_AF_CFG           IOCFG_AF_PP
#define SPI_IO_AF_SCK_CFG_HIGH  IOCFG_AF_PP_UP
#define SPI_IO_AF_SCK_CFG_LOW   IOCFG_AF_PP_PD
#define SPI_IO_AF_SDI_CFG       IOCFG_AF_PP_UP
#define SPI_IO_CS_CFG           IOCFG_OUT_PP
#define SPI_IO_CS_HIGH_CFG      IOCFG_IPU

// Common code is placed by file in link/nxp_itcm256k_sections.ld, platform code by these marks.
// DMA buffers in DTCM need no cache maintenance.
#define FAST_CODE                       __attribute__((section(".itcm_text")))
#define FAST_CODE_NOINLINE              __attribute__((noinline))
#define FAST_CODE_NOINLINE_CRITICAL     __attribute__((section(".itcm_text"), noinline))
#define FAST_IRQ_HANDLER                __attribute__((section(".itcm_text")))

#define DMA_DATA_ZERO_INIT              __attribute__((section(".dtcm_bss"), aligned(32)))
#define DMA_DATA                        __attribute__((section(".dtcm_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO            static DMA_DATA

// Hot variables in DTCM, word aligned like the STM32 fastram sections.
#define FAST_DATA_ZERO_INIT             __attribute__((section(".dtcm_bss"), aligned(4)))
#define FAST_DATA                       __attribute__((section(".dtcm_data"), aligned(4)))

// Hot constants in DTCM; a section of their own, since GCC rejects const and writable objects in one section.
#define FAST_RODATA                     __attribute__((section(".dtcm_rodata")))

// Code that must not fetch from the flash it is erasing; linked into ITCM.
#define RAMFUNC                         __attribute__((section(".ramfunc")))

#define DEFAULT_CPU_OVERCLOCK           0

// 125us = 8kHz
#define TASK_GYROPID_DESIRED_PERIOD     125
#define SCHEDULER_DELAY_LIMIT           10

#define USE_LATE_TASK_STATISTICS

// USDHC DMA bypasses the data cache, so asyncfatfs keeps its sector cache in a
// separate 32 byte aligned DMA buffer.
#if !defined(ENABLE_AFATFS_DMA_CACHE)
#define ENABLE_AFATFS_DMA_CACHE 1
#endif

// LPUART and LPSPI are named from one, matching Betaflight's default 1-based
// `resource` indices, so USE_RESOURCE_INDEX_FROM_ZERO must stay undefined.
#define SERIAL_UART_FIRST_INDEX     1
#define SERIAL_TRAIT_PIN_CONFIG     1

#define UART_TX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE

// 96 bit chip identifier: the 64 bit OCOTP UNIQUE_ID plus the next fuse word.
#if defined(IMXRT11XX)
#define U_ID_0 (OCOTP->FUSEN[0x10].FUSE)
#define U_ID_1 (OCOTP->FUSEN[0x11].FUSE)
#define U_ID_2 (OCOTP->FUSEN[0x12].FUSE)
#elif defined(IMXRT10XX)
#define U_ID_0 (OCOTP->CFG0)
#define U_ID_1 (OCOTP->CFG1)
#define U_ID_2 (OCOTP->CFG2)
#else
#error "Unsupported i.MXRT family: no unique device ID fuse shadows"
#endif

extern uint32_t SystemCoreClock;

// Startup helpers, called from the family's startup file before main().
void nxpClockInit(void);
void nxpMpuInit(void);

#ifndef DEFAULT_VOLTAGE_METER_SCALE
// 100 = 1.00x (100%) scaling; override per target/board to match its VBAT divider
#define DEFAULT_VOLTAGE_METER_SCALE   100
#endif

#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER

// The FlexIO DShot driver reports its answer rate search in the CLI telemetry.
#define PLATFORM_TRAIT_DSHOT_TRAINING 1

// Each FlexCAN instance reaches at most four pads.
#define CAN_MAX_PIN_SEL 4

// persistent.c keeps these in the SNVS low power general purpose registers,
// which survive a reset and, with a backup cell fitted, a power cycle.
#define USE_PERSISTENT_OBJECTS

#define NVIC_PRIORITY_GROUPING NVIC_PriorityGroup_2
#define NVIC_BUILD_PRIORITY(base,sub) \
    (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0)

// Converts an NVIC_BUILD_PRIORITY() value (BASEPRI layout) into the argument
// CMSIS NVIC_SetPriority() takes; always set an NVIC priority through this.
#define NVIC_PRIORITY_LEVEL(prio) ((uint32_t)(prio) >> (8 - __NVIC_PRIO_BITS))
