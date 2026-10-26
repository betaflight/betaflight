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
 * platform.h — NavQ95 (NXP i.MX 95, Cortex-M7) platform abstraction header.
 *
 * Pulls in CMSIS-Core only (ARM core_cm7.h, bundled in Betaflight under
 * lib/main/CMSIS/Core/Include) plus the few type shims the common code needs
 * (IRQn_Type, DMA_InitTypeDef). Deliberately free of the MCUXpresso fsl_*
 * drivers; peripheral drivers are added incrementally.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* ── CMSIS-Core (Cortex-M7) configuration ───────────────────────────────────
 * core_cm7.h expects these to be defined (normally by the SDK device header,
 * which we are intentionally not using yet). Values match the i.MX 95 M7.
 * Guarded with #ifndef because some (e.g. __FPU_PRESENT) are also passed on the
 * compiler command line via DEVICE_FLAGS; redefining would trip -Werror. */
#ifndef __CM7_REV
#define __CM7_REV              0x0100U   /* Core revision r1p0                 */
#endif
#ifndef __MPU_PRESENT
#define __MPU_PRESENT          1U        /* M7 has an MPU                      */
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
#define __NVIC_PRIO_BITS       4U        /* 16 priority levels                 */
#endif
#ifndef __Vendor_SysTickConfig
#define __Vendor_SysTickConfig 0U
#endif
#ifndef __FPU_PRESENT
#define __FPU_PRESENT          1U        /* double-precision FPU               */
#endif

/* Minimal IRQn_Type: core (negative) exceptions + the few external i.MX 95
 * IRQs we actually wire up. External IRQ numbers are the RAW NVIC external
 * index (0-based, after the 16 core exceptions), matching NuttX's
 * (IMX9_IRQ_<x> - IMX9_IRQ_EXTINT) i.e. (NuttX_value - 16). The full i.MX 95
 * IRQ map is added with the real drivers. */
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
    /* ── External (NVIC) IRQs — raw external index ──────────────────────── */
    LPUART1_IRQn          = 19,   /* NuttX IMX9_IRQ_LPUART1 (35) - 16 */
    LPUART2_IRQn          = 20,   /* NuttX IMX9_IRQ_LPUART2 (36) - 16 — console */
    LPUART3_IRQn          = 64,
    LPUART4_IRQn          = 65,
    LPUART5_IRQn          = 66,
    LPUART6_IRQn          = 67,
    LPUART7_IRQn          = 68,
    LPUART8_IRQn          = 69,
} IRQn_Type;

#include "core_cm7.h"

/* ── Type shims for Betaflight common code (replaced by real drivers) ─────── */
/* bus.h references DMA_InitTypeDef; placeholder until the DMA driver is ported. */
typedef struct { uint32_t placeholder; } DMA_InitTypeDef;
typedef struct { uint32_t placeholder; } DMA_HandleTypeDef;

/* Functional state — used by the generic timer/driver prototypes (e.g.
 * timerChannelConfigInterrupt(..., FunctionalState)). STM32-style enum. */
typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;

/* ── CMSIS SystemCoreClock ───────────────────────────────────────────────── */
/* Defined in platform_stubs.c; cli.c (cliStatus/cliEnv) references it. */
extern uint32_t SystemCoreClock;

/* ── NVIC priority grouping / build helpers ─────────────────────────────────
 * Betaflight's drivers/nvic.h builds interrupt priorities with
 * NVIC_BUILD_PRIORITY(base, sub); ATOMIC_BLOCK() uses the result. Same scheme
 * as the IMXRT port. */
#define NVIC_PriorityGroup_2            0x500
#define NVIC_PRIORITY_GROUPING          NVIC_PriorityGroup_2
#define NVIC_BUILD_PRIORITY(base, sub)  (((((base) << (4-(7-(NVIC_PRIORITY_GROUPING>>8)))) | \
                                          ((sub) & (0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))) << 4) & 0xf0)
#define NVIC_PRIORITY_BASE(prio)        (((prio) >> (4-(7-(NVIC_PRIORITY_GROUPING>>8)))) >> 4)
#define NVIC_PRIORITY_SUB(prio)         (((prio) >> 4) & (0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))

/* ── Chip unique ID ──────────────────────────────────────────────────────────
 * cli.c "mcu_id" prints U_ID_0..2. The i.MX 95 UID lives in OCOTP fuses, but
 * we are SDK-free in the skeleton (no OCOTP header). Return 0s for now; wire the
 * real fuse reads when the device header / OCOTP access is added. */
#ifndef U_ID_0
#define U_ID_0  (0U)
#define U_ID_1  (0U)
#define U_ID_2  (0U)
#endif

/* ── Bus instance counts ─────────────────────────────────────────────────────
 * The config parameter-group arrays (pg/bus_i2c.h, pg/bus_spi.h) are sized by
 * these. For I2C we use Option-A numbering (I2CDEV_1 -> LPI2C1, I2CDEV_6 ->
 * LPI2C6). With no USE_I2C_DEVICE_0, the i2cDevice_e enum numbers I2CDEV_1 = 0
 * ... I2CDEV_6 = 5, so the PG arrays must be sized for 6 slots (indices 0..5).
 * Only slots 0 (I2CDEV_1) and 5 (I2CDEV_6) are wired; 1..4 stay NONE/unused. */
#ifndef I2CDEV_COUNT
#define I2CDEV_COUNT                    6
#endif
#ifndef SPIDEV_COUNT
#define SPIDEV_COUNT                    1
#endif

/* SPI1 names the LPSPI1 instance. The generic SPI layer treats spiResource_t*
 * as an opaque handle and only compares/forwards it; the platform SPI driver
 * casts it back to the LPSPI base address. (i.MX 95 LPSPI1 = 0x44360000.) */
#ifndef SPI1
#define SPI1                            ((void *)0x44360000u)
#endif

/* I2Cn name the LPI2Cn instances. Like SPI1 above, the generic I2C layer treats
 * i2cResource_t* as an opaque handle; the platform I2C driver (bus_i2c_navq95.c)
 * casts it back to the LPI2C base address. Option-A bus numbering is used:
 * Betaflight I2CDEV_1 -> LPI2C1 and I2CDEV_6 -> LPI2C6, so the CLI/Configurator
 * bus number matches the silicon instance. (i.MX 95 LPI2C1 = 0x44340000 in the
 * AONMIX domain; LPI2C6 = 0x426C0000 in the WAKEUPMIX domain.) */
#ifndef I2C1
#define I2C1                            ((void *)0x44340000u)
#endif
#ifndef I2C6
#define I2C6                            ((void *)0x426C0000u)
#endif

/* ── Bus pin-selection table sizing ──────────────────────────────────────────
 * The generic bus impl headers size their per-bus pin-option arrays by these:
 *   bus_spi_impl.h:  spiPinDef_t sck/miso/mosiPins[MAX_SPI_PIN_SEL]
 *   bus_quadspi_impl.h / vtx soft-spi: MAX_QUADSPI_PIN_SEL
 *   serial_uart: UARTHARDWARE_MAX_PINS pin options per UART
 * One option per line is enough for the skeleton (real pin maps come with the
 * IOMUXC driver). Same values as the IMXRT port. */
#ifndef MAX_SPI_PIN_SEL
#define MAX_SPI_PIN_SEL                 1
#endif
#ifndef MAX_QUADSPI_PIN_SEL
#define MAX_QUADSPI_PIN_SEL             1
#endif
#ifndef UARTHARDWARE_MAX_PINS
#define UARTHARDWARE_MAX_PINS           1
#endif

/* ── Generic serial_uart.c platform contract ────────────────────────────────
 * The NavQ95 UART7 driver uses Betaflight's generic drivers/serial_uart.c core
 * (buffered TX/RX, IRQ wrappers, uartVTable) plus the i.MX 95 platform layer in
 * NXP/serial_uart_navq95.c. That core needs these platform traits:
 *   - UART_{TX,RX}_BUFFER_ATTRIBUTE: section attribute for the static UART ring
 *     buffers. Plain SRAM here (no DMA / special section yet), so empty.
 *   - UART_TRAIT_AF_PORT: the LPUART alt function is selected per-port over SCMI
 *     (one function per UART), not via a per-pin AF register, so AF_PORT (not
 *     AF_PIN). serial_uart_impl.h requires exactly one of AF_PIN / AF_PORT.
 *   - SERIAL_TRAIT_PIN_CONFIG: compile drivers/serial_uart_pinconfig.c, which
 *     maps the UART7_RX/TX_PIN tags into the serialPinConfig parameter group.
 * RCC / DMA / PINSWAP traits are intentionally left undefined (== 0): the i.MX
 * 95 LPUART needs no RCC gating from the M7 (the SM clocks it) and the driver is
 * interrupt-driven, not DMA. */
#ifndef UART_TX_BUFFER_ATTRIBUTE
#define UART_TX_BUFFER_ATTRIBUTE        /* none — plain SRAM */
#endif
#ifndef UART_RX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE        /* none — plain SRAM */
#endif
#ifndef UART_TRAIT_AF_PORT
#define UART_TRAIT_AF_PORT              1
#endif
#ifndef SERIAL_TRAIT_PIN_CONFIG
#define SERIAL_TRAIT_PIN_CONFIG         1
#endif

/* ── Misc capability defaults referenced by common config ──────────────────── */
#ifndef DEFAULT_CPU_OVERCLOCK
#define DEFAULT_CPU_OVERCLOCK           0
#endif

/* ── Scheduler timing ────────────────────────────────────────────────────────
 * fc/tasks.c sizes the gyro task period from TASK_GYROPID_DESIRED_PERIOD and
 * scheduler.c clamps reschedules to SCHEDULER_DELAY_LIMIT. Same values as the
 * RT1176 port (8 kHz gyro loop budget). */
#ifndef TASK_GYROPID_DESIRED_PERIOD
#define TASK_GYROPID_DESIRED_PERIOD     125     /* 125 µs = 8 kHz gyro loop */
#endif
#ifndef SCHEDULER_DELAY_LIMIT
#define SCHEDULER_DELAY_LIMIT           10
#endif

/* ── Timer alternate-function IO ─────────────────────────────────────────────
 * drivers/io.h only declares IOConfigGPIOAF() (and rx_pwm.c etc. only call it)
 * when USE_TIMER_AF is defined. The i.MX 95 pin mux is AF-based like the RT1176,
 * so enable it; the body is stubbed in platform_stubs.c until the real IOMUXC
 * driver lands. */
#ifndef USE_TIMER_AF
#define USE_TIMER_AF
#endif

/* ── Platform capability flags (minimal) ────────────────────────────────── */
#define USE_IO

/* ── IO configuration macros ────────────────────────────────────────────────
 * Betaflight generic bus/IO code (bus_i2c_utils.c, flash.c, bus_spi, ...) uses
 * the STM32-style IOCFG_* constants. The i.MX 95 pad config is encoded later by
 * the real IOMUXC driver; for the skeleton we pack a direction/speed/pull word
 * exactly like the RT1176 (IMXRT) and PICO ports do, so these macros resolve to
 * integers and the common code compiles. The packed value is interpreted by the
 * NavQ95 io driver when it is ported (ioConfigGPIO decodes it). */
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

/* GPIO pin state */
#define GPIO_PIN_RESET          (0U)
#define GPIO_PIN_SET            (1U)

/* i.MX 95 RGPIO ports are 32-bit (GPIO1..GPIO5). */
#define DEFIO_PORT_PINS         32

/* Fast-code / fast-data placement — no-ops until ITCM/DTCM layering is added. */
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
