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
 * ESP32-S3 peripheral register definitions.
 *
 * The ESP-IDF SOC struct headers declare peripheral register structs as
 * extern symbols (e.g. "extern gpio_dev_t GPIO;").  These symbols must
 * be defined and placed at the correct memory-mapped addresses so the
 * linker can resolve them.
 *
 * We define them here as pointers cast from the documented base addresses
 * in the ESP32-S3 Technical Reference Manual (matched by reg_base.h).
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/* Prevent our platform.h macros from conflicting with ESP-IDF externs */
#include "platform.h"
#undef UART0
#undef UART1
#undef UART2
#undef I2C0
#undef I2C1
#undef SPI0
#undef SPI1

#include "soc/gpio_struct.h"
#include "soc/uart_struct.h"
#include "soc/spi_struct.h"
#include "soc/i2c_struct.h"
#include "soc/rmt_struct.h"
#include "soc/ledc_struct.h"
#include "soc/sens_struct.h"
#include "soc/reg_base.h"

#if defined(ESP32S3)
#include "soc/systimer_struct.h"
#include "soc/system_struct.h"
#include "soc/usb_serial_jtag_struct.h"
#include "soc/gdma_struct.h"
#else
#include "soc/timer_group_struct.h"
#endif

#pragma GCC diagnostic pop

/* Each peripheral symbol is placed at the base address from reg_base.h.
 * The linker resolves e.g. &GPIO -> DR_REG_GPIO_BASE (0x60004000). */
gpio_dev_t              GPIO                __attribute__((section(".peripheral_gpio")))         = {};
uart_dev_t              UART0               __attribute__((section(".peripheral_uart0")))        = {};
uart_dev_t              UART1               __attribute__((section(".peripheral_uart1")))        = {};
uart_dev_t              UART2               __attribute__((section(".peripheral_uart2")))        = {};
#if defined(ESP32S3)
spi_dev_t               GPSPI2              __attribute__((section(".peripheral_spi2")))         = {};
spi_dev_t               GPSPI3              __attribute__((section(".peripheral_spi3")))         = {};
#else
/* SPI1 is the internal flash SPI, SPI2=HSPI, SPI3=VSPI */
spi_dev_t               SPI1                __attribute__((section(".peripheral_spi1")))         = {};
spi_dev_t               SPI2                __attribute__((section(".peripheral_spi2")))         = {};
spi_dev_t               SPI3                __attribute__((section(".peripheral_spi3")))         = {};
#endif
i2c_dev_t               I2C0                __attribute__((section(".peripheral_i2c0")))         = {};
i2c_dev_t               I2C1                __attribute__((section(".peripheral_i2c1")))         = {};
rmt_dev_t               RMT                 __attribute__((section(".peripheral_rmt")))          = {};
ledc_dev_t              LEDC                __attribute__((section(".peripheral_ledc")))         = {};
sens_dev_t              SENS                __attribute__((section(".peripheral_sens")))         = {};

#if defined(ESP32S3)
systimer_dev_t          SYSTIMER            __attribute__((section(".peripheral_systimer")))     = {};
/* RMTMEM is 4 TX + 4 RX channels × 48 items each = 384 items × 4 bytes */
uint32_t                RMTMEM[384]         __attribute__((section(".peripheral_rmtmem")))       = {};
system_dev_t            SYSTEM              __attribute__((section(".peripheral_system")))       = {};
usb_serial_jtag_dev_t   USB_SERIAL_JTAG     __attribute__((section(".peripheral_usb_serial")))  = {};
gdma_dev_t              GDMA                __attribute__((section(".peripheral_gdma")))         = {};
#else
timg_dev_t              TIMERG0             __attribute__((section(".peripheral_timg0")))        = {};
/* ESP32 has 8 bidirectional RMT channels × 64 items each = 512 items × 4 bytes */
uint32_t                RMTMEM[512]         __attribute__((section(".peripheral_rmtmem")))       = {};
#endif

/* Restore platform macros */
#define UART0 (&esp32UartDev0)
#define UART1 (&esp32UartDev1)
#define UART2 (&esp32UartDev2)
#define I2C0  (&esp32I2cDev0)
#define I2C1  (&esp32I2cDev1)
#define SPI0  (&esp32SpiDev0)
#define SPI1  (&esp32SpiDev1)
